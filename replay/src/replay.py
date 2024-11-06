import time, argparse, json, threading
import numpy as np
import math
from serial_emulator import SerialEmulator
from decoded_messages import DecodedMessage
from visualizer import Visualizer
import os
import cantools, can

CLUSTER_TSHOLD_MS = 20 # In [ms]

def compare_floats(a, b):
    return math.isclose(a, b, rel_tol=1e-8)

def filter_by_start_time(data, start_time):
    start_time_micseconds = start_time
    assert start_time_micseconds < data[-1]["timestamp"], "The start time is greater than the last timestamp in the file"
    return list(filter(lambda x: x["timestamp"] >= start_time_micseconds, data))

def set_ubx_flag(ubx_type):
    if ubx_type is not None:
        if ubx_type == "NAV-PVT":
            return True, False, False, False
        elif ubx_type == "NAV-ATT":
            return False, True, False, False
        elif ubx_type == "ESF-INS":
            return False, False, True, False
        elif ubx_type == "ESF-RAW":
            return False, False, False, True
        else:
            # print("Unknown UBX message type:", ubx_type)
            return False, False, False, False
        
def manage_map(fifo_path, latitude, longitude, heading, server_ip, server_port, visualizer, map_opened):
    try:
        if not map_opened:
            # Open the map GUI after the nodejs server is ready
            fp = open(fifo_path, 'r')
            info = fp.read()
            if "ready" not in info:
                raise Exception("Error opening map GUI")
            visualizer.open_map_gui(latitude, longitude, server_ip, server_port)
            map_opened = True
            print("It is possible to open the map GUI at http://localhost:8080")
    except Exception as e:
        print(f"Error opening map GUI: {e}")
        raise e
    try:
        # Send the new object position to the server that will update the map GUI
        visualizer.send_object_udp_message(latitude, longitude, heading, server_ip, server_port)
    except Exception as e:
        print(f"Error sending UDP message: {e}")
        raise e
    
def print_test_rate_stats(average_update_time, average_update_time_filtered, ubx_nav_pvt_present, ubx_nav_att_present, ubx_esf_ins_present, ubx_esf_raw_present):
    print("Average update periodicity:", average_update_time, "ms")

    print("Average update rate (filtered):", 1e3/average_update_time_filtered, "Hz")
    print("Average update periodicity (filtered):", average_update_time_filtered, "ms")

    print("UBX messages statistics:")
    print("UBX-NAV-PVT present:",ubx_nav_pvt_present)
    print("UBX-NAV-ATT present:",ubx_nav_att_present)
    print("UBX-ESF-INS present:",ubx_esf_ins_present)
    print("UBX-ESF-RAW present:",ubx_esf_raw_present)

def serial_test_rate(server_device, client_device, baudrate, filename, start_time, end_time, gui, serial, test_rate, server_ip, server_port, fifo_path, visualizer):
    """
    Writes the data from the file to the serial device or does a test rate analysis.
    """
    try:
        if serial:
            # Creation of the serial emulator
            ser = SerialEmulator(device_port=server_device, client_port=client_device, baudrate=baudrate)
        decoder = DecodedMessage()
        f = open(filename, "r")
        data = json.load(f)
        f.close()

        if start_time:
            data = filter_by_start_time(data, start_time)

        previous_time = 0 if not start_time else start_time
        map_opened = False

        latitude = None
        longitude = None    
        heading = None
        before_time = 0
        after_time = 0
        variable_delta_us_factor = 0
        previous_pos_time = previous_time
        delta_pos_time = 0
        average_update_time = 0
        cnt_update_time = 0
        average_update_time_filtered = 0
        cnt_update_time_filtered = 0
        update_timestamps = ["Timestamp_ms"]
        update_peridocities = ["Peridocity_ms"]
        update_rates = ["Rate_Hz"]
        update_msg_type = ["Message_type"]
        update_msg_clustered = ["Clustered"]
        update_msg_same_position = ["Same_pos_as_previous"]
        update_msg_lat = ["Latitude"]
        update_msg_lon = ["Longitude"]

        prev_latitude_deg = -8000
        prev_longitude_deg = -8000

        ubx_nav_pvt_present=False
        ubx_nav_att_present=False
        ubx_esf_ins_present=False
        ubx_esf_raw_present=False

        first_send = None
        startup_time = time.time() * 1e6
        for d in data:
            if not gui and not serial and not test_rate:
                break
            delta_time = d["timestamp"] - previous_time
            message_type = d["type"]
            if message_type == "Unknown":
                continue
            content = d["data"]
            if message_type == "UBX":
                content = bytes.fromhex(content)
                if gui:
                    tmp_lat, tmp_lon, tmp_heading = decoder.extract_data(content, message_type)
                    if tmp_lat:
                        latitude = tmp_lat
                    if tmp_lon:
                        longitude = tmp_lon
                    if tmp_heading:
                        heading = tmp_heading
            else:
                # For the NMEA messages we need to encode the content for the serial emulator and decode it for the GUI (to obtain a string)
                content = content.encode()
                if gui:
                    tmp_lat, tmp_lon, tmp_heading = decoder.extract_data(content.decode(), message_type)
                    if tmp_lat:
                        latitude = tmp_lat
                    if tmp_lon:
                        longitude = tmp_lon
                    if tmp_heading:
                        heading = tmp_heading

            if test_rate:
                test_rate_lat = None
                test_rate_lon = None
                if message_type == "UBX":
                    # Check the UBX message type
                    ubx_type = decoder.get_ubx_message_type(content)
                    ubx_nav_pvt_present, ubx_nav_att_present, ubx_esf_ins_present, ubx_esf_raw_present = set_ubx_flag(ubx_type)
                    tmp_lat, tmp_lon, _ = decoder.extract_data(content, message_type)
                else:
                    tmp_lat, tmp_lon, _ = decoder.extract_data(content.decode(), message_type)
                if tmp_lat:
                    test_rate_lat = tmp_lat
                if tmp_lon:
                    test_rate_lon = tmp_lon
                
                if not test_rate_lat or not test_rate_lon:
                    continue

                delta_pos_time = d["timestamp"] - previous_pos_time

                if message_type == "UBX":
                    print("Time since last update (UBX):", delta_pos_time/1e3, "Time:", d["timestamp"]/1e3)
                else:
                    print("Time since last update (NMEA):", delta_pos_time/1e3, "Time:", d["timestamp"]/1e3)
                
                print("Latitude [deg]:", test_rate_lat, "Longitude [deg]:", test_rate_lon)

                previous_pos_time = d["timestamp"]

                cnt_update_time = cnt_update_time + 1
                average_update_time = average_update_time + (delta_pos_time/1e3-average_update_time) / cnt_update_time
                
                # Check if the position update is clustered or not (based on the time and position)
                if delta_pos_time/1e3 > CLUSTER_TSHOLD_MS or not compare_floats(prev_latitude_deg, test_rate_lat) or not compare_floats(prev_longitude_deg, test_rate_lon):
                    cnt_update_time_filtered = cnt_update_time_filtered + 1
                    average_update_time_filtered = average_update_time_filtered + (delta_pos_time/1e3-average_update_time_filtered) / cnt_update_time_filtered
                    update_msg_clustered.append(0)
                else:
                    update_msg_clustered.append(1)

                if compare_floats(prev_latitude_deg, test_rate_lat) and compare_floats(prev_longitude_deg, test_rate_lon):
                    update_msg_same_position.append(1)
                else:
                    update_msg_same_position.append(0)

                prev_latitude_deg = test_rate_lat
                prev_longitude_deg = test_rate_lon

                update_timestamps.append(d["timestamp"]/1e3)
                update_peridocities.append(delta_pos_time/1e3)
                update_rates.append(1e6/(delta_pos_time))
                if message_type == "UBX":
                    update_msg_type.append("UBX-NAV-PVT")
                else:
                    update_msg_type.append("NMEA-Gx" + content[3:6].decode())
                update_msg_lat.append(test_rate_lat)
                update_msg_lon.append(test_rate_lon)
            else:
                # Calculate a variable delta time factor to adjust the time of the serial write to be as close as possible to a real time simulation
                # delta_time_us represents the real time in microseconds from the beginning of the simulation to the current time
                delta_time_us_real = time.time() * 1e6 - startup_time
                # start_time_us represents the time in microseconds from the beginning of the messages simulation to the start time selected by the user
                start_time_us = start_time if start_time else 0
                # delta_time_us_simulation represents the time in microseconds from the beginning of the messages simulation time to the current message time
                delta_time_us_simulation = d["timestamp"] - start_time_us
                # We want that the time of the serial write is as close as possible to the real time simulation
                # variable_delta_us_factor represents the difference between the simulation time and the real time
                # It should be as close as possible to 0 and it is used to adjust the waiting time for the serial write
                variable_delta_us_factor = delta_time_us_simulation - delta_time_us_real
                try:
                    # Wait for the real time to be as close as possible to the simulation time
                    time.sleep(variable_delta_us_factor / 1e6)
                except:
                    print("Trying to sleep for a negative time, thus not sleeping: ", variable_delta_us_factor / 1e3)
                if serial:
                    ser.write(content)
                    if first_send is None:
                        first_send = time.time()
            if gui and latitude and longitude:
                manage_map(fifo_path, latitude, longitude, heading, server_ip, server_port, visualizer, map_opened)
            previous_time = d["timestamp"]
            if end_time and time.time() * 1e6 - startup_time > end_time:
                break
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if serial:
            ser.stop()
            print("Time to send all messages:", time.time() - first_send, "s")
            print("Difference to the last message:", time.time() - first_send - d["timestamp"] / 1e6, "s")
        if gui:
            visualizer.stop_server(server_ip, server_port)
        if test_rate:
            print_test_rate_stats(average_update_time, average_update_time_filtered, ubx_nav_pvt_present, ubx_nav_att_present, ubx_esf_ins_present, ubx_esf_raw_present)
            print("Saving data to replay_out.csv...")
            np.savetxt('replay_out.csv', [p for p in zip(update_timestamps, update_msg_type, update_peridocities, update_rates, update_msg_clustered, update_msg_lat, update_msg_lon, update_msg_same_position)], delimiter=',', fmt='%s')

def write_CAN(device, filename, db_file, start_time, end_time, gui, visualizer, server_ip, server_port):
    """
    Writes the data from the file to the CAN device.
    """
    try:
        first_send = None
        f = open(filename, "r")
        data = json.load(f)
        f.close()
        # Filter the data by the start time
        if start_time:
            data = filter_by_start_time(data, start_time)
        # Load the CAN database
        db = cantools.database.load_file(db_file)
        # Create the CAN bus
        bus = can.interface.Bus(channel=device, interface='socketcan')
        previous_time = 0 if not start_time else start_time
        variable_delta_us_factor = 0
        startup_time = time.time() * 1e6
        for d in data:
            delta_time = d["timestamp"] - previous_time
            arbitration_id = d["arbitration_id"]
            content = d["data"]
            # Get the message from the CAN database
            message = db.get_message_by_frame_id(arbitration_id)
            if message:
                # Encode the content of the message
                data = db.encode_message(message.frame_id, content)
                final_message = can.Message(arbitration_id=message.frame_id, data=data)
            start_time_us = start_time if start_time else 0
            # Calculate the delta time in the recording between the current message and the start time
            delta_time_us_simulation = d["timestamp"] - start_time_us
            # Calculate the real time in microseconds from the beginning of the simulation to the current time
            delta_time_us_real = time.time() * 1e6 - startup_time
            # Update the variable_delta_time_us_factor to adjust the time of the CAN write to be as close as possible to a real time simulation
            variable_delta_us_factor = delta_time_us_simulation - delta_time_us_real
            try:
                # Wait for the real time to be as close as possible to the simulation time
                time.sleep(variable_delta_us_factor / 1e6)
            except:
                print("Trying to sleep for a negative time, thus not sleeping: ", variable_delta_us_factor / 1e3)
            if message:
                # Write the message to the CAN bus
                if first_send is None:
                    first_send = time.time()
                bus.send(final_message)
            if gui:
                # TODO implement the visualization of the CAN messages
                pass
            previous_time = d["timestamp"]
            if end_time and time.time() * 1e6 - startup_time > end_time:
                break
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()
        print("Time to send all messages:", time.time() - first_send)
        print("Difference to the last message:", time.time() - first_send - d["timestamp"] / 1e6)
        if gui:
            visualizer.stop_server(server_ip, server_port)

def main():
    """
    This script reads a json file with the following format:
    
    [
        {
            "timestamp": <time in microseconds>,
            "type": <message type>,
            "data": <message data>
        },
        ...
    ]

    and writes the data to a serial device.

    Command-line Arguments:
    - --enable_serial (bool): Whether to enable the serial emulator. Default is False. Can be activated by writing it.
    - --serial_filename (str): The file to read data from. Default is "./data/examples/example1.json".
    - --server_device (str): The device to write data to. Default is "./replay/ttyNewServer".
    - --client_device (str): The device to read data from. Default is "./replay/ttyNewClient".
    - --baudrate (int): The baudrate to write. Default is 115200.
    - --start_time (int): The timestamp to start the reading in seconds. If not specified, will read from the beginning of the file.
    - --end_time (int): The time to stop reading in seconds. If not specified, will write until the end of the file.
    - --enable_gui (bool): Whether to display the GUI. Default is False. Can be activated by writing it.
    - --enable_test_rate (int): Test rate mode. Instead of showing the trace or reproducing it, it will output the positioning (Lat, Lon) update frequency and save the related data, message by message, on a file named replay_out.csv. Default is False. Can be activated by writing it.
    - --httpport (int): The port for the HTTP server. Default is 8080.
    - --server_ip (str): The IP address of the server. Default is
    - --server_port (int): The port of the server. Default is 48110.
    - --enable_CAN (bool): Whether to enable the CAN emulator. Default is False. Can be activated by writing it.
    - --CAN_db (str): The CAN database file. Default is "./data/can_db/motohawk.dbc".
    - --CAN_device (str): The CAN device to write to. Default is "vcan0".
    - --CAN_filename (str): The CAN file to read from. Default is "./data/CANlog.log".

    Example:
    python3 replay.py --enable_serial --serial_filename ./data/examples/example1.json --server_device ./replay/ttyNewServer --client_device ./replay/ttyNewClient --baudrate 115200 --start_time 0 --end_time 10 --enable_gui --enable_test_rate --httpport 8080 --server_ip
    """
    args = argparse.ArgumentParser()
    args.add_argument("--enable_serial", action="store_true", help="Enable serial emulator")
    args.add_argument("--serial_filename", type=str, help="The file to read data from", default="./data/gnss_output/example.json")
    args.add_argument("--server_device", type=str, help="The device to write data to", default="./replay/ttyNewServer")
    args.add_argument("--client_device", type=str, help="The device to read data from", default="./replay/ttyNewClient")
    args.add_argument("--baudrate", type=int, help="The baudrate to write", default=115200)
    args.add_argument("--start_time", type=int, help="The timestamp to start the reading in seconds. If not specified, will read from the beginning of the file.", default=None)
    args.add_argument("--end_time", type=int, help="The time to stop reading in seconds, if not specified, will write until the endo fo the file", default=None)
    args.add_argument("--enable_gui", action="store_true", help="Whether to display the GUI. Default is False", default=False)
    args.add_argument("--enable_test_rate", action="store_true", help="Test rate mode. Instead of showing the trace or reproducing it, it will output the positioning (Lat, Lon) update frequency and save the related data, message by message, on a file named replay_out.csv. Default is False", default=False)
    args.add_argument("--httpport", type=int, help="The port for the HTTP server. Default is 8080", default=8080)
    args.add_argument("--server_ip", type=str, help="The IP address of the server. Default is 127.0.0.1", default="127.0.0.1")
    args.add_argument("--server_port", type=int, help="The port of the server. Default is 48110", default=48110)
    args.add_argument("--enable_CAN", action="store_true", help="Enable CAN emulator", default=False)
    args.add_argument("--CAN_db", type=str, help="The CAN database file", default="./data/can_db/motohawk.dbc")
    args.add_argument("--CAN_device", type=str, help="The CAN device to write to", default="vcan0")
    args.add_argument("--CAN_filename", type=str, help="The CAN file to read from", default="./data/can_output/can_log.json")

    args = args.parse_args()
    serial = args.enable_serial
    serial_filename = args.serial_filename
    server_device = args.server_device
    client_device = args.client_device
    baudrate = args.baudrate
    start_time = args.start_time * 1e6 if args.start_time else None
    end_time = args.end_time * 1e6 if args.end_time else None
    gui = args.enable_gui
    httpport = args.httpport
    server_ip = args.server_ip
    server_port = args.server_port
    test_rate = args.enable_test_rate

    CAN = args.enable_CAN
    CAN = True # For testing purposes
    CAN_db = args.CAN_db
    CAN_db = "./data/can_db/PCAN.dbc" # For testing purposes
    CAN_device = args.CAN_device
    CAN_filename = args.CAN_filename
    CAN_filename = "./data/can_output/CANlog.json" # For testing purposes

    assert serial > 0 or gui > 0 or test_rate > 0 or CAN > 0, "At least one of the serial or GUI or test rate or CAN options must be activated"

    if test_rate > 0 and (serial > 0 or gui > 0):
        "Error: test rate mode can be selected only when both --gui and --serial are set to 0"
        exit(1)

    visualizer = None
    
    if gui or test_rate:
        # Creation of the visualizer object
        visualizer = Visualizer()
        # If GUI modality is activated, start the nodejs server
        fifo_path = "./replay/fifo"
        if not os.path.exists(fifo_path):
            os.mkfifo(fifo_path)
        visualizer.start_nodejs_server(httpport, server_ip, server_port, fifo_path)
    
    if CAN:
        can_thread = threading.Thread(
            target=write_CAN, args=(CAN_device, CAN_filename, CAN_db, start_time, end_time, gui, visualizer, server_ip, server_port)
        )
        can_thread.daemon = True
        can_thread.start()

    if serial or test_rate:
        serial_test_rate(server_device, client_device, baudrate, serial_filename, start_time, end_time, gui, serial, test_rate, server_ip, server_port, fifo_path, visualizer)

    if CAN:
        can_thread.join()

if __name__ == "__main__":
    main()
