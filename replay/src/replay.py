import time, argparse, json, threading, socket, os
import numpy as np
import math
from serial_emulator import SerialEmulator
from decoded_messages import DecodedMessage

CLUSTER_TSHOLD_MS = 20 # In [ms]

def nmea_to_degrees(lat_or_lon, value, direction):
    # Split degrees and minutes
    if lat_or_lon == "lon":  # the longitude format is DDDMM.MMMM
        degrees = int(value[:3])
        minutes = float(value[3:])
    elif lat_or_lon == "lat":  # the latitude format DDMM.MMMM
        degrees = int(value[:2])
        minutes = float(value[2:])
    else:
        print("Critical error: called nmea_to_degrees() with an unrecognized first argument. The first argument must be either 'lat' or 'lon'.")
        exit(1)
    
    # Convert to decimal degrees
    decimal_degrees = degrees + (minutes / 60)
    
    # Fix the sign depending on the direction -> South and West mean negative degrees
    if direction in ['S', 'W']:
        decimal_degrees = -decimal_degrees
    
    return decimal_degrees

def compare_floats(a, b):
    return math.isclose(a, b, rel_tol=1e-8)

def open_map_gui(lat, lon, server_ip, server_port):
    """
    Opens the map GUI.
    """
    message = f"map,{lat},{lon}"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(message.encode(), (server_ip, server_port))
    except Exception as e:
        print(f"Error sending UDP message: {e}")
        raise e

def start_nodejs_server(httpport, ip, port, fifo_path):
    """
    Starts the nodejs server for the vehicle visualizer.
    """
    try:
        os.system(f"node ./replay/vehicle_visualizer/server.js {httpport} {ip} {port} {fifo_path} &")
    except Exception as e:
        print(f"Error starting nodejs server: {e}")
        raise e

def send_object_udp_message(lat, lon, heading, server_ip, server_port):
    """
    Sends a UDP message with the latitude, longitude, and heading to the specified server.
    """
    id = 1
    station_type = 5
    if not heading:
        heading = 361
    message = f"object,{id},{lat},{lon},{station_type},{heading}"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(message.encode(), (server_ip, server_port))
    except Exception as e:
        print(f"Error sending UDP message: {e}")
        raise e

def stop_server(server_ip, server_port):
    """
    Stops the nodejs server for the vehicle visualizer.
    """
    message = f"terminate"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(message.encode(), (server_ip, server_port))
    except Exception as e:
        print(f"Error stopping nodejs server: {e}")
        raise e

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
    - --filename (str): The file to read data from. Default is "./data/examples/example1.json".
    - --server_device (str): The device to write data to. Default is "./replay/ttyNewServer".
    - --client_device (str): The device to read data from. Default is "./replay/ttyNewClient".
    - --baudrate (int): The baudrate to write. Default is 115200.
    - --start_time (int): The timestamp to start the reading in seconds. If not specified, will read from the beginning of the file.
    - --end_time (int): The time to stop reading in seconds. If not specified, will write until the end of the file.

    Example:
    python replay/src/replay.py --filename ./data/examples/vehicle_log_brief.json --server_device ./replay/ttyNewServer --client_device ./replay/ttyNewClient --baudrate 115200 --end_time 60 --gui 1 --serial 1
    """
    args = argparse.ArgumentParser()
    args.add_argument("--filename", type=str, help="The file to read data from", default="./data/examples/vehicle_log_brief.json")
    args.add_argument("--server_device", type=str, help="The device to write data to", default="./replay/ttyNewServer")
    args.add_argument("--client_device", type=str, help="The device to read data from", default="./replay/ttyNewClient")
    args.add_argument("--baudrate", type=int, help="The baudrate to write", default=115200)
    args.add_argument("--start_time", type=int, help="The timestamp to start the reading in seconds. If not specified, will read from the beginning of the file.", default=None)
    args.add_argument("--end_time", type=int, help="The time to stop reading in seconds, if not specified, will write until the endo fo the file", default=None)
    args.add_argument("--gui", type=int, help="Whether to display the GUI. Default is False (0), can be activated with any other positive value", default=0)
    args.add_argument("--serial", type=int, help="Whether to use the serial emulator. Default is False (0), can be activated with any other positive value", default=0)
    args.add_argument("--test_rate", type=int, help="Test rate mode. Instead of showing the trace or reproducing it, it will output the positioning (Lat, Lon) update frequency and save the related data, message by message, on a file named replay_out.csv. Default is False (0), can be activated with any other positive value", default=0)
    args.add_argument("--httpport", type=int, help="The port for the HTTP server. Default is 8080", default=8080)
    args.add_argument("--server_ip", type=str, help="The IP address of the server. Default is 127.0.0.1", default="127.0.0.1")
    args.add_argument("--server_port", type=int, help="The port of the server. Default is 48110", default=48110)

    args = args.parse_args()
    filename = args.filename
    server_device = args.server_device
    client_device = args.client_device
    baudrate = args.baudrate
    start_time = args.start_time
    end_time = args.end_time * 1e6 if args.end_time else None
    gui = args.gui
    serial = args.serial
    test_rate = args.test_rate

    assert serial > 0 or gui > 0 or test_rate > 0, "At least one of the serial or GUI or test rate options must be activated"

    if test_rate > 0 and (serial > 0 or gui > 0):
        "Error: test rate mode can be selected only when both --gui and --serial are set to 0"
        exit(1)
    
    ser = None
    decoder = None
    if serial:
        # Creation of the serial emulator
        ser = SerialEmulator(device_port=server_device, client_port=client_device, baudrate=baudrate)
    if gui or test_rate:
        # Creation of the decoded message object
        decoder = DecodedMessage()

    f = open(filename, "r")
    data = json.load(f)
    f.close()
    
    if start_time:
        # Filter the data to start from the specified time
        start_time_micseconds = start_time * 1e6
        assert start_time_micseconds < data[-1]["timestamp"], "The start time is greater than the last timestamp in the file"
        data = list(filter(lambda x: x["timestamp"] >= start_time_micseconds, data))
    
    previous_time = 0 if not start_time else start_time*1e6
    map_opened = False

    if gui:
        # If GUI modality is activated, start the nodejs server
        httpport = args.httpport
        server_ip = args.server_ip
        server_port = args.server_port
        fifo_path = "./replay/fifo"
        if not os.path.exists(fifo_path):
            os.mkfifo(fifo_path)
        start_nodejs_server(httpport, server_ip, server_port, fifo_path)

    latitude = None
    longitude = None    
    heading = None
    before_time = 0
    after_time = 0
    variable_delta_us_factor = 0
    startup_time = time.time() * 1e6
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

    for d in data:
        before_time = time.time() * 1e6
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
                if(len(content)>=4):
                    if(content[2]==0x01 and content[3]==0x05):
                        ubx_nav_pvt_present=True
                    elif(content[2]==0x01 and content[3]==0x07):
                        ubx_nav_att_present=True
                    elif(content[2]==0x10 and content[3]==0x15):
                        ubx_esf_ins_present=True
                    elif(content[2]==0x10 and content[3]==0x03):
                        ubx_esf_raw_present=True

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
                print("Time since last update (UBX):",delta_pos_time/1e3,"Time:",d["timestamp"]/1e3)
            else:
                print("Time since last update (NMEA):",delta_pos_time/1e3,"Time:",d["timestamp"]/1e3)
            
            print("Latitude [deg]:", test_rate_lat, "Longitude [deg]:", test_rate_lon)

            previous_pos_time = d["timestamp"]

            cnt_update_time = cnt_update_time + 1
            average_update_time = average_update_time + (delta_pos_time/1e3-average_update_time) / cnt_update_time
            
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
            # before_time represents the time passed from the beginning of the for loop to the beginning of the serial write
            before_time = time.time() * 1e6 - before_time
            if delta_time > before_time + after_time + variable_delta_us_factor:
                # The delta time is diminished by three time factors
                try:
                    time.sleep((delta_time - before_time - after_time - variable_delta_us_factor) / 1e6)
                except:
                    print("Trying to sleep for a negative time, thus not sleeping: ",(delta_time - before_time - after_time - variable_delta_us_factor) / 1e3)
            else:
                factors = [before_time, after_time, variable_delta_us_factor]
                factors.sort()
                if delta_time > factors[0] + factors[1]:
                    # The delta time is diminished by two time factors
                    try:
                        time.sleep((delta_time - factors[0] - factors[1]) / 1e6)
                    except:
                        print("Trying to sleep for a negative time, thus not sleeping: ",(delta_time - factors[0] - factors[1]) / 1e3)
                elif delta_time > factors[0]:
                    # The delta time is diminished by one time factor
                    try:
                        time.sleep((delta_time - factors[0]) / 1e6)
                    except:
                        print("Trying to sleep for a negative time, thus not sleeping: ",(delta_time - factors[0]) / 1e3)
                else:
                    # The delta time is not diminished by any time factor
                    try:
                        time.sleep(delta_time / 1e6)
                    except:
                        print("Trying to sleep for a negative time, thus not sleeping: ",delta_time / 1e3)
                    # print(delta_time / 1e3)
            if serial:
                ser.write(content)
            # after_time represents the time passed from the end of the serial write to the end of the for loop
            after_time = time.time() * 1e6

            # Calculate a variable delta time factor to adjust the time of the serial write to be as close as possible to a real time simulation
            # delta_time_us represents the real time in microseconds from the beginning of the simulation to the current time
            delta_time_us_real = time.time() * 1e6 - startup_time
            # start_time_us represents the time in microseconds from the beginning of the messages simulation to the start time selected by the user
            start_time_us = start_time * 1e6 if start_time else 0
            # delta_time_us_simulation represents the time in microseconds from the beginning of the messages simulation time to the current message time
            delta_time_us_simulation = d["timestamp"] - start_time_us
            # We want that the time of the serial write is as close as possible to the real time simulation
            # variable_delta_us_factor represents the difference between the simulation time and the real time
            # It should be as close as possible to 0 and it is used to adjust the waiting time for the serial write
            variable_delta_us_factor = abs(delta_time_us_simulation - delta_time_us_real)

        if gui and latitude and longitude:
            try:
                if not map_opened:
                    # Open the map GUI after the nodejs server is ready
                    fp = open(fifo_path, 'r')
                    info = fp.read()
                    if "ready" not in info:
                        raise Exception("Error opening map GUI")
                    open_map_gui(latitude, longitude, server_ip, server_port)
                    map_opened = True
                    print("It is possible to open the map GUI at http://localhost:8080")
            except Exception as e:
                print(f"Error opening map GUI: {e}")
                raise e
            try:
                # Send the new object position to the server that will update the map GUI
                send_object_udp_message(latitude, longitude, heading, server_ip, server_port)
            except Exception as e:
                print(f"Error sending UDP message: {e}")
                raise e
        previous_time = d["timestamp"]
        if end_time and time.time() - startup_time > end_time:
            break
        after_time = time.time() * 1e6 - after_time


    if serial:
        ser.stop()
    if gui:
        stop_server(server_ip, server_port)
    if test_rate:
        print("Average update rate:", 1e3/average_update_time, "Hz")
        print("Average update periodicity:", average_update_time, "ms")

        print("Average update rate (filtered):", 1e3/average_update_time_filtered, "Hz")
        print("Average update periodicity (filtered):", average_update_time_filtered, "ms")

        print("UBX messages statistics:")
        print("UBX-NAV-PVT present:",ubx_nav_pvt_present)
        print("UBX-NAV-ATT present:",ubx_nav_att_present)
        print("UBX-ESF-INS present:",ubx_esf_ins_present)
        print("UBX-ESF-RAW present:",ubx_esf_raw_present)

        np.savetxt('replay_out.csv', [p for p in zip(update_timestamps, update_msg_type, update_peridocities, update_rates, update_msg_clustered, update_msg_lat, update_msg_lon, update_msg_same_position)], delimiter=',', fmt='%s')

if __name__ == "__main__":
    main()
