import time, argparse, json, threading, socket, os
from serial_emulator import SerialEmulator
from decoded_messages import DecodedMessage

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
    python replay.py --filename ./data/examples/vehicle_log_brief.json --server_device ./replay/ttyNewServer --client_device ./replay/ttyNewClient --baudrate 115200 --end_time 60
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
    args.add_argument("--httpport", type=int, help="The port for the HTTP server. Default is 8080", default=8080)
    args.add_argument("--server_ip", type=str, help="The IP address of the server. Default is 127.0.0.1", default="127.0.0.1")
    args.add_argument("--server_port", type=int, help="The port of the server. Default is 48110", default=48110)

    args = args.parse_args()
    filename = args.filename
    server_device = args.server_device
    client_device = args.client_device
    baudrate = args.baudrate
    start_time = args.start_time
    end_time = args.end_time
    gui = args.gui
    serial = args.serial

    # gui = 1
    assert serial > 0 or gui > 0, "At least one of the serial or GUI options must be activated"
    
    ser = None
    decoder = None
    if serial:
        ser = SerialEmulator(device_port=server_device, client_port=client_device, baudrate=baudrate)
    if gui:
        decoder = DecodedMessage()

    f = open(filename, "r")
    data = json.load(f)
    f.close()
    
    if start_time:
        start_time_micseconds = start_time * 1e6
        assert start_time_micseconds < data[-1]["timestamp"], "The start time is greater than the last timestamp in the file"
        data = list(filter(lambda x: x["timestamp"] >= start_time_micseconds, data))
    
    previous_time = 0 if not start_time else start_time*1e6
    map_opened = False

    if gui:
        httpport = args.httpport
        server_ip = args.server_ip
        server_port = args.server_port
        fifo_path = "./replay/fifo"
        if not os.path.exists(fifo_path):
            os.mkfifo(fifo_path)
        start_nodejs_server(httpport, server_ip, server_port, fifo_path)

    lat = None
    lon = None    
    heading = None
    before_time = 0
    after_time = 0
    delta_us = 0
    startup_time = time.time()*1e6
    for d in data:
        before_time = time.time() * 1e6
        delta_time = d["timestamp"] - previous_time
        message_type = d["type"]
        content = d["data"]
        if message_type == "UBX":
            content = bytes.fromhex(content)
            if gui:
                tmp_lat, tmp_lon, tmp_heading = decoder.extract_data(content, message_type)
                if tmp_lat:
                    lat = tmp_lat
                if tmp_lon:
                    lon = tmp_lon
                if tmp_heading:
                    heading = tmp_heading
        else:
            content = content.encode()
            if gui:
                tmp_lat, tmp_lon, tmp_heading = decoder.extract_data(content.decode(), message_type)
                if tmp_lat:
                    lat = tmp_lat
                if tmp_lon:
                    lon = tmp_lon
                if tmp_heading:
                    heading = tmp_heading
        before_time = time.time() * 1e6 - before_time
        if delta_time > before_time + after_time + delta_us:
            time.sleep((delta_time - before_time - after_time - delta_us)/1e6)
            # time.sleep(delta_time/1e6)
            # pass
        else:
            time.sleep(delta_time/1e6)
            # pass
        after_time = time.time() * 1e6
        if serial:
            # Calculate a ... -> to be commented
            delta_us=-(d["timestamp"]-(time.time()*1e6-startup_time)-(start_time*1e6 if start_time else 0))
            ser.write(content)
        if gui and lat and lon:
            try:
                if not map_opened:
                    fp = open(fifo_path, 'r')
                    info = fp.read()
                    if "ready" not in info:
                        raise Exception("Error opening map GUI")
                    open_map_gui(lat, lon, server_ip, server_port)
                    map_opened = True
                    print("It is possible to open the map GUI at http://localhost:8080")
            except Exception as e:
                print(f"Error opening map GUI: {e}")
                raise e
            try:
                send_object_udp_message(lat, lon, heading, server_ip, server_port)
            except Exception as e:
                print(f"Error sending UDP message: {e}")
                raise e
        previous_time = d["timestamp"]
        if end_time and time.time() > end_time:
            break
        after_time = time.time() * 1e6 - after_time
    if serial:
        ser.stop()
    if gui:
        stop_server(server_ip, server_port)

if __name__ == "__main__":
    main()
