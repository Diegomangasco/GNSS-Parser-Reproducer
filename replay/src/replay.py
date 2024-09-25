import time, argparse, json, threading, socket, os
from serial_emulator import SerialEmulator
from decoded_messages import DecodedMessage


def start_nodejs_server(httpport, ip, port):
    """
    Starts the nodejs server for the vehicle visualizer.
    """
    try:
        os.system(f"node ./replay/vehicle_visualizer/server.js {httpport} {ip} {port} &")
    except Exception as e:
        print(f"Error starting nodejs server: {e}")

def send_udp_message(lat, lon, heading, server_ip, server_port):
    """
    Sends a UDP message with the latitude, longitude, and heading to the specified server.
    """
    message = json.dumps({"lat": lat, "lon": lon, "heading": heading})
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(message.encode(), (server_ip, server_port))
    except Exception as e:
        print(f"Error sending UDP message: {e}")

def udp_thread(content, server_ip, server_port, lat, lon, heading):
    """
    Thread function to send a UDP message with the latitude, longitude, and heading to the specified server.
    """
    send_udp_message(lat, lon, heading, server_ip, server_port)


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
    - --server_device (str): The device to write data to. Default is "./replay/serial_devices/ttyNewServer".
    - --client_device (str): The device to read data from. Default is "./replay/serial_devices/ttyNewClient".
    - --baudrate (int): The baudrate to write. Default is 115200.
    - --start_time (int): The timestamp to start the reading in seconds. If not specified, will read from the beginning of the file.
    - --end_time (int): The time to stop reading in seconds. If not specified, will write until the end of the file.

    Example:
    python replay.py --filename ./data/outlog.json --server_device ./replay/ttyNewServer --client_device ./replay/ttyNewClient --baudrate 115200 --end_time 60
    """
    args = argparse.ArgumentParser()
    args.add_argument("--filename", type=str, help="The file to read data from", default="./data/examples/example1.json")
    args.add_argument("--server_device", type=str, help="The device to write data to", default="./replay/serial_devices/ttyNewServer")
    args.add_argument("--client_device", type=str, help="The device to read data from", default="./replay/serial_devices/ttyNewClient")
    args.add_argument("--baudrate", type=int, help="The baudrate to write", default=115200)
    args.add_argument("--start_time", type=int, help="The timestamp to start the reading in seconds. If not specified, will read from the beginning of the file.", default=None)
    args.add_argument("--end_time", type=int, help="The time to stop reading in seconds, if not specified, will write until the endo fo the file", default=None)
    args.add_argument("--gui", type=int, help="Whether to display the GUI. Default is False (0), can be activated with any other positive value", default=0)
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

    # TODO To cancel
    gui = True
    
    ser = SerialEmulator(device_port=server_device, client_port=client_device, baudrate=baudrate)
    decoder = DecodedMessages()

    f = open(filename, "r")
    data = json.load(f)
    f.close()

    if start_time:
        start_time_micseconds = start_time * 1e6
        data = list(filter(lambda x: x["timestamp"] >= start_time_micseconds, data))

    previous_time = 0

    if gui:
        httpport = args.httpport
        server_ip = args.server_ip
        server_port = args.server_port
        start_nodejs_server(httpport, server_ip, server_port)

    for d in data:
        delta_time = d["timestamp"] - previous_time
        message_type = d["type"]
        content = d["data"]
        if message_type == "UBX":
            content = bytes.fromhex(content)
        else:
            content = content.encode()
        lat, lon, heading = decoder.extract_data(content, message_type)
        time.sleep(delta_time/1e6)
        ser.write(content)
        if gui:
            try:
                udp_sender = threading.Thread(target=udp_thread, args=(content, server_ip, server_port, lat, lon, heading))
                udp_sender.start()
                udp_sender.join()
            except Exception as e:
                print(f"Error starting thread: {e}")
        previous_time = d["timestamp"]
        if end_time and time.time() > end_time:
            break
    ser.stop()

if __name__ == "__main__":
    main()
