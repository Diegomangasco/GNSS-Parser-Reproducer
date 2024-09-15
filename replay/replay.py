import time, serial, argparse, json
from serial_emulator import SerialEmulator

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
    - --filename (str): The file to read data from. Default is "./data/outlog.json".
    - --server_device (str): The device to write data to. Default is "./replay/ttyNewServer".
    - --client_device (str): The device to read data from. Default is "./replay/ttyNewClient".
    - --baudrate (int): The baudrate to write. Default is 115200.
    - --end_time (int): The time to stop reading in seconds. If not specified, will write until the end of the file.

    Example:
    python replay.py --filename ./data/outlog.json --server_device ./replay/ttyNewServer --client_device ./replay/ttyNewClient --baudrate 115200 --end_time 60
    """
    args = argparse.ArgumentParser()
    args.add_argument("--filename", type=str, help="The file to read data from", default="./data/outlog.json")
    args.add_argument("--server_device", type=str, help="The device to write data to", default="./replay/ttyNewServer")
    args.add_argument("--client_device", type=str, help="The device to read data from", default="./replay/ttyNewClient")
    args.add_argument("--baudrate", type=int, help="The baudrate to write", default=115200)
    args.add_argument("--end_time", type=int, help="The time to stop reading in seconds, if not specified, will write until the endo fo the file", default=None)

    args = args.parse_args()
    filename = args.filename
    server_device = args.server_device
    client_device = args.client_device
    baudrate = args.baudrate
    end_time = args.end_time
    
    ser = SerialEmulator(device_port=server_device, client_port=client_device, baudrate=baudrate)

    f = open(filename, "r")
    data = json.load(f)
    f.close()

    for d in data:
        delta_time = d["timestamp"]
        message_type = d["type"]
        content = d["data"]
        if message_type == "UBX":
            content = bytes.fromhex(content)
        else:
            content = content.encode()
        time.sleep(delta_time/1e6)
        ser.write(content)
        if end_time and time.time() > end_time:
            break
    ser.stop()

if __name__ == "__main__":
    main()
