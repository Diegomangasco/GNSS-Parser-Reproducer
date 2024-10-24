import serial, time, json, argparse
import signal

global terminatorFlag

def signal_handler(sig, frame):
    """
    Signal handler for the SIGINT signal.
    """
    global terminatorFlag
    print('\nTerminating...');
    terminatorFlag = True

def save_message(messages, res, timestamp, message_type):
    """
    Saves a message to the provided list of messages.
    
    Parameters:
    - messages (list): The list to which the message will be appended.
    - res (str): The message content.
    - timestamp (float): The timestamp of the message.
    - message_type (str): The type of the message (e.g., "NMEA", "UBX").
    """
    # print(message_type)
    # print(res.hex())
    try:
        data = {
            "timestamp": timestamp,
            "type": message_type,
            "data": res.hex() if message_type in ["UBX", "Unknown"] else res.decode()
        }
        messages.append(data)
    except:
        print("Ignored message:",res)
        print("Ignored message type:",message_type)

def write_to_file(f, messages):
    """
    Writes the list of messages to the specified file in JSON format.\n
    
    Parameters:
    - f (file object): The file object to write to.
    - messages (list): The list of messages to write.
    """
    json_object = json.dumps(messages)
    print("Writing to file...")
    f.write(json_object)
    print("Done...")
    f.close()

def setup_file(filename):
    """
    Sets up a file for writing by truncating its content.
    
    Parameters:
    - filename (str): The file to write to.

    Returns:
    - f (file object): The file object to write to.
    """
    f = open(filename, "a")

    # Drop the old content of the file
    f.seek(0)
    f.truncate()
    return f

def close_file(f):
    f.close()

def main():
    global terminatorFlag
    """
    Main function to read data from a serial device and save it to a file.
    
    Command-line Arguments:
    - --device (str): The device to read from. Default is "/dev/ttyACM0".
    - --filename (str): The file to write to. Default is "./data/outlog.json".
    - --baudrate (int): The baudrate to read from. Default is 115200.
    - --end_time (int): The time to stop reading in seconds. If not specified, will read indefinitely.

    Example:
    python record.py --device /dev/ttyACM0 --filename /home/outlog.json --baudrate 115200 --end_time 60
    """
    args = argparse.ArgumentParser()
    args.add_argument("--device", type=str, help="The device to read from", default="/dev/ttyACM0")
    args.add_argument("--filename", type=str, help="The file to write to", default="./data/outlog.json")
    args.add_argument("--baudrate", type=int, help="The baudrate to read from", default=115200)
    args.add_argument("--end_time", type=int, help="The time to stop reading in seconds, if not specified, will read indefinitely", default=None)

    terminatorFlag = False
    signal.signal(signal.SIGINT, signal_handler)

    args = args.parse_args()
    device = args.device
    filename = args.filename
    baudrate = args.baudrate
    end_time = args.end_time

    ser = serial.Serial(
        port=device,
        baudrate=int(baudrate),
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0
    )
    
    f = setup_file(filename)

    messages = list()
    queue = b''
    ubx_flag = False
    ubx_timestamp = None
    nmea_timestamp = None
    unknown_timestamp = None
    previous_data = b''
    flat_time = time.time() * 1e6

    null_cnt = 0

    print('Recording...');
    if end_time is not None:
        end_time = time.time() + end_time
    while True:
        data = ser.read(size=1)
        # print(data)
        if not data:
            null_cnt = null_cnt + 1
        else:
            null_cnt = 0
        if null_cnt > 10000:
            print("Error. Serial stopped sending data...")
            break
        if len(queue) < 1:
            previous_data = data
            queue += data
            continue
        # Read the last two bytes of the queue
        last_two_bytes = previous_data + data
        if last_two_bytes == b'$G':
            # A NMEA message is starting
            if ubx_flag:
                save_message(messages, queue[:-1], ubx_timestamp - flat_time, "UBX")
            elif len(queue) - 1 > 0:
                save_message(messages, queue[:-1], unknown_timestamp - flat_time, "Unknown")
            nmea_timestamp = time.time() * 1e6
            queue = last_two_bytes
            ubx_flag = False
        elif last_two_bytes == b'\r\n':
            # One message is ending
            if not ubx_flag:
                save_message(messages, queue + b'\n', nmea_timestamp - flat_time, "NMEA")
            else:
                save_message(messages, queue[:-1], ubx_timestamp - flat_time, "UBX")
                ubx_flag = False
            queue = b''
            unknown_timestamp = time.time() * 1e6
        elif last_two_bytes == b'\xb5\x62':
            # A UBX message is starting
            if ubx_flag:
                save_message(messages, queue[:-1], ubx_timestamp - flat_time, "UBX")
            elif len(queue) - 1 > 0:
                save_message(messages, queue[:-1], unknown_timestamp - flat_time, "Unknown")
            ubx_flag = True
            ubx_timestamp = time.time() * 1e6
            queue = last_two_bytes
        else:
            queue += data
        t = time.time()
        if (end_time is not None and t > end_time) or terminatorFlag:
            break
        previous_data = data

    # Write the messages to the file
    write_to_file(f, messages)

if __name__ == "__main__":
    main()
