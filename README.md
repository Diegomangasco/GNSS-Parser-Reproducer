# GNSS-Parser-Reproducer

This project is designed to record and replay GNSS serial data, specifically handling NMEA and UBX messages. 
It consists of three main scripts: `record/record.py`, `replay/replay.py`, and `replay/serial_emulator.py`.

## Features

- **serial_emulator.py**: Emulates a serial device using `socat`.
- **record.py**: Reads data from a serial device and saves it to a JSON file.
- **replay.py**: Reads data from a JSON file and writes it to a serial device.

## Requirements

- Python 3.x
- pyserial
- socat (for serial emulation)

## Installation

1. Clone the repository:
    ```sh
    git clone https://github.com/Diegomangasco/GNSS-Parser-Reproducer.git
    cd GNSS-Parser-Reproducer
    ```

2. Install the required Python packages:
    ```sh
    pip install pyserial
    ```

3. Ensure `socat` is installed on your system:
    ```sh
    sudo apt-get install socat
    ```
