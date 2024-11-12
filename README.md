# TRACE-X: Telemetry Replay and Analysis of CAN bus and EXternal navigation data

This project is designed to record and replay GNSS serial data, specifically handling NMEA and UBX messages, and CAN Bus data. 

It consists of two main scripts: `record/src/record.py` and `replay/src/replay.py`.

## Features

- **record.py**: Reads data from a Serial Device and/or a CAN Bus and saves it to a JSON file.
- **replay.py**: Reads data from JSON files and emulates in real-time a Serial Device and/or a CAN Bus. If desired, it displays a GUI to visualize the vehicle and the objects perceived.
- **serial_emulator.py**: Utility class to emulate a serial device using `socat`.
- **decoded_messages**: Utility class to decode NMEA messages to extract latitude, longitude, and heading of vehicle.

## Requirements

- Python 3.x
- pyserial
- nodejs (for the GUI mode)
- socat (for serial emulation)
- cantools (for CAN Bus emulation)
- [Optional] can-utils

## Installation

1. Clone the repository:
    ```sh
        git clone https://github.com/Diegomangasco/GNSS-Parser-Reproducer.git
        cd GNSS-Parser-Reproducer
    ```

2. Install the required Python packages:
    ```sh
        pip install pyserial
        pip install cantools
        pip install pyproj
    ```

3. Ensure `socat` is installed on your system:
    ```sh
        sudo apt-get install socat
    ```

4. Ensure `nodejs` is installed on your system:
    ```sh
        sudo apt install nodejs
    ```

5. Install the npm packages:
    ```sh
        cd replay/vehicle_visualizer
        npm install
    ```

6. Prepare the virtual CAN Bus for the emulation:
    ```sh
        sudo ip link add dev vcan0 type vcan
        sudo ip link set up vcan0       
    ```

7. [Optional] Install the can-utils packages (just if you want to test with "canplayer" and "candump"):
    ```sh
        sudo apt install can-utils
    ```

## Authors
- **Diego Gasco** - Politecnico di Torino - diego.gasco@polito.it
- **Carlos Mateo Risma Carletti** - Politecnico di Torino - carlos.rismacarletti@polito.it
- **Francesco Raviglione** - Politecnico di Torino - francesco.raviglione@polito.it
- **Marco Rapelli** - Politecnico di Torino - marco.rapelli@polito.it
- **Claudio Casetti** - Politecnico di Torino - claudio.casetti@polito.it

## Work-in-progress
- [ ] Enable the reliable usage of baud rates higher than 115200
- [X] Make the record script more robust to issues that may stop the recording of the trace, making it save anyway what has been captured until that moment
- [X] CAN Database parsing
- [ ] GUI for CAN objects
