# GNSS-Parser-Reproducer

This project is designed to record and replay GNSS serial data, specifically handling NMEA and UBX messages. 

It consists of three main scripts: `record/src/record.py`, `replay/src/replay.py`, and `replay/src/serial_emulator.py`.

## Features

- **serial_emulator.py**: Emulates a serial device using `socat`.
- **record.py**: Reads data from a serial device and saves it to a JSON file.
- **replay.py**: Reads data from a JSON file and writes it to a serial device and, if desired, it displays a GUI to visualize the vehicle.
- **decoded_messages**: Decodes NMEA messages to extract latitude, longitude, and heading of vehicle for the GUI.

## Requirements

- Python 3.x
- pyserial
- socat (for serial emulation)
- nodejs (for the GUI mode)
- can-utils
- cantools

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
6. Install the can-utils packages:
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
- [ ] CAN Database parsing
