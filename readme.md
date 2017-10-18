# Python nrf52 OTA BLE

This is a python program to perform a Device Firmware Update over BLE on nRF52832 devices.
It has been developed on Linux 14.04.
The target device should have SDK version 11 and must have a working DFU service.

## Prerequisite

This project depends by several modules:

- bluepy
	sudo apt-get install python-pip libglib2.0-dev
	sudo pip install bluepy

- nrfutil (version 0.5.2)
	sudo pip install --pre nrfutil==0.5.2

- pyqt4
	sudo apt-get install python-qt4

## Usage

Launch the program with the following command:

	sudo python gui.py

"sudo" is needed to scan BLE devices.

- Select a file to be uploaded by pressing the "Select File" button. The file can be a .bin (or .hex) together with .dat, a .zip (containing the bin and dat files), just an .hex file or just a .bin file (nrfutil will be used to generate the .dat file in the latest cases).
- Scan the BLE devices (the blank field below the scan button will be filled with the found devices).
- Select a device from the list: ota programming will start immediately.
