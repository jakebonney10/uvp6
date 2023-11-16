# UVP6 ROS Package

## Description

The `uvp6` ROS package provides an interface for the Underwater Vision Profiler 6 (UVP6), enabling control and data acquisition via RS232 communication. This package includes a Python driver for UVP6, custom ROS messages, and a driver node for real-time data handling and publishing.

## Features

- Control and data acquisition from UVP6 via RS232.
- ROS messages for HWconf, ACQconf, LPM_DATA, and BLACK_DATA frames.
- ROS node for data acquisition and publishing.
- Handles start and stop of data acquisition.
- Custom Python driver for UVP6.

## Installation

Please follow the standard ROS package installation procedures to install `uvp6`.

## Usage

Run the UVP6 driver node using:

```bash
rosrun uvp6 uvp6_driver.py
```

Or alternatively, run the driver using the launch file, specifying the serial port and baudrate within the launch file.

```bash
roslaunch uvp6 upv6.launch
```

## Custom ROS Messages
This package includes custom ROS messages for different data frames from the UVP6:

- `HWconfMsg`
- `ACQconfMsg`
- `LPMDataMsg`
- `BlackDataMsg`

## Setting Parameters via Command Line

Before running the UVP6 driver node, you can set the serial port and baud rate parameters using the ROS command line interface. This is useful for configuring the connection to the UVP6 instrument according to your setup.

Use the following `rosparam` commands to set these parameters:

```bash
rosparam set /uvp6_driver/port YOUR_SERIAL_PORT
rosparam set /uvp6_driver/baudrate YOUR_BAUD_RATE
```

Replace YOUR_SERIAL_PORT with the actual serial port your UVP6 instrument is connected to (e.g., /dev/ttyUSB0 for Linux or COM3 for Windows) and YOUR_BAUD_RATE with the desired baud rate (e.g., 9600).

After setting these parameters, you can run the UVP6 driver node as described in the Usage section.

## UVP6 Instrument Details

### Sensor Description
UVP6 comes in two models: UVP6HF and UVP6LP, with different power consumption profiles. UVP6HF consumes 6W during acquisition and 0.02W in standby, whereas UVP6LP consumes 0.8W during acquisition and 0.02W in standby.

### Operational Modes
UVP6 supports different operation modes suitable for various deployments.

### Storage
Equipped with a minimum of 400GB of mass storage.

## Recommended Operational Procedures

### Initial Setup
- Mount the instrument on its frame or hosting platform.
- Connect to power (AC or battery) and configure using UVPapp.
- Perform time synchronization and data deletion if necessary.

### Deployment
- Charge the battery if applicable.
- Clean the porthole and light before deployment.

### Daily Operations
- Regular data download, sequence merging, and sample creation.
- Conduct data quality control (QC) and backups.

### End of Cruise
- Remove the instrument from its host/frame without disassembling the camera from the arm.
- Clean and pack the instrument.

### Data Processing
- Process data and images, export to ODV if needed, and upload to EcoPart and EcoTaxa for classification.

## Safety and Maintenance
- **Optical Alignment**: Do not disconnect the arm connecting the camera to the light unit.
- **Sunlight Protection**: Avoid exposing the instrument to direct sunlight.
- **Connector Care**: Prevent corrosion and follow maintenance recommendations.

## License
This code was written under the MIT license.

