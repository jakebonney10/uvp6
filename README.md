# UVP6 ROS Package

## Description

The UVP6 ROS Driver is a package designed for integration and operation of the Underwater Vision Profiler 6 (UVP6) with autonomous surface or underwater vehicles (ASVs or UUVs) in supervised mode. This package enables real-time data acquisition, processing, and control of the UVP6, supporting various research and monitoring activities in marine environments.
## Features

- Control and data acquisition from UVP6 via RS232 in SUPERVISED mode.
- ROS messages for HWconf, ACQconf, LPM_DATA, and BLACK_DATA frames.
- ROS node for data acquisition and publishing.
- Handles start and stop of data acquisition.
- Custom Python library for UVP6 communication.

## Installation

Please follow the standard ROS package installation procedures to install `uvp6`. Clone the repository into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/your-username/uvp6-ros-driver.git
```

Navigate back to your catkin workspace and compile:
```bash
cd ~/catkin_ws
catkin_make
```

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

- `HWconf`: Hardware configuration data.
- `ACQconf`: Acquisition configuration data.
- `TAXOconf`: Taxonomy classifier configuration.
- `LPMData`: Large particle measurement data.
- `BlackData`: Data from black frames.
- `TaxoData`: Taxonomy data for large particle identifiers.
- `ObjectData`: Used in TaxoData to record varying number of object data.

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
UVP6 supports different operation modes suitable for various deployments, this driver is meant to control the device in SUPERVISED mode.

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
- DO NOT disconnect the blue cable from the camera housing as it will require reprogramming. 
- Clean and pack the instrument.

### Data Processing
- Process data and images, export to ODV if needed, and upload to EcoPart and EcoTaxa for classification.

## Safety and Maintenance
- **Optical Alignment**: Do not disconnect the arm connecting the camera to the light unit.
- **Sunlight Protection**: Avoid exposing the instrument to direct sunlight.
- **Connector Care**: Prevent corrosion and follow maintenance recommendations.

## Documentation

- [UVP6 Publication](docs/Limnology-OceanMethods-2021-Picheral-The_Underwater_Vision_Profiler6_an_imaging_sensor_of_particle_size_spectra.pdf): Literature on the UVP6 device.
- [UVP6 User Guide](docs/uvp6_user_guide_20220309.pdf): Detailed instructions and specifications for the UVP6 instrument.
- [UVP6 Supervised RT Data Specifications](docs/UVP6_supervised_RTdata_specifications_20231123.pdf): Specifications for real-time data handling in supervised mode.

## Contributing
Contributions to the UVP6 ROS Driver are welcome. Please follow standard ROS development practices for contributions. You can fork and create pull requests as I will be regularly maintaining this package through 2024.

## License
This code was written under the MIT license.

