# Orientation Converter - ROS1  

## Overview  
The **Orientation Converter** is a ROS node that corrects the orientation of a robot by compensating for the Earth's magnetic declination using the World Magnetic Model (WMM). It takes GPS and IMU quaternion data as input and publishes a corrected quaternion with true north alignment.  

## Key Features  
- Uses **GeographicLib** to calculate real-time magnetic declination  
- Applies **Quaternion transformations** to correct yaw values  
- Subscribes to **GPS and IMU quaternion data**  
- Publishes **corrected orientation** with declination adjustment  
- Utilizes **ROS threading and smart pointers** for efficient processing  

---

## Dependencies & Installation  

### Dependencies  
Ensure you have the following installed:  

- **ROS1 Noetic**
- **GeographicLib**  
- **robot_localization**  
- **tf2_geometry_msgs**  

### Installation  
```bash
# Install GeographicLib datasets (WMM)
sudo apt install geographiclib-tools
wget -O /usr/share/GeographicLib/magnetic.wmm2020 https://geographiclib.sourceforge.io/magnetic.html

# Clone & Build
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/Nil69420/orientation_converter.git
cd ~/ros_ws
catkin build
source devel/setup.bash
```

---

## Parameters, Topics, and Services  

### **Subscribed Topics**  
| Topic Name             | Message Type                     | Description |
|------------------------|--------------------------------|-------------|
| `/gps/filtered`        | `sensor_msgs/NavSatFix`       | GPS data (latitude, longitude) |
| `/filter/quaternion`   | `geometry_msgs/QuaternionStamped` | IMU Quaternion |

### **Published Topics**  
| Topic Name                | Message Type                     | Description |
|---------------------------|--------------------------------|-------------|
| `/corrected_orientation`  | `geometry_msgs/QuaternionStamped` | Corrected quaternion with declination adjustment |

### **Parameters**  
| Parameter Name     | Type   | Description |
|--------------------|--------|-------------|
| `magnetic_model`  | `string` | Path to the WMM data file |

---

## Algorithm  

1. **GPS Processing**  
   - Receives GPS coordinates (latitude & longitude) from `/gps/filtered`  
   - Stores the latest valid GPS fix  

2. **IMU Quaternion Handling**  
   - Receives IMU orientation data from `/filter/quaternion`  
   - Converts quaternion to **roll, pitch, yaw**  

3. **Magnetic Declination Correction**  
   - Uses **GeographicLib::MagneticModel** to compute declination at the GPS location  
   - Adjusts yaw by adding computed declination  

4. **Quaternion Transformation & Publishing**  
   - Converts corrected yaw back to quaternion  
   - Publishes the corrected orientation to `/corrected_orientation`  

---

## Usage  
```bash

roslaunch magnetic_declination_converter magnetic_correction.launch

rostopic pub /gps/filtered sensor_msgs/NavSatFix '{latitude: 37.7749, longitude: -122.4194, altitude: 10.0}'

```
