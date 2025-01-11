# Moria Bringup Package
The `moria_bringup` package is a ROS 2 package designed to provide launch scripts for initializing and running the Moria robot simulation and bringup processes. This package is tailored for the Moria two-wheel robot and supports components such as RViz2, robot state publishing, and URDF description handling.

## Features
- **Launch Configurations**:
  - `rviz2.launch.py`: Launches RViz2 with the robot model, joint state publisher GUI, and robot state publisher.
  - `state_publisher.py`: Launches the robot state publisher for other simulation purposes. Added in case of different uses other than using without Rviz2.
- **URDF Integration**: Dynamically loads the robot description (URDF) using xacro, with parameters for ROS 2 control and simulation mode.
- **Simulation Ready**: Supports simulation time for use with Gazebo or similar environments.

## Package Contents
### Launch Files
`rviz2.launch.py`
This launch file:
- Loads the robot description from a xacro file.
- Launches the following nodes:
  - `robot_state_publisher` for publishing the robot's URDF to the `/robot_description` topic.
  - `joint_state_publisher_gui` for manually adjusting joint states in simulation. 
  - `rviz2` for visualizing the robot model in RViz.

`state_publisher.launch.py`
This launch file:
- Loads the robot description from a xacro file.
- Launches the 'robot_state_publisher` node to publish the robot's state.

## Package Dependencies
- ament_cmake
- robot_state_publisher
- rviz2
- moria_description
- moria_node
- hls_lfcd_lds_driver

## Installation
1. Clone the repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/kemaltml/moria_ws
```

2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
```

3. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage
### Launch Rviz2 Visualization 
To launch Rviz2 with the robot model:
```bash
ros2 launch moria_bringup rviz2.launch.py
```

# TR 
# Moria Bringup Paketi
`moria_bringup` paketi, Moria robot simülasyonu ve başlatma süreçlerini başlatmak için tasarlanmış bir ROS 2 paketidir. Bu paket, Moria iki tekerlekli robot için özelleştirilmiştir ve RViz2, robot durumu yayınlama ve URDF tanımı gibi bileşenleri destekler.

## Özellikler
- **Başlatma Yapılandırmaları**:
  - `rviz2.launch.py`: Robot modeli, eklem durumu yayıncı GUI'si ve robot durumu yayıncısı ile RViz2'yi başlatır.
  - `state_publisher.py`: Farklı simülasyon ihtiyaçları için robot durumu yayıncısını başlatır. RViz2 olmadan kullanılmak üzere eklenmiştir.
- **URDF Entegrasyonu**: ROS 2 kontrol ve simülasyon modu parametreleri ile xacro kullanarak robot tanımını dinamik olarak yükler.
- **Simülasyon Desteği**: Gazebo veya benzeri ortamlarda kullanılmak üzere simülasyon zamanını destekler.

## Paket İçeriği
### Başlatma Dosyaları
`rviz2.launch.py`
Bu başlatma dosyası:
- Xacro dosyasından robot tanımını yükler.
- Şu düğümleri başlatır:
  - `/robot_description` konusuna robotun URDF'sini yayınlamak için `robot_state_publisher`.
  - Simülasyonda eklem durumlarını manuel olarak ayarlamak için `joint_state_publisher_gui`.
  - RViz'de robot modelini görselleştirmek için `rviz2`.

`state_publisher.launch.py`
Bu başlatma dosyası:
- Xacro dosyasından robot tanımını yükler.
- Robotun durumunu yayınlamak için `robot_state_publisher` düğümünü başlatır.

## Paket Bağımlılıkları
- ament_cmake
- robot_state_publisher
- rviz2
- moria_description
- moria_node
- hls_lfcd_lds_driver

## Kurulum
1. Depoyu ROS2 çalışma alanınıza klonlayın:
```bash
cd ~/ros2_ws/src
git clone https://github.com/kemaltml/moria_ws
```
2. Çalışma Alanını Derleyin:
```bash
cd ~/ros2_ws
colcon build
```
3. Çalışma Alanını Kaynaklayın:
```bash
source ~/ros2_ws/install/setup.bash
```

## Kullanım
RViz2'yi Başlatma
Robot modelini Rviz2 ile başlatmak için:
```bash
ros2 launch moria_birngup rviz2.launch.py
```
