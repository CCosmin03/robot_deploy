# Setup Jetson — clone, build, rulare pipeline (ROS Melodic / Ubuntu 18.04)

## 0. Autentificare GitHub prin SSH

GitHub nu mai acceptă parola contului pentru operații git — foloseștechei SSH.

```bash
ssh-keygen -t ed25519 -C "jetson-robot"
cat ~/.ssh/id_ed25519.pub
```

Copiază output-ul de mai sus și adaugă-l pe GitHub la
[github.com/settings/keys](https://github.com/settings/keys) → **New SSH key**.

Testează conexiunea:

```bash
ssh -T git@github.com
```

Ar trebui să vezi: `Hi CCosmin03! You've successfully authenticated...`

## 1. Instalare ROS Melodic (dacă lipsește)

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-melodic-desktop
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Clonare repo

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:CCosmin03/robot_deploy.git
```

## 3. Pregătire workspace catkin

Doar folderele din `ros_ws/` sunt pachete ROS — le legăm în `src`:

```bash
cd ~/catkin_ws/src
ln -s robot_deploy/ros_ws/robot robot
ln -s robot_deploy/ros_ws/robot_bringup robot_bringup
ln -s robot_deploy/ros_ws/robot_description robot_description
ln -s robot_deploy/ros_ws/robot_msgs robot_msgs
ln -s robot_deploy/ros_ws/robot_navigation robot_navigation
ln -s robot_deploy/ros_ws/robot_slam robot_slam
ln -s robot_deploy/ros_ws/robot_teleop robot_teleop
```

## 4. Instalare dependențe de sistem

```bash
sudo apt update
sudo apt install -y \
  ros-melodic-rosserial ros-melodic-rosserial-python \
  ros-melodic-rplidar-ros \
  ros-melodic-gmapping ros-melodic-hector-mapping ros-melodic-hector-slam \
  ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-dwa-local-planner \
  ros-melodic-joint-state-publisher ros-melodic-robot-state-publisher \
  ros-melodic-tf ros-melodic-tf2-ros \
  python-pip
```

> ⚠️ **ARM (Jetson):** `slam_karto` și `cartographer_ros` nu au întotdeauna binare `arm64` pentru Melodic în apt. Dacă folosești `robot_karto.launch` sau `robot_cartographer*.launch`, s-ar putea să trebuiască compilate din surse în workspace (clonate în `~/catkin_ws/src`, construite o dată cu restul de `catkin_make`). Dacă folosești doar `gmapping`/`hector`, pachetele de mai sus sunt suficiente.

Rezolvă orice dependență rămasă automat:

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 5. Build (catkin_make) + source devel

```bash
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```

## 6. Reguli udev (Arduino Mega + LIDAR)

```bash
sudo cp ~/catkin_ws/src/robot_bringup/99-robot-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Rulează și scriptul din `robot_bringup/scripts/create_udev_rules` dacă vrei symlink-uri stabile pentru LIDAR (`/dev/rplidar`) — verifică-i conținutul înainte, ar putea necesita `sudo`.

## 7. Flashare firmware Arduino Mega (PlatformIO)

```bash
pip install -U platformio
cd ~/catkin_ws/src/robot_deploy
pio run -e megaatmega2560 -t upload
```

Dacă Jetson nu vede portul serial (`/dev/ttyACM0` sau `/dev/ttyUSB0`):

```bash
sudo usermod -aG dialout $USER
# apoi delogare/relogare sau reboot
```

## 8. Rulare pipeline complet

Deschide sesiuni separate (recomand `tmux`).

**Terminal 1 — bringup (bază, IMU, odometrie, LIDAR, motoare via rosserial):**

```bash
roslaunch robot_bringup robot.launch
```

**Terminal 2 — SLAM (alege un backend):**

```bash
roslaunch robot_slam robot_gmapping.launch
# sau: robot_hector.launch / robot_gmapping_imu.launch / robot_karto.launch / robot_cartographer.launch
```

**Terminal 3 — teleop (plimbă robotul să construiască harta):**

```bash
roslaunch robot_teleop robot_teleop_key.launch
```

**Terminal 4 — export model (salvează harta SLAM ca "model" pentru navigație):**

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/my_map
```

Produce `my_map.pgm` + `my_map.yaml`. Pentru a-l folosi ca hartă implicită de navigație, suprascrie fișierele existente:

```bash
cp ~/maps/my_map.pgm  ~/catkin_ws/src/robot_navigation/maps/map.pgm
cp ~/maps/my_map.yaml ~/catkin_ws/src/robot_navigation/maps/map.yaml
```

**Terminal 5 — navigație cu harta exportată (după ce oprești SLAM):**

```bash
roslaunch robot_navigation robot_navigation.launch
```

## Sanity checks utile

```bash
rostopic list
rosnode list
rostopic echo /odom -n1
```
