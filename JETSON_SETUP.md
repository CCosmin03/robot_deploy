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

## 8. Rulare live (pe robot)

Deschide sesiuni separate (recomand `tmux`).

### 8.1 Bringup (Terminal 1 — mereu primul, obligatoriu)

Pornește baza, IMU, odometria, LIDAR-ul și legătura serială cu Arduino (motoare).

```bash
roslaunch robot_bringup robot.launch
```

`odometry_node` publică mereu TF-ul `odom -> base_footprint`. La configurațiile cu EKF
(**c2**, **c5**) mai rulează și `ekf_localization_node`, care publică `odom_ekf -> base_footprint`
— dacă amândouă rulează în paralel, `base_footprint` ajunge cu doi părinți concurenți în
arborele TF și robotul "sare"/pâlpâie în RViz. La c2/c5, pornește bringup-ul cu:

```bash
roslaunch robot_bringup robot.launch publish_odom_tf:=false
```

La c1, c3, c4, c6 lași `publish_odom_tf` implicit (`true`, nu trebuie specificat).

### 8.2 SLAM (Terminal 2 — alege una din cele 6 configurații)

Toate deschid RViz automat (`open_rviz:=true` implicit).

| # | Comandă |
|---|---|
| **c1** GMap enc | `roslaunch robot_slam robot_gmapping.launch` |
| **c2** GMap+EKF | `roslaunch robot_slam robot_gmapping_imu.launch` |
| **c3** Hector | `roslaunch robot_slam robot_hector.launch` |
| **c4** Carto enc | `roslaunch robot_slam robot_cartographer_no_imu.launch` |
| **c5** Carto+EKF | `roslaunch robot_slam robot_cartographer_ekf.launch` |
| **c6** Carto IMU | `roslaunch robot_slam robot_cartographer.launch` (pornește și `move_base`, moștenit din structura turtlebot3 originală) |

Toate cele 6 au acum argumentul `load_robot_description` (implicit `false` — presupune că
`robot_description` e deja încărcat de bringup, ca mai sus). Nu trebuie specificat în modul
live descris aici; e relevant doar la testarea manuală offline (secțiunea 10.1).

### 8.3 Teleop (Terminal 3 — plimbă robotul să construiască harta)

```bash
roslaunch robot_teleop robot_teleop_key.launch
```

### 8.4 Salvează harta

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/my_map
```

Produce `my_map.pgm` + `my_map.yaml`. Pentru a-l folosi ca hartă implicită de navigație,
suprascrie fișierele existente:

```bash
cp ~/maps/my_map.pgm  ~/catkin_ws/src/robot_navigation/maps/map.pgm
cp ~/maps/my_map.yaml ~/catkin_ws/src/robot_navigation/maps/map.yaml
```

### 8.5 Navigație cu harta exportată (după ce oprești SLAM)

```bash
roslaunch robot_navigation robot_navigation.launch
```

## 9. Înregistrare date pentru evaluare (rosbag)

Pipeline complet, pas cu pas, pentru un scenariu (exemplu: S2 — buclă în jurul canapelei,
5 rulări independente conform metodologiei din capitolul 6).

**Terminal 1 — bringup** (fără flag-uri speciale la înregistrare — nu rulează niciun SLAM/EKF
în acest moment, deci `publish_odom_tf` rămâne implicit `true`):

```bash
roslaunch robot_bringup robot.launch
```

**Terminal 2 — poziționează robotul la punctul de start al scenariului**, apoi pornește
înregistrarea *înainte* să te miști (prima poziție din bag devine originea traiectoriei,
folosită de `gen_gt_odom.py`):

```bash
mkdir -p ~/recordings/s2
cd ~/recordings/s2
rosbag record /scan /odom /imu /tf /tf_static -O s2_r1.bag
```

**Terminal 3 — teleop:**

```bash
roslaunch robot_teleop robot_teleop_key.launch
```

Parcurge traseul dreptunghiular în jurul canapelei, revino aproape de punctul de start
(închiderea buclei), apoi `Ctrl+C` pe `rosbag record` (terminal 2) ca să oprești
înregistrarea. Repetă pentru `s2_r2.bag` ... `s2_r5.bag` (5 rulări independente, robotul
repornit din același punct de fiecare dată). Bringup-ul (terminal 1) și teleop-ul
(terminal 3) rămân pornite între rulări — doar `rosbag record` se oprește/repornește
cu nume diferit de fiecare dată.

La final ar trebui să ai:

```
~/recordings/s2/s2_r1.bag
~/recordings/s2/s2_r2.bag
~/recordings/s2/s2_r3.bag
~/recordings/s2/s2_r4.bag
~/recordings/s2/s2_r5.bag
```

Numele fișierelor nu contează pentru `run_scenario.sh` (le ia pe toate cele cu extensia
`.bag` din director, sortate alfabetic → run1, run2, ...), dar convenția `s2_rN.bag`
ține ordinea clară.

## 10. Evaluare offline (rosbag replay)

### 10.1 Manual, pentru o singură configurație

Fără bringup rulând, nimic nu încarcă `robot_description` — adaugă
`load_robot_description:=true` pe comanda de SLAM (nu mai trebuie terminal separat):

```bash
roscore &
rosparam set /use_sim_time true

# load_robot_description:=true incarca robot_description + porneste
# robot_state_publisher, ca sa vezi si modelul robotului in RViz (SLAM-ul
# merge si fara asta, e doar cosmetic — transformarile statice vin din
# /tf_static redat din bag oricum)
roslaunch robot_slam robot_gmapping.launch use_sim_time:=true load_robot_description:=true
# asteapta ~10s sa se initializeze nodul SLAM
```

Redarea bag-ului — la **c2** și **c5** exclude `/tf` (bag-ul are înregistrat `odom -> base_footprint`
de la vechiul teleop/bringup, care ar intra în conflict cu EKF-ul live la fel ca la 8.1):

| Config | `--topics` la `rosbag play` |
|---|---|
| c1, c3, c4, c6 | `/scan /odom /imu /tf /tf_static` (sau fără `--topics`, redă tot) |
| c2, c5 | `/scan /odom /imu /tf_static` (**fără** `/tf`) |

```bash
rosbag play nume_inregistrare.bag --clock --rate 1.0 --delay 3 --topics /scan /odom /imu /tf_static
```

După ce se termină replay-ul: `rosrun map_server map_saver -f nume_harta`.

### 10.2 Automat, toate cele 6 configurații x 5 rulări pentru un scenariu

```bash
cd ~/catkin_ws/src/robot_slam/evaluation/scripts
./run_scenario.sh s2 ~/recordings/s2/
```

Rulează automat toate cele 6 configurații pe fiecare din cele (maxim 5) bag-uri găsite
în `~/recordings/s2/` — 30 de experimente pentru S2. La prima rulare generează și harta
GT geometrică (`real_map_s2_sofa.pgm/.yaml`, din `scenario_config.yaml`) și referința
odometrică per bag (`s2_gt_r1.tum` ... `s2_gt_r5.tum`), dacă nu există deja. Rezultatele
apar în `evaluation/results/c1_s2_r1/` ... `c6_s2_r5/`.

`run_experiment.sh` (apelat intern) are deja `open_rviz:=false` pe toate configurațiile
(rulează headless, fără DISPLAY) și filtrează `/tf` la redare pentru c2/c5, la fel ca mai sus.

La final, raport agregat:

```bash
python3 generate_report.py --results-dir ../results --output ../reports/report.txt --csv ../reports/report.csv
```

## Sanity checks utile

```bash
rostopic list
rosnode list
rostopic echo /odom -n1
```
