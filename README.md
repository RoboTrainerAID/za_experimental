# ZA Experimental for Robotrainer v3
Build only this package
```bash
# shortcut for workspace src folder
ws
catkin build za_experimental
source devel/setup.bash
```

## How to launch
```bash
# Autostart
# ROS-Master + State_publisher + Laserscanner + Diagnostics + FTS.launch + Teleop

# LED + phidgets + battery
srt 

# RQT with node list service call and errors
roslaunch za_experimental rqt_overview.launch

# RVIZ with TF + Laser + Map + Model
roslaunch za_experimental rviz.launch
# roslaunch za_experimental rviz.launch standalone:=true

# Controller + AMCL + Map_server + Driver
# entspricht rt2_adaptive
roslaunch za_experimental rt2.launch

# Init drivers
rt2_init

# Recover drivers from emergency
rt2_recover

# Launch cameras
roslaunch robotrainer_bringup rt2_camera_tracking.launch

# Launch gait estimation?
```

## How to start controller parameterization
```bash
srt
roslaunch za_experimental rt2.launch
rt2_init
roslaunch za_experimental rqt_reconfigure.launch
# click: activate_force_parameterization
# follow instructions in console
# 1. forward
# 2. left
# 3. right
# 4. turn left
# 5. turn right
```

## How to start Spatial Control Actions (SCA) or Modalities
```bash
# Start panel and data_srv
roslaunch robotrainer_panel robotrainer.launch
```


```bash
rosrun plotjuggler PlotJuggler
rosrun plotjuggler PlotJuggler -n -l src/za_experimental/config/plotjuggler.xml
rviz -d src/za_experimental/config/data.rviz
```

Start cob gazebo simulation
```bash
export ROBOT=robotrainer2
export ROBOT_ENV=ipa-apartment
roslaunch cob_bringup_sim robot.launch

roslaunch za_experimental simulation.launch
```

Show urdf in rviz
```bash
# roslaunch robotrainer_bringup test_description.launch
roslaunch za_experimental rviz.launch
```

Start RT2
```bash
# roslaunch robotrainer_bringup rt2.launch

# Problem:
# map location original:
# $(arg pkg_env_config)/$(arg robot_env)/map.yaml
# cob_default_env_config/ipr-robotrainer/map.yaml

# map location new:
# cob_default_env_config/envs/ipa-apartment/map.yaml

roslaunch za_experimental rt2.launch
```

## Default kinematic config
```bash
# default is angular 5, linear 1
rosservice call /robotrainer_hw/set_state "{angular_key: 5, linear_key: 1, without_controller_updates: True}"

# Current config medium large footprint: angular 3, linear 1
rosservice call /robotrainer_hw/set_state "{angular_key: 3, linear_key: 1, without_controller_updates: True}"

# Get current state:
rosservice call /robotrainer_hw/get_state
```

## First WORKING Test

```bash
# should autostart on startup
# roslaunch robotrainer_bringup sr3_on_startup.launch

# starts with alias 'srt' or desktop bash file
rosservice call /robotrainer_hw/set_state "{angular_key: 3, linear_key: 1, without_controller_updates: True}"
roslaunch robotrainer_bringup sr3_on_login.launch

# necessary to publish tf and urdf
# roslaunch robotrainer_bringup test_description.launch

# alias rt2_adaptive also just rt2.launch
roslaunch robotrainer_bringup rt2_adaptive.launch

# dashboard to see which errors remain
roslaunch sr2_dashboard sr2_dashboard_new.launch
# or just with rqt and Diagnostic Viewer Plugin
rqt

# Init controllers
rosservice call /base/driver/init
```

## Mapping

```bash
roslaunch robotrainer_bringup rt2_mapping.launch
# Save generated files to path:
cd ~/workspace/ros_ws_melodic_robotrainer/src/cob_environments/cob_default_env_config/iras
rosrun map_server map_saver -f map
```

## Call Action with GUI

```bash
rosrun actionlib axclient.py /name_of_the_action
```

## Erkenntnisse from .bash_history

```bash
# Why do I need to set sync signal?
grep set_digital .bash_history 
grep phidgets set_digital
rosservice list | grep phidgets
rosservice call /phidgets/base_monitor/set_digital "uri: 'external_sync'
state: 1" 
rosservice call /phidgets/base_monitor/set_digital "uri: 'sync_signal'
state: 1" 
rosservice call /phidgets/base_monitor/set_digital "uri: 'sync_signal'
state: 0" 

# What is this used for? Calls sca sync signal and deviation/reset
rt2_call_start_services

# -> Both sync and deviation reset are automatically called from user_study_manager.py which is launched by alias user_study_manager

# In the folder .RoboTrainer there are status files
cat /home/robotrainer/.RoboTrainer/study_manager_status.status

# Commands for one evaluation session
srt
user_study_manager
eval_record_cam_bg
rt2_camera
eval_record_base_bg
roscds
rt2_adaptive
rt2_performance
rosservice call /base/driver/init
```

## Commands as alias
from [./robotrainer/robotrainer_config/scripts/command_aliases.bash](../robotrainer/robotrainer_config/scripts/command_aliases.bash)
```bash
## Commands
alias srt=`rospack find robotrainer_bringup`/scripts/robotrainer_on_login.bash
alias start_transport=`rospack find robotrainer_bringup`/scripts/robotrainer_transport.bash

alias rt2_mapping="roslaunch robotrainer_bringup rt2_mapping.launch"
alias rt2_adaptive="roslaunch robotrainer_bringup rt2_adaptive.launch"
alias rt2_camera="roslaunch robotrainer_bringup rt2_camera_tracking.launch"
alias rt2_performance="roslaunch robotrainer_bringup rt2_user_performance.launch"
alias user_study_manager="roslaunch robotrainer_bringup rt2_user_study_manager.launch"

## Kinematics
alias kin_rt2_11='rosservice call /robotrainer_hw/set_state "{angular_key: 1, linear_key: 1}"'
alias kin_rt2_14='rosservice call /robotrainer_hw/set_state "{angular_key: 1, linear_key: 4}"'
alias kin_rt2_51='rosservice call /robotrainer_hw/set_state "{angular_key: 5, linear_key: 1}"'
alias kin_rt2_54='rosservice call /robotrainer_hw/set_state "{angular_key: 5, linear_key: 4}"'

alias eval_record_base_bg="bash `rospack find robotrainer_config`/scripts/record_bag_in_background.bash .RoboTrainer/study_record_base.status `rospack find robotrainer_config`/scripts/record_base_values.bash ~/RT2_Data/RoSy_Eval/"

alias eval_record_cam_bg="bash `rospack find robotrainer_config`/scripts/record_bag_in_background.bash .RoboTrainer/study_record_cam.status `rospack find robotrainer_config`/scripts/record_camera_values.bash ~/RT2_Data/RoSy_Eval/Cams"

alias rt2_call_start_services="rosservice call /rt2_sca_sync_node/send_sync_signal_1s \"{}\" & rosservice call /robotrainer_deviation/reset \"{}\""
```

## Autostart
1. Files to autostart on login are in /home/robotrainer/.config/autostart/robotrainer_on_login.bash.desktop
    ```bash
    Exec="/home/robotrainer/robotrainer_on_login.bash"
    ```

2. Autostart on startup:
    robotrainer.service loaded active running "bringup robotrainer"
    this deamon starts: `roslaunch /tmp/robotrainer.launch`
    systemctl robotrainer.service located in:
    `/usr/sbin/robotrainer-start`
    summarized content:
    ```bash
    source /home/robotrainer/workspace/ros_ws_melodic_robotrainer/devel/setup.bash
    JOB_FOLDER=/etc/ros/melodic/robotrainer.d
    log_path="/tmp"

    export ROS_HOSTNAME=$(hostname)
    export ROS_MASTER_URI=http://127.0.0.1:11311
    export ROS_HOME=${ROS_HOME:=$(echo ~robotrainer)/.ros}
    export ROS_LOG_DIR=$log_path

    XACRO_FILENAME=$log_path/robotrainer.xacro
    XACRO_ROBOT_NAME=$(echo "robotrainer" | cut -d- -f1)
    rosrun robot_upstart mkxacro $JOB_FOLDER $XACRO_ROBOT_NAME > $XACRO_FILENAME
    URDF_FILENAME=$log_path/robotrainer.urdf
    rosrun xacro xacro $XACRO_FILENAME -o $URDF_FILENAME
    export ROBOT_URDF_FILENAME=$URDF_FILENAME

    LAUNCH_FILENAME=$log_path/robotrainer.launch
    rosrun robot_upstart mklaunch $JOB_FOLDER > $LAUNCH_FILENAME

    # Punch it.
    setuidgid robotrainer roslaunch $LAUNCH_FILENAME &
    PID=$!
    log info "robotrainer: Started roslaunch as background process, PID $PID, ROS_LOG_DIR=$ROS_LOG_DIR"
    echo "$PID" > $log_path/robotrainer.pid
    wait "$PID"
    ```

    Launch file in /tmp/robotrainer.launch
    ```xml
    <launch>
        <!-- Generated from launch files in /etc/ros/melodic/robotrainer.d -->
    <include file="/etc/ros/melodic/robotrainer.d/sr3_startup_script.launch" />
    </launch>
    ```

    JOB_FOLDER for autostart scripts /etc/ros/melodic/robotrainer.d
    /etc/ros/melodic/robotrainer.d/reinit_can.bash
    ```bash
    #!/bin/bash
    ## Reset CAN devices
    #rmmod pcan
    #modprobe pcan
    ip link set can0 type can bitrate 500000
    ip link set can0 up
    ip link set can1 type can bitrate 1000000
    ip link set can1 up
    ```

    /etc/ros/melodic/robotrainer.d/sr3_startup_script.launch
    ```xml
    <?xml version="1.0"?>
    <launch>
        <include file="$(find robotrainer_bringup)/robot/sr3_on_startup.launch" />
    </launch>
    ```


## Laserscanner 3 removed
Changes made in robotrainer_bringup/robot/sr3_on_startup.launch  
For 3 Laserscanner:
```xml
<rosparam subst_value="True" param="input_scans">['/base_laser_left/scan', '/base_laser_right/scan', '/base_laser_back/scan']</rosparam>
```

For 2 Laserscanners, back removed:
```xml
<rosparam subst_value="True" param="input_scans">['/base_laser_left/scan', '/base_laser_right/scan']</rosparam>
```

## Troubleshooting
```bash
# ERROR:
libGL error: pci id for fd 15: 8086:46a6, driver (null)
libGL error: No driver found
libGL error: failed to load driver: (null)
libGL error: failed to open drm device: Permission denied
libGL error: failed to load driver: iris

# Tried so far
sudo usermod -a -G video $USER
newgrp video
```

## Launch RoboTrainer v2 on device
From: [robotrainer/README.md](../robotrainer/README.md)
```bash
rosparam load src/robotrainer_parameters/yamls/demo_scenario.yaml
rosparam load src/robotrainer_bringup/configs/modalities.yaml

# Not sure if the following only works on device
# ssh modalities-sr2
roslaunch sr2_bringup robotrainer.launch

rosservice call /base/driver/init

rviz

rqt
```

## Dependencies GitHub Forks
- -b robotrinerV2 https://github.com/KITrobotics/cob_robots.git
- -b robotrainerV2 https://github.com/KITrobotics/cob_common.git
- -b robotrianerV2 https://github.com/KITrobotics/cob_calibration_data.git
- -b cpu_monitor_str_repair https://github.com/KITrobotics/cob_command_tools.git
- -b towards_melodic https://github.com/KITrobotics/cob_extern.git

