# ZA Experimental for Robotrainer v3

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
roslaunch za_experimental rviz_urdf.launch
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
rosrun map_server map_saver -f iras
# Copy generated files to path:
# ~/workspace/ros_ws_melodic_robotrainer/src/cob_environments/cob_default_env_config/iras
```

## Call Action with GUI

```bash
rosrun actionlib axclient.py /name_of_the_action
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
```
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

## Debugging output from rt2_adaptive

```log
roslaunch robotrainer_bringup rt2_adaptive.launch
... logging to /tmp/robotrainer/ros_logs/685ee0c2-11f2-11ef-ba04-e0d55e1939dd/roslaunch-iar-ipr-sr3-4096.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://iar-ipr-sr3:41659/

SUMMARY
========

PARAMETERS
 * /amcl/base_frame_id: base_footprint
 * /amcl/global_frame_id: map
 * /amcl/gui_publish_rate: -1.0
 * /amcl/kld_err: 0.01
 * /amcl/kld_z: 0.99
 * /amcl/laser_lambda_short: 0.1
 * /amcl/laser_likelihood_max_dist: 2.0
 * /amcl/laser_max_beams: 30
 * /amcl/laser_max_range: 29.5
 * /amcl/laser_model_type: likelihood_field
 * /amcl/laser_sigma_hit: 0.2
 * /amcl/laser_z_hit: 0.95
 * /amcl/laser_z_max: 0.05
 * /amcl/laser_z_rand: 0.05
 * /amcl/laser_z_short: 0.1
 * /amcl/max_particles: 5000
 * /amcl/min_particles: 100
 * /amcl/odom_alpha1: 0.2
 * /amcl/odom_alpha2: 0.2
 * /amcl/odom_alpha3: 0.2
 * /amcl/odom_alpha4: 0.2
 * /amcl/odom_alpha5: 0.2
 * /amcl/odom_frame_id: odom_combined
 * /amcl/odom_model_type: omni
 * /amcl/recovery_alpha_fast: 0.0
 * /amcl/recovery_alpha_slow: 0.0
 * /amcl/resample_interval: 2
 * /amcl/transform_tolerance: 0.1
 * /amcl/update_min_a: 0.5
 * /amcl/update_min_d: 0.2
 * /base/Calibration/Offset/T_between_meas: 0.01
 * /base/Calibration/Offset/force/x: -19.1041026664
 * /base/Calibration/Offset/force/y: -1.5473841984
 * /base/Calibration/Offset/force/z: 0.933212123875
 * /base/Calibration/Offset/isStatic: False
 * /base/Calibration/Offset/n_measurements: 200
 * /base/Calibration/Offset/torque/x: -0.246421458952
 * /base/Calibration/Offset/torque/y: 0.0373995095506
 * /base/Calibration/Offset/torque/z: 0.0604286034941
 * /base/FTS/auto_init: True
 * /base/FTS/base_identifier: 32
 * /base/FTS/fts_name: ATI_45_Mini
 * /base/HWComm/path: can1
 * /base/HWComm/type: 5
 * /base/Node/ft_pub_freq: 200
 * /base/Node/ft_pull_freq: 800
 * /base/Node/sensor_frame: fts_reference_link
 * /base/Node/sensor_hw: ati_force_torque/...
 * /base/Node/sim: False
 * /base/Node/static_application: True
 * /base/Node/transform_frame: base_link
 * /base/Publish/gravity_compensated: False
 * /base/Publish/low_pass: False
 * /base/Publish/moving_mean: False
 * /base/Publish/output_data: True
 * /base/Publish/sensor_data: False
 * /base/Publish/threshold_filtered: False
 * /base/Publish/transformed_data: False
 * /base/defaults/steer_ctrl/d_phi_max: 10.0
 * /base/defaults/steer_ctrl/damp: 2.75
 * /base/defaults/steer_ctrl/dd_phi_max: 40.0
 * /base/defaults/steer_ctrl/spring: 15.0
 * /base/defaults/steer_ctrl/virt_mass: 0.1
 * /base/driver/LowPassFilter/name: LowPassFilter
 * /base/driver/LowPassFilter/params/DampingFrequency: 15.0
 * /base/driver/LowPassFilter/params/DampingIntensity: -6.0
 * /base/driver/LowPassFilter/params/SamplingFrequency: 200.0
 * /base/driver/LowPassFilter/type: iirob_filters/Low...
 * /base/driver/MovingMeanFilter/name: MovingMeanFilter
 * /base/driver/MovingMeanFilter/params/divider: 4
 * /base/driver/MovingMeanFilter/type: filters/MovingMea...
 * /base/driver/ThresholdFilter/name: ThresholdFilter
 * /base/driver/ThresholdFilter/params/angular_threshold: 0.3
 * /base/driver/ThresholdFilter/params/linear_threshold: 2.5
 * /base/driver/ThresholdFilter/type: iirob_filters/Thr...
 * /base/driver/bus/device: can0
 * /base/driver/defaults/dcf_overlay/1016sub1: 0x7F0064
 * /base/driver/defaults/dcf_overlay/6083: 1000000
 * /base/driver/defaults/dcf_overlay/6084: 1000000
 * /base/driver/defaults/dcf_overlay/6099sub1: 100000
 * /base/driver/defaults/dcf_overlay/6099sub2: 10000
 * /base/driver/defaults/dcf_overlay/60C5: 1000000
 * /base/driver/defaults/dcf_overlay/60C6: 1000000
 * /base/driver/defaults/eds_file: robots/common/Elm...
 * /base/driver/defaults/eds_pkg: cob_hardware_config
 * /base/driver/defaults/eff_from_device: obj6078/1000.0*9.76
 * /base/driver/drive_wheel/dcf_overlay/6098: 35
 * /base/driver/drive_wheel/eff_from_device: obj6078/1000.0*14.14
 * /base/driver/drive_wheel/vel_from_device: p1 != p1 || obj20...
 * /base/driver/heartbeat/msg: 77f#05
 * /base/driver/heartbeat/rate: 20
 * /base/driver/nodes/bl_caster_r_wheel_joint/dcf_overlay/6098: 35
 * /base/driver/nodes/bl_caster_r_wheel_joint/eff_from_device: obj6078/1000.0*14.14
 * /base/driver/nodes/bl_caster_r_wheel_joint/id: 4
 * /base/driver/nodes/bl_caster_r_wheel_joint/vel_from_device: p1 != p1 || obj20...
 * /base/driver/nodes/bl_caster_rotation_joint/dcf_overlay/607C: 156454
 * /base/driver/nodes/bl_caster_rotation_joint/dcf_overlay/6098: 19
 * /base/driver/nodes/bl_caster_rotation_joint/id: 3
 * /base/driver/nodes/br_caster_r_wheel_joint/dcf_overlay/6098: 35
 * /base/driver/nodes/br_caster_r_wheel_joint/eff_from_device: obj6078/1000.0*14.14
 * /base/driver/nodes/br_caster_r_wheel_joint/id: 6
 * /base/driver/nodes/br_caster_r_wheel_joint/vel_from_device: p1 != p1 || obj20...
 * /base/driver/nodes/br_caster_rotation_joint/dcf_overlay/607C: 29656
 * /base/driver/nodes/br_caster_rotation_joint/dcf_overlay/6098: 19
 * /base/driver/nodes/br_caster_rotation_joint/id: 5
 * /base/driver/nodes/f_caster_r_wheel_joint/dcf_overlay/6098: 35
 * /base/driver/nodes/f_caster_r_wheel_joint/eff_from_device: obj6078/1000.0*14.14
 * /base/driver/nodes/f_caster_r_wheel_joint/id: 2
 * /base/driver/nodes/f_caster_r_wheel_joint/vel_from_device: p1 != p1 || obj20...
 * /base/driver/nodes/f_caster_rotation_joint/dcf_overlay/607C: 158457
 * /base/driver/nodes/f_caster_rotation_joint/dcf_overlay/6098: 19
 * /base/driver/nodes/f_caster_rotation_joint/id: 1
 * /base/driver/reset_errors_before_recover: False
 * /base/driver/sync/interval_ms: 20
 * /base/driver/sync/overflow: 0
 * /base/driver/use_fts: True
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/parametrization/rot_base/minTorque: 10.0
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/parametrization/rot_base/returnForce: 8.0
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/parametrization/rot_base/springConst: 60
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/parametrization/x_base/minForce: 40.0
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/parametrization/x_base/returnForce: 20.0
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/parametrization/x_base/springConst: 200
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/parametrization/y_base/minForce: 40.0
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/parametrization/y_base/returnForce: 15.0
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/parametrization/y_base/springConst: 150
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/damping_adaption/max/rot: 1.0
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/damping_adaption/max/x: 1.0
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/damping_adaption/max/y: 1.0
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/damping_adaption/min/rot: 0.4
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/damping_adaption/min/x: 0.4
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/damping_adaption/min/y: 0.4
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/force_scale_max_vel/rot: 0.3
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/force_scale_max_vel/x: 0.45
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/force_scale_max_vel/y: 0.45
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/force_scale_zero_vel/rot: 1.0
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/force_scale_zero_vel/x: 1.2
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/force_scale_zero_vel/y: 1.2
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/tanh/rot: 1.2
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/tanh/x: 1.1
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/tanh/y: 1.2
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/transition/transition_rate: 0.35
 * /base/fts_adaptive_force_controller/FTSAdaptiveForceController/velocity_adaption/use_passive_behavior_ctrlr: False
 * /base/fts_adaptive_force_controller/FTSBaseController/backwards_max_force_scale: 0.5
 * /base/fts_adaptive_force_controller/FTSBaseController/backwards_max_vel_scale: 0.7
 * /base/fts_adaptive_force_controller/FTSBaseController/debug: True
 * /base/fts_adaptive_force_controller/FTSBaseController/global_control_actions/adaptive_cor/adapt_cor: False
 * /base/fts_adaptive_force_controller/FTSBaseController/global_control_actions/adaptive_cor/cor_x: 0.5
 * /base/fts_adaptive_force_controller/FTSBaseController/global_control_actions/adaptive_cor/cor_y: 0.0
 * /base/fts_adaptive_force_controller/FTSBaseController/global_control_actions/global_counterforce/counterforce_x: 20.0
 * /base/fts_adaptive_force_controller/FTSBaseController/global_control_actions/global_counterforce/counterforce_y: 0.0
 * /base/fts_adaptive_force_controller/FTSBaseController/global_control_actions/global_counterforce/countertorque_z: 0.0
 * /base/fts_adaptive_force_controller/FTSBaseController/global_control_actions/global_counterforce/enabled: False
 * /base/fts_adaptive_force_controller/FTSBaseController/no_hw_output: False
 * /base/fts_adaptive_force_controller/FTSBaseController/reversed_max_force_scale: 1
 * /base/fts_adaptive_force_controller/FTSBaseController/reversed_max_vel_scale: 0.5
 * /base/fts_adaptive_force_controller/FTSBaseController/rot_controller: True
 * /base/fts_adaptive_force_controller/FTSBaseController/rot_damping: -1
 * /base/fts_adaptive_force_controller/FTSBaseController/rot_gain: 1.0
 * /base/fts_adaptive_force_controller/FTSBaseController/rot_mass: -1
 * /base/fts_adaptive_force_controller/FTSBaseController/rot_max_rot_vel: 1.2
 * /base/fts_adaptive_force_controller/FTSBaseController/rot_max_torque: 30.0
 * /base/fts_adaptive_force_controller/FTSBaseController/rot_min_torque: 0.3
 * /base/fts_adaptive_force_controller/FTSBaseController/rot_reversed: False
 * /base/fts_adaptive_force_controller/FTSBaseController/rot_time_const: 0.3
 * /base/fts_adaptive_force_controller/FTSBaseController/update_rate: 200.0
 * /base/fts_adaptive_force_controller/FTSBaseController/x_damping: -1
 * /base/fts_adaptive_force_controller/FTSBaseController/x_force_controller: True
 * /base/fts_adaptive_force_controller/FTSBaseController/x_gain: 1.3
 * /base/fts_adaptive_force_controller/FTSBaseController/x_mass: -1
 * /base/fts_adaptive_force_controller/FTSBaseController/x_max_force: 100.0
 * /base/fts_adaptive_force_controller/FTSBaseController/x_max_vel: 1.6
 * /base/fts_adaptive_force_controller/FTSBaseController/x_min_force: 3.0
 * /base/fts_adaptive_force_controller/FTSBaseController/x_time_const: 0.5
 * /base/fts_adaptive_force_controller/FTSBaseController/y_damping: -1
 * /base/fts_adaptive_force_controller/FTSBaseController/y_force_controller: True
 * /base/fts_adaptive_force_controller/FTSBaseController/y_gain: 1.3
 * /base/fts_adaptive_force_controller/FTSBaseController/y_mass: -1
 * /base/fts_adaptive_force_controller/FTSBaseController/y_max_force: 100.0
 * /base/fts_adaptive_force_controller/FTSBaseController/y_max_vel: 1.6
 * /base/fts_adaptive_force_controller/FTSBaseController/y_min_force: 3.0
 * /base/fts_adaptive_force_controller/FTSBaseController/y_reversed: False
 * /base/fts_adaptive_force_controller/FTSBaseController/y_time_const: 0.5
 * /base/fts_adaptive_force_controller/max_rot_velocity: 1.0
 * /base/fts_adaptive_force_controller/max_trans_velocity: 1.5
 * /base/fts_adaptive_force_controller/publish_rate: 50
 * /base/fts_adaptive_force_controller/required_drive_mode: 3
 * /base/fts_adaptive_force_controller/type: robotrainer_contr...
 * /base/fts_adaptive_force_controller/use_fts: True
 * /base/joint_names: ['f_caster_rotati...
 * /base/joint_state_controller/publish_rate: 50
 * /base/joint_state_controller/type: joint_state_contr...
 * /base/odometry_controller/child_frame_id: /base_footprint
 * /base/odometry_controller/cov_pose: 0.1
 * /base/odometry_controller/cov_twist: 0.1
 * /base/odometry_controller/frame_id: /odom_combined
 * /base/odometry_controller/publish_rate: 50
 * /base/odometry_controller/type: robotrainer_contr...
 * /base/odometry_controller/wheels: [{'steer': 'f_cas...
 * /base/stuck_detector/shutdown: False
 * /base/stuck_detector/threshold: 0.174533
 * /base/stuck_detector/timeout: 2.0
 * /base/twist_controller/defaults/steer_ctrl/d_phi_max: 10.0
 * /base/twist_controller/defaults/steer_ctrl/damp: 2.75
 * /base/twist_controller/defaults/steer_ctrl/dd_phi_max: 40.0
 * /base/twist_controller/defaults/steer_ctrl/spring: 15.0
 * /base/twist_controller/defaults/steer_ctrl/virt_mass: 0.1
 * /base/twist_controller/max_rot_velocity: 1.2
 * /base/twist_controller/max_trans_velocity: 1.2
 * /base/twist_controller/pub_divider: 1
 * /base/twist_controller/required_drive_mode: 3
 * /base/twist_controller/timeout: 0.5
 * /base/twist_controller/type: robotrainer_contr...
 * /base/twist_controller/wheels: [{'steer': 'f_cas...
 * /base/twist_mux/locks: [{'topic': 'twist...
 * /base/twist_mux/topics: [{'topic': 'twist...
 * /base/velocity_smoother/accel_lim_vx: 0.5
 * /base/velocity_smoother/accel_lim_vy: 0.5
 * /base/velocity_smoother/accel_lim_w: 0.5
 * /base/velocity_smoother/decel_factor: 1.5
 * /base/velocity_smoother/decel_factor_safe: 2.5
 * /base/velocity_smoother/frequency: 50.0
 * /base/velocity_smoother/robot_feedback: 1
 * /base/velocity_smoother/speed_lim_vx: 2.0
 * /base/velocity_smoother/speed_lim_vy: 1.0
 * /base/velocity_smoother/speed_lim_w: 1.2
 * /base/wheels: [{'steer': 'f_cas...
 * /leg_detection/KalmanFilter/A: [1, 0, 0.05, 0, 0...
 * /leg_detection/KalmanFilter/C: [1, 0, 0, 0, 0, 0...
 * /leg_detection/KalmanFilter/P: [0.45, 0, 0, 0, 0...
 * /leg_detection/KalmanFilter/Q: [0.1, 0, 0, 0, 0,...
 * /leg_detection/KalmanFilter/R: [0.012, 0, 0, 0.012]
 * /leg_detection/KalmanFilter/dt: 0.05
 * /leg_detection/KalmanFilter/m: 2
 * /leg_detection/KalmanFilter/n: 6
 * /leg_detection/KalmanFilter/x0: [0, 0, 0, 0, 0, 0]
 * /leg_detection/clusterTolerance: 0.07
 * /leg_detection/cluster_bounding_box_uncertainty: 0.04
 * /leg_detection/euclidian_dist_gate: 0.4
 * /leg_detection/frequency: 0.05
 * /leg_detection/in_free_space_threshold: 0.1
 * /leg_detection/isBoundingBoxTracking: False
 * /leg_detection/isOnePersonToTrack: True
 * /leg_detection/leg_radius: 0.1
 * /leg_detection/local_map_topic: /move_base/global...
 * /leg_detection/mahalanobis_dist_gate: 1.2
 * /leg_detection/maxClusterSize: 120
 * /leg_detection/max_cost: 999999.0
 * /leg_detection/max_cov: 0.81
 * /leg_detection/max_dist_btw_legs: 0.8
 * /leg_detection/max_neighbors_for_outlier_removal: 3
 * /leg_detection/max_nn_gating_distance: 1.0
 * /leg_detection/minClusterSize: 3
 * /leg_detection/min_dist_travelled: 0.25
 * /leg_detection/min_observations: 3
 * /leg_detection/min_predictions: 3
 * /leg_detection/occluded_dead_age: 10
 * /leg_detection/outlier_removal_radius: 0.07
 * /leg_detection/ref_point_x: -0.5
 * /leg_detection/ref_point_y: 0.0
 * /leg_detection/scan_topic: /scan_unified
 * /leg_detection/tracking_bounding_box_uncertainty: 0.2
 * /leg_detection/transform_link: base_link
 * /leg_detection/variance_observation: 0.25
 * /leg_detection/with_map: False
 * /leg_detection/x_lower_limit: -1.8
 * /leg_detection/x_upper_limit: -0.3
 * /leg_detection/y_lower_limit: -0.7
 * /leg_detection/y_upper_limit: 0.7
 * /leg_detection/z_coordinate: 0.0
 * /modalities/path_tracking/controller_update_rate: 200.0
 * /modalities/path_tracking/max_force: 100.0
 * /modalities/path_tracking/max_velocity: 1.2
 * /modalities/path_tracking/time_const_T: 0.8
 * /modalities/virtual_counterforce/controller_update_rate: 200.0
 * /modalities/virtual_counterforce/max_force: 100.0
 * /modalities/virtual_counterforce/max_velocity: 0.8
 * /modalities/virtual_counterforce/time_const_T: 0.5
 * /modalities/virtual_counterforce/trapezoid_max_at_percent_radius: 0.25
 * /modalities/virtual_forces/compensate_velocity: False
 * /modalities/virtual_forces/compensation_reference_velocity: 0.3
 * /modalities/virtual_forces/controller_update_rate: 200.0
 * /modalities/virtual_forces/linear_velocity_from_force_damping: True
 * /modalities/virtual_forces/max_force: 100.0
 * /modalities/virtual_forces/max_velocity: 0.8
 * /modalities/virtual_forces/time_const_T: 0.0
 * /modalities/virtual_forces/trapezoid_max_at_percent_radius: 0.25
 * /modalities/virtual_walls/controller_update_rate: 200.0
 * /modalities/virtual_walls/limit_human_velocity: True
 * /modalities/virtual_walls/max_force: 120.0
 * /modalities/virtual_walls/max_velocity: 1.2
 * /modalities/virtual_walls/time_const_T: 0.8
 * /modalities/virtual_walls/trapezoid_max_at_percent_radius: 0.25
 * /modalities/virtual_walls/wall_force: 100.0
 * /robotrainer/modalities_chain_config: [{'type': 'robotr...
 * /rosdistro: melodic
 * /rosversion: 1.14.6

NODES
  /
    amcl (amcl/amcl)
    leg_detection (leg_tracker/leg_tracker)
    map_server (map_server/map_server)
  /base/
    base_controller_loader (controller_manager/controller_manager)
    base_controller_spawner (controller_manager/controller_manager)
    driver (canopen_motor_node/canopen_motor_node)
    stuck_detector (cob_base_controller_utils/cob_stuck_detector)
    twist_marker (twist_mux/twist_marker)
    twist_mux (twist_mux/twist_mux)
    velocity_smoother (cob_base_velocity_smoother/velocity_smoother)

ROS_MASTER_URI=http://localhost:11311

process[base/driver-1]: started with pid [4111]
process[base/stuck_detector-2]: started with pid [4112]
process[base/base_controller_spawner-3]: started with pid [4133]
process[base/twist_mux-4]: started with pid [4134]
process[base/twist_marker-5]: started with pid [4136]
[ INFO] [1715693424.995118269]: Topic handler 'collision_velocity_filter' subscribed to topic 'twist_mux/command_safe': timeout = 0.250000s, priority = 10
[ INFO] [1715693424.997391339]: Topic handler 'navigation' subscribed to topic 'twist_mux/command_navigation': timeout = 0.250000s, priority = 20
[ INFO] [1715693424.998894859]: Topic handler 'syncmm' subscribed to topic 'twist_mux/command_syncmm': timeout = 0.500000s, priority = 60
[ INFO] [1715693425.000288810]: Topic handler 'script_server' subscribed to topic 'twist_mux/command_script_server': timeout = 0.500000s, priority = 70
[ INFO] [1715693425.002249882]: Topic handler 'teleop_keyboard' subscribed to topic 'twist_mux/command_teleop_keyboard': timeout = 0.500000s, priority = 80
[ INFO] [1715693425.003766212]: Topic handler 'teleop_android' subscribed to topic 'twist_mux/command_teleop_android': timeout = 0.500000s, priority = 90
process[base/velocity_smoother-6]: started with pid [4141]
[ INFO] [1715693425.005649401]: Topic handler 'teleop_joystick' subscribed to topic 'twist_mux/command_teleop_joy': timeout = 0.250000s, priority = 100
process[base/base_controller_loader-7]: started with pid [4148]
[ INFO] [1715693425.017932336]: Topic handler 'pause_navigation' subscribed to topic 'twist_mux/locks/pause_navigation': timeout = None, priority = 21
[ INFO] [1715693425.020488579]: Topic handler 'pause_teleop' subscribed to topic 'twist_mux/locks/pause_teleop': timeout = None, priority = 101
[ INFO] [1715693425.023818589]: Topic handler 'pause_all' subscribed to topic 'twist_mux/locks/pause_all': timeout = None, priority = 255
process[map_server-8]: started with pid [4153]
[ INFO] [1715693425.032755360]: Reconfigure request : 2.000000 1.000000 1.200000 0.500000 0.500000 0.500000 1.500000 2.500000
process[amcl-9]: started with pid [4155]
process[leg_detection-10]: started with pid [4158]
[ INFO] [1715693425.330619304]: Using fixed control period: 0.020000000
[ WARN] [1715693440.225596204]: No laser scan received (and thus no pose updates have been published) for 1715693440.225516 seconds.  Verify that data is being published on the /scan_unified topic.
[ WARN] [1715693455.225522382]: No laser scan received (and thus no pose updates have been published) for 1715693455.225440 seconds.  Verify that data is being published on the /scan_unified topic.
[ INFO] [1715693461.813464274]: Initializing XXX
[ INFO] [1715693461.814066090]: Current state: 1 device error: system:0 internal_error: 0 (OK)
[ INFO] [1715693461.814644823]: Current state: 2 device error: system:0 internal_error: 0 (OK)
[ WARN] [1715693470.225516527]: No laser scan received (and thus no pose updates have been published) for 1715693470.225444 seconds.  Verify that data is being published on the /scan_unified topic.
[ERROR] [1715693471.453532228]: Tried to advertise a service that is already advertised in this node [/base/driver/set_parameters]
[ERROR] [1715693471.460354698]: Tried to advertise a service that is already advertised in this node [/base/driver/set_parameters]
[ERROR] [1715693471.468858455]: Tried to advertise a service that is already advertised in this node [/base/driver/set_parameters]
[ INFO] [1715693471.477061662]: Sensor type ati_force_torque/ATIForceTorqueSensorHWCan was successfully loaded.
[ INFO] [1715693471.504113389]: Moving Mean Filter Params: Divider: 4 
[ INFO] [1715693471.506293348]: Low Pass Filter Params: Sampling Frequency:200.000000, Damping Frequency:15.000000, Damping Intensity:-6.000000; Divider: 1 
[ WARN] [1715693471.506574862]: GravityCompensation configuration not found. It will not be used!
[ INFO] [1715693471.508108327]: Threshhold Filter Params: Threshold: 0.000000; Treshold lin.: 2.500000; Threshold Anglular: 0.300000
[ INFO] [1715693471.629770553]: Calibrating sensor. Plase wait...
[ INFO] [1715693473.648419370]: Autoinit: FTS initialized!
[ INFO] [1715693473.653551802]: FTS Transform not yet initalized, Trying to get one...
[ INFO] [1715693473.653826846]: Got FTS Transform for static application!
Loaded 'joint_state_controller'
[ INFO] [1715693476.841140801]: configure steer 0: s: 15.000000, d: 2.750000, m: 0.100000, v: 10.000000, a: 40.000000
[ INFO] [1715693476.860917387]: configure steer 1: s: 15.000000, d: 2.750000, m: 0.100000, v: 10.000000, a: 40.000000
[ INFO] [1715693476.881879633]: configure steer 2: s: 15.000000, d: 2.750000, m: 0.100000, v: 10.000000, a: 40.000000
Loaded 'twist_controller'
[ WARN] [1715693476.915335322]: !!!ParamParser (1. try): could not get dynamic transformation!!!: "base_link" passed to lookupTransform argument target_frame does not exist. 
[base/base_controller_loader-7] process has finished cleanly
log file: /tmp/robotrainer/ros_logs/685ee0c2-11f2-11ef-ba04-e0d55e1939dd/base-base_controller_loader-7*.log
Loaded 'odometry_controller'
[ INFO] [1715693479.987952453]: Init FTSBaseController!
[ WARN] [1715693480.364934039]: !!!ParamParser (1. try): could not get dynamic transformation!!!: "base_link" passed to lookupTransform argument target_frame does not exist. 
[ WARN] [1715693483.494645462]: Controller is not started, therefore, the parameters can not be set! 
               Returning the parameters from controllers
[ INFO] [1715693483.554228284]: Trying to load modalities so that they can be configured!
[ INFO] [1715693483.586457962]: Using linear velocity calculation V=F/D
[ INFO] [1715693483.586512286]: VirtualForces: Dynamic reconfigure.
    max_force:            100
    max_velocity          0.8
    controller_update_rate: 200
    time_const_T: 0
    damping_for_lienar: 0.008
[ INFO] [1715693483.591880573]: [fts_base_controller.cpp] Force_Modality loaded
[ INFO] [1715693483.596223523]: [modalities_controller_base.h] constructor
[ERROR] [1715693483.614527190]: Tried to advertise a service that is already advertised in this node [/robotrainer/set_parameters]
[ERROR] [1715693483.625044918]: Tried to advertise a service that is already advertised in this node [/modalities/virtual_forces/set_parameters]
[ INFO] [1715693483.634232703]: Using linear velocity calculation V=F/D
[ INFO] [1715693483.634283428]: VirtualForces: Dynamic reconfigure.
    max_force:            100
    max_velocity          0.8
    controller_update_rate: 200
    time_const_T: 0
    damping_for_lienar: 0.008
[ INFO] [1715693483.637511533]: VirtualForcesModalitie loaded
[ WARN] [1715693483.637593623]: SEVERE WARNING!!! Attempting to unload library while objects created by this loader exist in the heap! You should delete your objects before attempting to unload the library or destroying the ClassLoader. The library will NOT be unloaded.
[ INFO] [1715693483.637707456]: [fts_base_controller.cpp] Force_Controller_Modality loaded
[ INFO] [1715693483.637756584]: OmnidirectionalMode constructor
[ INFO] [1715693483.637777640]: In set_mode: 0
[ INFO] [1715693483.641499549]: Init FTSAdaptiveForceController!
[ WARN] [1715693483.687233479]: Controller is not started, therefore, the parameters can not be set! 
     Returning the parameters from controllers
[ INFO] [1715693483.697902074]: [ADAPT INIT] - Starting initialization of passive behavior controller with gain:[1.30, 1.30, 1.00] timeConst:[0.50, 0.50, 0.30]
[ INFO] [1715693483.699665575]: [PASS BHVR INIT CHAINED] - Setting base ctrlr Values to gain:[1.30, 1.30, 1.00] and timeConst:[0.50, 0.50, 0.30]
[ INFO] [1715693483.699701296]: [PASS BHVR INIT AS CHAINED MODE]
[ INFO] [1715693483.724480842]: Dyn reconfigure passiveBehaviorCtrlBase
[ INFO] [1715693483.724645601]: USING: SlidingIntegral_MultiDimension WITH conditionalDetection of instabilities
[ INFO] [1715693483.724691698]: PassiveBehaviorControl active for [X] DIMENSION!
[ WARN] [1715693483.725001542]: diagnostic_updater: No HW_ID was set. This is probably a bug. Please report it. For devices that do not have a HW_ID, set this value to 'none'. This warning only occurs once all diagnostics are OK so it is okay to wait until the device is open before calling setHardwareID.
Loaded 'fts_adaptive_force_controller'
Started ['joint_state_controller'] successfully
Started ['odometry_controller'] successfully
Started ['fts_adaptive_force_controller'] successfully
[base/base_controller_spawner-3] process has finished cleanly
log file: /tmp/robotrainer/ros_logs/685ee0c2-11f2-11ef-ba04-e0d55e1939dd/base-base_controller_spawner-3*.log
[ WARN] [1715694021.479592037]: filterFTData: Waiting for Lock for 1ms not successful! Using old data! If this happens more often You might have a problem with timing of measurements!
```