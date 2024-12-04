
# Dual ur5e joystick teleoperation and Data Collection Startup Sequence
**Each command should be run in a separate terminal. Please start in the following order:**

1. Navigate to the folder:
   ```bash
   cd /home/username/demo_ws
   - src
      - Bimanual_Universal_Robots_ur5e_ROS_Driver
        https://github.com/Denghaoyuan123/Bimanual_Universal_Robots_ur5e_ROS_Driver

      - realsense-ros

      - This repo 
        https://github.com/Denghaoyuan123/Bimanual_ur5e_joystick_control
   ```

2. Open Terminator and start in the following sequence:

   - **Terminal 1**: Start communication for Arm 1
     ```bash
     roslaunch ur_robot_driver ur5e_bringup1.launch ns:=robot1 robot_ip:=xxx reverse_port:=50001 script_sender_port:=50002 trajectory_port:=50003 script_command_port:=50004
     ```

   - **Terminal 2**: Request service for Arm 1
     ```bash
     rosservice call /robot1/controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
     stop_controllers: ['scaled_pos_joint_traj_controller']
     strictness: 2
     start_asap: false
     timeout: 0.0"
     ```

   - **Terminal 3**: Start communication for Arm 2
     ```bash
     roslaunch ur_robot_driver ur5e_bringup1.launch ns:=robot2 robot_ip:=xxx reverse_port:=50011 script_sender_port:=50012 trajectory_port:=50013 script_command_port:=50014
     ```

   - **Terminal 4**: Request service for Arm 2
     ```bash
     rosservice call /robot2/controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
     stop_controllers: ['scaled_pos_joint_traj_controller']
     strictness: 2
     start_asap: false
     timeout: 0.0"
     ```

   - **Terminal 5**: Start the cameras
     ```bash
     roslaunch realsense2_camera rs_aligned_depth1.launch
     roslaunch realsense2_camera rs_aligned_depth2.launch
     ```

   - **Terminal 6**: Start remote control (need to prepare two joysticks)
     ```bash
     roslaunch ur5e_joystick_control ur5e_joystick_control.launch
     ```

   - **Terminal 7**: For each trajectory, set a new path: demo_01, demo_02, demo_03
     ```bash
     rosrun ur5e_joystick_control demo_database_dualarm.py --result_path /path/to/your/demo/demo_20
     ```
