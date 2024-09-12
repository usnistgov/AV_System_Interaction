# NIST Autonomous Vehicle Control and Interaction Codebase

## Code Overview

    The speed_override_node is a ROS2 node that subscribes to the /stop_cmd topic and publishes to the /speed_override topic.

    The /stop_cmd topic is subscribed to by the AV_DM node, which uses the stop command to stop the AV.
    
    The /speed_override topic is published to by the speed_override_node, which overrides the speed of the AV. Making the AV stop when the stop command is received.

### Package name: av_dm_control

## Instructions:

    1. Clone Git:
        1.1: git clone git@github.com:usnistgov/AV_System_Interaction.git av_dm_control

    2. Change into the ros working directory

    3. Build Package
        3.1 colcon build --packages-select av_dm_control
    
    4. Running Control Node:
        4.1: source install/local_setup.bash
        4.2: ros2 run av_dm_control speed_control.py 

