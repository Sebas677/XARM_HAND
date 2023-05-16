# XARM_HAND
Use of xarm_ros to move a xarm5 robotic arm using both hands to manipulate the x,y &amp; z axis

hand.zip:
  *Contains all the CV logic and xarm5 robotic arm movement
  
xarm_gazebo.zip:
  *Several files were modified in order to avoid manipulation conflicts:
    *Adapted xarm_example1_table.world to have multiple boxes, tables and coffee chair

In order to use this project, you should replace the whole xarm_gazebo pkg from xarm_ros.

The hand.zip  includes a launchfile wich run the whole simulation:
`````roslaunch hand robot_hand.launch




https://drive.google.com/file/d/12biMAaxAdLywPSM6WyolW0Z4LfllieCG/view?usp=sharing

