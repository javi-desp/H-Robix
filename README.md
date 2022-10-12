# H-Robix

H-Robix is a project, consisted in designing, building and programming an hexapod in ROS, with dinamixel servomotors.



![photo_2022-06-16_17-32-17](https://user-images.githubusercontent.com/65245295/195049259-039b3ba3-06bd-4a9c-95d8-c9cb65a84974.png)


## Software & Hardware scheme


## ROS Repository Structure

asasasasasasasa

```
Catkin_ws
├── src
│   ├── h_robix_control
│   ├── h_robix_description
│   ├── h_robix_gazebo
│   ├── h_robix_movement
│   ├── h_robix_sh_codes
│   ├── rqt_virtual_joystick
├── devel
├── logs
└── build
```

###### **h_robix_control**


aasasasasasa

```
h_robix_control
    ├── launch
        ├── hexapod_motor_driver.launch
        ├── hexapod_teleoperation.launch
    ├── msg and srv files 
        ├── hexapod.launch
    ├── src
        ├── buttons_handler.py
        ├── conf_motors.py
        ├── dinamixel_motor_controllers.cpp
        ├── pub_crr_motors_data.py
```

###### **h_robix_description**

```

 h_robix_description (urdf, params and meshes to run Rviz, Gzebo and get joint info to perform kinematics)
    ├── launch 
        ├── rviz_visualizer.launch
    ├── meshes 
        ├── body.stl, coxa.stl, femur.stl, tibia.stl
    ├── urdf 
        ├── h_robix.urdf.xacro

```

###### **h_robix_gazebo**

crr not available, joint controllers not well defined


###### **h_robix_movement**

```

 h_robix_description (urdf, params and meshes to run Rviz, Gzebo and get joint info to perform kinematics)
    ├── launch 
        ├── rviz_visualizer.launch
    ├── meshes 
        ├── body.stl, coxa.stl, femur.stl, tibia.stl
    ├── urdf 
        ├── h_robix.urdf.xacro

```

###### **h_robix_sh_codes**

```
h_robix_sh_codes
    ├── hexapod_network.sh
```

###### **rqt_virtual_joystick**



## Video demonstration tripod gait  
https://user-images.githubusercontent.com/65245295/195037681-7bfc925a-f934-4565-9218-098b3da9bce3.mp4


