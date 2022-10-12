# H-Robix

H-Robix is a project, consisted in designing, building and programming an hexapod in ROS, with dinamixel servomotors.



![photo_2022-06-16_17-32-17](https://user-images.githubusercontent.com/65245295/195049259-039b3ba3-06bd-4a9c-95d8-c9cb65a84974.png)


## ROS Repository Structure

```
Catkin_ws
├── src
│   ├── Hexapod (all launch files needed to run the hexapod control locally)
│       ├── launch 
│           ├── hexapod.launch
│           ├── hexapod_sim.launch
│           ├── rviz_hexapod.launch
│   ├── Hexapod_description (urdf, params and meshes to run Rviz, Gzebo and get joint info to perform kinematics)
│       ├── meshes 
│           ├── body.stl, coxa.stl, femur.stl, tibia.stl
│       ├── params 
│           ├── config.params
│       ├── urdf 
│           ├── hexapod.xacro
│   ├── Hexapod_control 
│   └── Examples: contains 'demo' crawler ROS packages that build upon some of the 'core' crawler ROS packages
│       ├── Demo Crawler ROS Package 1
├── devel
├── logs
└── build
```




## Video
https://user-images.githubusercontent.com/65245295/195037681-7bfc925a-f934-4565-9218-098b3da9bce3.mp4


