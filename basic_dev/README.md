# RMUA挑战赛

## pxx

[microsoft/AirSim: Open source simulator for autonomous vehicles built on Unreal Engine / Unity, from Microsoft AI & Research](https://github.com/microsoft/AirSim)

[代码结构 - AirSim](https://microsoft.github.io/AirSim/code_structure/)

> 问题：
>
> [How to convert rotor velocity to PWM and constant units · Issue #2592 · microsoft/AirSim](https://github.com/microsoft/AirSim/issues/2592)
>
> 无法利用升力系数和反扭力系数，因此无法使用se3、mpc之类的控制器，只能考虑传统pid控制器：
>
> 1. 需要已知空气密度和螺旋桨直径，但是并未提供
> 2. airsim可以通过python的相关api查询以上参数，但是DJI貌似关闭了airsim相关接口
>
> 解决方案：**考虑直接移植px4控制器**

根据`setting.json`，仿真器服务自带的飞控使用的是`airsim`自带的[SimpleFlight](https://github.com/microsoft/AirSim/tree/main/AirLib/include/vehicles/multirotor/firmwares/simple_flight)飞控

* [ ] 旋翼无人机控制器（下一步考虑移植PX4控制器）

  - run（未测试，pwm貌似并不对应rotors转速，不能使用se3控制器）

    ```bash
    cd WORKSPACE_NAME/src
    git clone git@github.com:ethz-asl/mav_comm.git
    
    cd WORKSPACE_NAME
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes 
    
    source ./devel/setup.sh
    roslaunch rotors_control rmua.launch
    ```
    
  
* [ ] 发布消息控制无人机（键盘操控，以方便后面测试）

  ```bash
  rostopic pub /airsim_node/drone_1/vel_cmd_body_frame airsim_ros/VelCmd "twist:
    linear:
      x: 0
      y: 0
      z: 0
    angular:
      x: 0
      y: 0
      z: 0"
  ```

* [ ] 相机内外参标定，考虑使用VINS_Fusion
* [ ] 

