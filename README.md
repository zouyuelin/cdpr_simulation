# 初期准备
安装visp和log2plot 库，进入到对应的文件
```shell
mkdir build
cd build
cmake ..
make -j
sudo make install
```
visp库是视觉处理相关的库，也有矩阵运算的库函数使用
log2plot 库是用于显示和打印相关的信息，相比于cout更加直观易用

# 1.文件结构

## 1). cdpr
这是cdpr的基本功能包，需要根据自己的需求修改cube.yaml文件，然后生成sdf:
```python
python3 gen_cdpr.py cube.yaml
```
生成cube.sdf以后，需要显示轨迹，则需要添加libdraw_traj.so在其中，详情见5.

## 2). cdpr_controllers
这个功能包是主要的修改对象，最佳例子见其src文件夹下的cdprF.cpp，在读取或者生存每个cable的tension后，通过话题发布出去：
```c++
vpColVector tau(n);
//TODO: set tau's values
robot.sendTensions(tau);
```
整个控制系统及虚拟环境可以通过:
```shell
roslaunch cdpr_controllers cdpr_test.launch
```
启动，然后点击gazebo下方的小三角形开始进行仿真。

## 3). code_folder_cable_robot_Gazebo
原开源作者提高的代码，可以进行参考，也可以跳过。
## 4). fengji
里面是风机的相关模型，包括urdf描述文件，可以通过solidworks导出生成
## 5). Gazenp_trakectpru_display
该功能包是显示actuator的运动轨迹和desired posiiton的，生成的.so文件需要加到cdpr/sdf/cube.sdf中
```python
      <visual name="visualpd">
        <plugin name="draw_traj" filename="libdraw_traj.so"/>
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
        <transparency>1.0</transparency>
      </visual>
```
大概在cube.sdf294行添加，与frame 进行绑定，在platform的上方。
这里不需要再手动添加，直接运行gen_cdpr.py即可

另外需要显示规划的路径使，则需要修改cpp文件中的waypoint：
```c++
            this->waypoint = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
            this->waypoint->setMaterial("Gazebo/Red");
            this->waypoint->setVisibilityFlags(GZ_VISIBILITY_GUI);
            //way point 
            this->waypoint->AddPoint(ignition::math::Vector3d(-2, -4, 0), 
                                           ignition::math::Color(0, 0, 1, 1.0));
            this->waypoint->AddPoint(ignition::math::Vector3d(-2, -4.3, 1), 
                                           ignition::math::Color(0, 0, 1, 1.0));
            this->waypoint->AddPoint(ignition::math::Vector3d(-1, -3.5, 2.0), 
                                           ignition::math::Color(0, 0, 1, 1.0));
```
当然，对应的cdpr/controllers/src/cdprF.cpp 的120行添加对应的waypoint。
## 6). trajectory_generator
用于生成路径和对应tension的功能包，参考价值中等。

# 2.运行方法

```shell
roslaunch cdpr_controllers cdpr_test.launch
```

# 3.修改方法

## 1）修改点
位于cdpr_controllers/sdf/trajectory.yaml中，可以自行添加或删减

# 4.话题名
```/cable_command``` 发布话题可以输入力到platform\
```/cable_states```  cable的tensions\
```/pf_state```      platform 的位置情况，速度等