# 软件配置
Ubuntu20.04（用的WSL2），ros2-foxy，Carla-0.9.13
这些配置都是安装Carla-ros-bridge时候官方要求的配置，我这里是推荐大家直接使用这一套的，不然会出现各种各样的问题，很浪费时间
# 算法实现
控制算法：纵向pid，横向lqr

规划算法：参考线平滑，路径dp+qp，速度dp+qp
# 使用方法  
我写的源代码在my_planning_and_control这个文件夹里，如果想对算法做出修改，也是改这个文件夹。    
运行案列我是借助的carla_ad_demo(这是carla-ros-bridge里自带的案例)，在它的场景里运行自己规控算法，并没有自己写场景。  
运行指令为    
colcon build  
. install/setup.bash  
ros2 launch carla_ad_demo carla_ad_demo_launch.py  
