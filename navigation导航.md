# navigation导航

## `move_base`包
​		提供了一个动作的实现（参见actionlib包），在给定一个目标后，将尝试通过移动基础来达到它.move_base节点将全局和局部规划器链接在一起以完成其全局导航任务。它支持任何遵循`nav_core`包中指定的`nav_core::BaseGlobalPlanner`接口的全局规划器，以及任何遵循`nav_core`包中指定的`nav_core::BaseLocalPlanner`接口的本地规划器。move_base节点还维护两个costmap，一个用于全局规划器，另一个用于本地规划器（请参阅costmap_2d包），用于完成导航任务。
![move_base框架](photo/move_base框架.png)

​		上面显示了move_base节点的高级视图及其与其他组件的交互。蓝色根据机器人平台而有所不同，灰色是可选的，适用于所有系统，并且白色节点是必需的，适用于所有系统。有关move_base节点的配置以及整个导航堆栈的更多信息，请参阅http://wiki.ros.org/navigation/Tutorials/RobotSetup

### move_base包的安装
#### 1.普通安装

在安装导航框架时，move_base功能包就在其中,命令安装：
```sh
$ sudo apt-get install ros-kinetic-navigation
```
#### 2.源码编译安装
navigation包中，包含move_base功能包，命令安装：
```sh
$ git clone https://github.com/ros-planning/navigation.git
```
编译过程需要导航堆栈，堆map_msg和move_base_msgs进行命令安装：
```sh
$ git clone https://github.com/ros-planning/navigation_msgs.git
```
#### 3.move_base功能包中的订阅、发布话题及服务
- 动作订阅：
  - move_base/goal：move_base的运动规划目标
  - move_base/cancel：取消特定目标强求
- 动作发布：
  - move_base/feedback：反馈信息，含有机器人底盘的坐标
  - move_base/status：发送到move_base的目标状态信息
  - move_base/result：此处move_base操作的结果为空
- 话题订阅：
  - move_base_simple/goal：为无需追踪目标执行状态的用户，提供一个非action接口
- 话题发布：
  - cmd_vel：输出到机器人底盘的速度命令
- 服务：
  - ~make_plan：允许用户从move_base获取给定目标的路径规划，但不会执行该路径规划
  - ~clear_unknown_space：允许用户直接清除机器人周围的未知空间。适合costmap停止很长时间后，在一个全新环境中重新启动时使用
  - ~clear_costmaps：允许用户命令move_base节点清除costmap中的障碍。这可能导致机器人撞上障碍物，请谨慎使用

#### 4.move_base功能包中的参数配置
`base_global_planner` (string, default: "navfn/NavfnROS" For 1.1+ series) 

- 设置move_base的全局路径规划器的插件名称，该插件必须继承自此插件nav_core包中指定的nav_core::BaseGlobalPlanner 接口。(1.0 series default: "NavfnROS")

`base_local_planner` (string, default: "base_local_planner/TrajectoryPlannerROS" For 1.1+ series) 

- 设置move_base的局部路径规划器的插件名称，该插件必须继承自此插件nav_core包中指定的nav_core::BaseGlobalPlanner 接口。(1.0 series default: "TrajectoryPlannerROS")

`recovery_behaviors` (list, default: [{name: conservative_reset, type: clear_costmap_recovery/ClearCostmapRecovery}, {name: rotate_recovery, type: rotate_recovery/RotateRecovery}, {name: aggressive_reset, type: clear_costmap_recovery/ClearCostmapRecovery}] For 1.1+ series) 

- 指定用于move_base的修复操作插件列表，当move_base不能找到有效的计划的时候，将按照这里指定的顺序执行这些操作。每个操作执行完成后，move_base都会尝试生成有效的计划。如果该计划生成成功，move_base会继续正常运行。否则，下一个修复操作启动执行。

`controller_frequency` (double, default: 20.0) 

- 以Hz为单位的速率运行控制循环并向基座发送速度命令。太大会占用CPU 这里我们设置为3， 好点的处理器可以设置稍高。

`planner_patience` (double, default: 5.0) 

- 在空间清理操作执行前，路径规划器等待多长时间（秒）用来找出一个有效规划。

`controller_patience` (double, default: 15.0) 

- 在空间清理操作执行前，控制器会等待多长时间（秒）用来找出一个有效控制。

`conservative_reset_dist` (double, default: 3.0) 

- 当在地图中清理出空间时候，距离机器人几米远的障碍将会从costmap清除。注意：该参数仅用于move_base使用了默认参数的情况。

`recovery_behavior_enabled` (bool, default: true)

- 是否启用move_base修复机制来清理出空间。

`clearing_rotation_allowed` (bool, default: true) 

- 决定做清理空间操作时候，机器人是否会采用原地旋转。注意：该参数仅用于move_base使用了默认参数的情况，这意味着用户尚未将recovery_behaviors参数设置为任何自定义形式。

`shutdown_costmaps` (bool, default: false)

- 当move_base进入inactive状态时候，决定是否停用节点的costmap。

`oscillation_timeout` (double, default: 0.0) 

执行修复操作之前，允许的震荡时间是几秒。 值0意味着永不超时。

`oscillation_distance` (double, default: 0.5) 

- 机器人需要移动多少距离才算作没有震荡。 移动完毕后重置定时器计入参数~oscillation_timeout。

`planner_frequency` (double, default: 0.0) 

- 全局路径规划器loop速率。如果设置这个为0.0, 当收到新目标点或者局部路径规划器报告路径不通时候全局路径规划器才启动。

`max_planning_retries` (int32_t, default: -1) 

- 恢复操作之前尝试规划的次数，-1代表无上限的不断尝试。

参考https://www.ncnynl.com/archives/201708/1898.html



## 1.map_sever

​		提供map_server ROS节点，该节点将地图数据作为ROS服务提供。它还提供了map_saver命令行实用程序，允许将动态生成的映射保存到文件中。

- 地图形式
 - 图片格式：该图像以相应像素的颜色描述世界上每个单元的占用状态。在标准配置中，较白的像素是空闲的，较黑的像素是占用的，中间的像素是未知的。通过SDL_Image读取图像数据，大多数图像格式都支持.
 - YAML格式：

```
image: testmap.png
resolution: 0.1
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```
- `image`:包含占用数据的图像文件的路径;可以是绝对的，或者是相对于YAML文件的位置.
- `resolution`：地图的分辨率，单位是米/像素
- `origin`：在地图中左下角像素的二维位姿为(x,y,yaw)，yaw表示逆时针旋转，yaw=0为无旋转，
- `occupied_thresh`：占用概率大于此阈值的像素被认为已完全占用。
- `free_thresh`：占用率小于此阈值的像素被认为是完全空闲的。
- `negate`：是否白/黑、自由/占用的语义应该颠倒(阈值的解释不受影响)

### map_saver

map_saver从SLAM映射服务保存地图到磁盘的映射。

`rosrun map_server map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>]`

map_saver检索地图数据并将其写入map.pgm和map.yaml中。使用-f选项为输出文件提供不同的基名称。--occ和--free选项接受0到100之间的值。

## 2.nav_core
这个包提供了导航特定机器人动作的通用接口。`BaseGlobalPlanner`、`BaseLocalPlanner`和`RecoveryBehavior`，这些接口可用于构建操作.nav_core包含导航堆栈的关键接口。所有希望在move_base节点中用作插件的计划器和恢复行为都必须遵守这些接口。

![nav_core](photo/nav_core.png)

- BaseGlobalPlanner
 - `global_planner`:一种快速，内插的全局规划器，可作为navfn的更灵活的替代品。(pluginlib名称：“global_planner/ GlobalPlanner”)
 - `navfn`:基于网格的全局规划器，使用导航功能计算机器人的路径。(pluginlib名称：“navfn/NavfnROS”)
 - `carrot_planner`:一个简单的全局规划器，它采用用户指定的目标点并尝试尽可能靠近它移动机器人，即使该目标点处于障碍物中。(pluginlib名称：“carrot_planner/CarrotPlanner”)
- BaseLocalPlanner
 - `base_local_planner`:提供动态窗口方法（DWA）和轨迹展示方法到本地控制的实现
 - `dwa_local_planner`:与base_local_planner的DWA相比，模块化的DWA实现具有更清晰、更容易理解的接口，以及更灵活的完整机器人y轴变量
 - `eband_local_planner`:在SE2流形上实现了弹性带法
 - `teb_local_planner `:实现了在线轨迹优化的时间弹性带法
- RecoveryBehavior
 - `clear_costmap_recovery`:将move_base使用的costmaps恢复到用户指定范围之外的静态映射的恢复行为
 - `rotate_recovery`:一种恢复行为，使机器人360度旋转以试图清除空间。

![ros_plugin](photo/ros_plugin.png)

​		在上图的这个例子中，假设你想使用一个形状画图模块，ROS的官方包中已经有了polygon_interface这个基类，它已经提供了标准的接口函数，并且有两个子类：矩形插件包rectangle_plugin package 和三角形插件包 triangle_plugin package。要想使用这两个子类你只要在相应文件中将它们注册为插件，告诉ROS我将使用这两个插件就行了。

​		在nav_core中仅仅只有4个头文件，正是这些头文件提供了多个模板：`nav_core::BaseGlobalPlanner`，`nav_core::BaseLocalPlanner`， `nav_core::RecoveryBehavior`，在官方的wiki文档里可以看到他们的相关介绍。如果联系到到我们自己要写global或者local planner的插件，所以我们要按照ROS提供的模板去实现。

​		可以将`navfn`包和`global_planner`包理解成一个并列关系，因为他们两个都是用来做全局规划的，两个包里面也都实现了A*,Dijkstra算法。关于两个包的关系，可以参考https://blog.csdn.net/heyijia0327/article/details/45030929

### 如何在ROS中写一个全局规划器作为插件

http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS

https://www.ncnynl.com/archives/201702/1331.html

## 3.参数配置
### move_base通用参数配置

参考http://wiki.ros.org/move_base?distro=kinetic
 - `controller_frequency`(double, default: 20.0):向底盘控制移动话题cmd_vel发送命令的频率.此值太大会占用CPU 这里我们设置为3， 好点的处理器可以设置稍高.
 - `controller_patience`(double, default: 15.0):在空间清理操作执行前,控制器花多长时间等有效控制下发.
 - `planner_frequency`(double, default: 0.0):全局规划操作的执行频率.如果设置为0.0,则全局规划器仅在接收到新的目标点或者局部规划器报告路径堵塞时才会重新执行规划操作.
 - `planner_patience`(double, default: 5.0):在空间清理操作执行前,规划器将在几秒钟内等待尝试找到有效规划。
 - `oscillation_timeout`(double, default: 0.0):执行修复机制前,允许振荡的时长.
 - `oscillation_distance`(double, default: 0.5):来回运动在多大距离以上不会被认为是振荡.
 - `base_local_planner`(string, default: "base_local_planner/TrajectoryPlannerROS"）:指定用于move_base的局部规划器插件名称.
 - `base_global_planner`(string, default: "navfn/NavfnROS"）:指定用于move_base的全局规划器插件名称.
 - `recovery_behaviors`(list, default: [{name: conservative_reset, type: clear_costmap_recovery/ClearCostmapRecovery}, {name: rotate_recovery, type: rotate_recovery/RotateRecovery}, {name: aggressive_reset, type: clear_costmap_recovery/ClearCostmapRecovery}]）：与move_base一起使用的恢复行为插件列表，当move_base无法按照指定的顺序查找有效计划时，将运行这些行为。每个行为完成后，move_base将尝试制定规划。如果规划成功，move_base将继续正常运行。否则，将执行列表中的下一个恢复行为。
 - `conservative_reset_dist`(double, default: 3.0)：当试图清除地图中的空间时，机器人与障碍物之间的距离(以米为单位)将从costmap中清除。注意，此参数仅在move_base使用默认恢复行为时使用。
 - `recovery_behavior_enabled`(bool, default: true)： 是否允许move_base恢复行为尝试清除空间。
 - `clearing_rotation_allowed`(bool, default: true)：确定机器人在试图清空空间时是否尝试原地旋转。
 - `max_planning_retries`(int32_t, default: -1)：在执行恢复行为之前，允许计划重试的次数。值-1.0表示无限次重试。

### 全局规划器参数配置
参考http://wiki.ros.org/global_planner?distro=kinetic

- `allow_unknown`(bool, default: true):指定是否允许规划器创建跨越未知空间的路径。只设计该参数为true还不行,还要在costmap_commons_params.yaml中设置track_unknown_space参数也为true才行。
- `default_tolerance`(double, default: 0.0)：对规划器的目标点的容忍。规划器将尝试创建一个尽可能接近指定目标的计划，但不会超过default_tolerance。
- `visualize_potential`(bool, default: false)：是否显示从PointCloud2计算得到的潜在区域.
- `use_dijkstra`(bool, default: true)：设置为true,将使用dijkstra算法,否则使用A*算法.
- `use_quadratic`(bool, default: true)：设置为true,将使用二次函数近似函数,否则使用更加简单的计算方式,这样节省硬件计算资源.
- `use_grid_path`(bool, default: false)：如果设置为true,则会规划一条沿着网格边界的路径,偏向于直线穿越网格,否则将使用梯度下降算法,路径更为光滑点.
- `old_navfn_behavior`(bool, default: false)：若在某些情况下,想让global_planner完全复制navfn的功能,那就设置为true,但是需要注意navfn是非常旧的ROS系统中使用的,现在已经都用global_planner代替navfn了,所以不建议设置为true.
- `lethal_cost`(int, default: 253)：致命代价值,默认是设置为253,可以动态来配置该参数.
- `neutral_cost`(int, default: 50)：中等代价值,默认设置是50,可以动态配置该参数.
- `cost_factor`(double, default: 3.)：代价地图与每个代价值相乘的因子.
- `publish_potential`(bool, default: True)：是否发布costmap的势函数.
- `orientation_mode`(int, default: 0)：如何设置每个点的方向（None = 0,Forward = 1,Interpolate = 2,ForwardThenInterpolate = 3,Backward = 4,Leftward = 5,Rightward = 6）（可动态重新配置）
- `orientation_window_size`(int, default: 1)：根据orientation_mode指定的位置导数来得到使用窗口的方向.默认值1,可以动态重新配置.

#### global_planner部分代码解读

参考http://wiki.ros.org/global_planner?distro=kinetic

在global_planner包中的planner_core.cpp文件中.

- `void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id){}`
 - `old_navfn_behavior`：默认为false，如果由于某种原因，希望global_planner准确地反映navfn的行为，将其设置为true(并对其他布尔参数使用默认值)
 - `use_quadratic`：如果为真，使用potential的二次逼近。否则，使用更简单的计算。
 - `use_dijkstra`：如果为真，请使用dijkstra算法。否则,用A*算法。
 - `use_grid_path`：如果为真，创建一个遵循网格边界的路径（网格路径）。否则，使用梯度下降法（梯度路径）。
 - `allow_unknown`：指定是否允许规划器创建跨越未知空间的路径。默认为true，注意:如果你正在使用带有体素或障碍层的分层costmap_2d, 您还必须将该层的track_unknown_space参数设置为“true”，否则它会将所有未知空间转换为可用空间。


参考https://blog.csdn.net/shixiaolu63/article/details/84544499

### 局部规划器参数配置

参考http://wiki.ros.org/base_local_planner?distro=kinetic
- Robot Configuration Parameters
  - `acc_lim_x`(double, default:2.5):机器人的x方向加速度限制，单位是米/秒^2
  - `acc_lim_y`(double, default: 2.5)：机器人的y方向加速度限制，单位是米/秒^2
  - `acc_lim_theta`(double, default: 3.2):机器人的旋转加速度限制，单位是弧度/秒^2
  - `max_vel_x`(double, default: 0.5)：允许的前进速度的最大值，单位是米/秒
  - `min_vel_x`(double, default: 0.1)：允许的前进速度最小值，单位是米/秒
  - `max_vel_theta`(double, default: 1.0)：允许的旋转速度最大值，单位是弧度/秒
  - `min_vel_theta`(double, default: -1.0)：允许的旋转速度最小值，单位是弧度/秒
  - `min_in_place_vel_theta`(double, default: 0.4)：在以弧度/秒为单位进行原地旋转时，基座允许的最小旋转速度
  - `escape_vel`(double, default: -0.1)：躲避时行驶的速度，单位为米/秒。请注意，它必须是负的，以便机器人实际逆转。正速度会导致机器人在试图躲避时向前移动。
  - `holonomic_robot`(bool, default: true)：确定为完整机器人或非完整机器人生成速度命令。对于完整的机器人，可以向基座发出扫射速度命令。对于非完整机器人，不会发出扫射速度命令。
  - `y_vels`(list, default: [-0.3, -0.1, 0.1, 0.3])：完整机器人考虑的扫射速度，单位为米/秒
- Goal Tolerance Parameters
  - `yaw_goal_tolerance`(double, default: 0.05)：控制器在实现其目标时的偏航/旋转的弧度公差
  - `xy_goal_tolerance`(double, default: 0.10)：控制器在达到目标时，在x和y距离上的公差以米为单位
  - `latch_xy_goal_tolerance`(bool, default: false)：如果目标公差被锁定，如果机器人到达目标xy位置，它将简单地原地旋转，即使它在这样做的时候超出了目标公差。
- Forward Simulation Parameters
  - `sim_time`(double, default: 1.0):向前模拟轨迹的时间量,以秒为单位
  - `sim_granularity`(double, default: 0.025):在给定轨迹上的点之间采用的步长（以米为单位）
  - `angular_sim_granularity`(double, default: ~<name>/sim_granularity):在给定轨迹上,采用角度样本之间的步长。以弧度表示
  - `vx_samples`(integer, default: 3)：探索x速度空间时要使用的样本数
  - `vtheta_samples`(integer, default: 20)：探索θ速度空间时要使用的样本数
  - `controller_frequency`(double, default: 20.0)：以Hz为单位调用此控制器的频率。
- Trajectory Scoring Parameters
  - `meter_scoring`(bool, default: false)：无论gdist_scale和pdist_scale参数，应该假定goal_distance和path_distance被表示，米或单元格。默认情况下假设单元格。
  - `pdist_scale`(double, default: 0.6)：控制器应该保持接近给定路径的权重，最大可能值为5.0
  - `gdist_scale`(double, default: 0.8)：控制器应该尝试达到其本地目标的权重，还有控制速度，最大可能值为5.0
  - `occdist_scale`(double, default: 0.01)：控制器应该试图避开障碍物的权重
  - `heading_lookahead(double, default: 0.325)：是否根据机器人前往路径或距离路径的距离进行评分
  - `heading_scoring`(bool, default: false)：是否根据机器人前往路径或距离路径的距离进行评分
  - `heading_scoring_timestep`(double, default: 0.8)：使用航向评分时，沿着模拟轨迹在几秒钟内向前看的距离
  - `dwa`(bool, default: true)：是否使用动态窗口方法（DWA）或是否使用轨迹展开
  - `publish_cost_grid_pc`(bool, default: false)：是否发布规划器在规划时将使用的成本网格。如果为true，则〜<name>/cost_cloud主题上将提供sensor_msgs/PointCloud2。每个点云代表成本网格，并且每个单独的评分函数组件都有一个字段以及每个单元的总成本，并考虑评分参数。
  - `global_frame_id`(string, default: odom)：要为cost_cloud设置的框架。应设置为与本地costmap的全局框架相同的框架。

用于记录每个轨迹的代价函数如下所示
```
cost = 
  pdist_scale * (distance to path from the endpoint of the trajectory in map cells or meters depending on the meter_scoring parameter) 
  + gdist_scale * (distance to local goal from the endpoint of the trajectory in map cells or meters depending on the meter_scoring parameter) 
  + occdist_scale * (maximum obstacle cost along the trajectory in obstacle cost (0-254))
```

- Oscillation Prevention Parameters
  - `oscillation_reset_dist`(double, default: 0.05)：机器人运动多远距离才会重置振荡标记.
- Global Plan Parameters
  - `prune_plan`(bool, default: true)：机器人前进是是否清除身后1m外的轨迹.

#### local_planner参数调试说明

因为是差分驱动小车，y轴方向的参数在此处忽略

- `min_in_place_vel_theta`：原地旋转速度，当该值比较小时原地转动速度慢，反之相反.
- `acc_lim_x`：前进方向的加速度，该值不能设置太小，否则前进方向速度变化很慢，可能发生不按照生成前向的路径走，而原地转动.
- `max_vel_x`:前进方向最大速度
- `min_vel_x`:前进方向最小速度
- `max_vel_theta`:前进方向最小速度

规划器的规划参数：

- `~<name>/path_distance_bias` (double, default: 32.0)：刻画局部路径和全局路径的贴合程度，该权重参数越大说明越贴合，越小说明不用那么贴合。
- `~<name>/goal_distance_bias` (double, default: 24.0)：达到局部目标点的权重参数，也用来控制速度。权重如果设置为0表示要求完全到达目标点，这会导致机器人走动缓慢和震荡的现象，因为要求达到目标点的精度太高，所对机器人的控制比较苛刻.
- `pdist_scale`和`gdist_scale`对于路径规划的影响，推荐设置 `pdist_scale`为0.8，`gdist_scale`为0.4，

参考 http://www.cnblogs.com/cv-pr/p/5800270.html和
https://blog.csdn.net/lingchen2348/article/details/79831150

## 4.costmap_2d包

- 订阅话题
  - `footprint(geometry_msgs/Polygon)`：规范机器人的footprint,这将代替以前footprint的参数规范.
- 发布话题
  - `costmap(nav_msgs/OccupancyGrid)`：代价地图的值
  - `costmap_updates(map_msgs/OccupancyGridUpdate)`：代价地图更新区域的值
  - `voxel_grid(costmap_2d/VoxelGrid)`：当底层占用网格使用像素并且用户请求发布像素网格时，可选地广播

参数配置
- `Plugins`：插件规范的顺序，每层一个。每个规范都是一个带有名称和类型字段的字典。该名称用于定义插件的参数命名空间。配置看链接http://wiki.ros.org/costmap_2d/Tutorials/Configuring%20Layered%20Costmaps
- Coordinate frame and tf parameters
  - `global_frame`(string, default: "/map")：Costmap操作的全局框架。
  - `robot_base_frame`(string, default: "base_link")：机器人base link的框架名称。
  - `transform_tolerance`(double, default: 0.2)：指定转换（tf）数据中可容忍的延迟（秒）。这个参数可以保护在tf树中丢失一个link的同时仍然允许用户在系统中舒适地存在一定的延迟。如果全局帧指定的坐标帧与robot基本帧参数之间的tf转换是早于ros::time::now（）的transform_tolerance时间，则导航堆栈将停止robot。
- Rate parameters
  - `update_frequency`(double, default: 5.0)：要更新地图的频率，单位Hz
  - `publish_frequency`(double, default: 0.0)：地图发布显示信息的频率，单位Hz
- Map management parameters
  - `rolling_window`(bool, default: false):是否使用滚动窗口版本的成本地图。如果static_map参数设置为true，则必须将此参数设置为false。
  - `always_send_full_costmap`(bool, default: false):如果为true，则每次更新都会将完整的Costmap发布到“~<name>/costmap”。如果为false，则仅在“~<name>/costmap_updates”主题上发布已更改的Costmap部分。
- 以下参数可以被一些图层覆盖，即静态地图图层。
  - `width`(int, default: 10):地图的宽度（米）
  - `height`(int, default: 10):地图的高度（米）
  - `resolution`(double, default: 0.05):地图的分辨率（米/格）
  - `origin_x`(double, default: 0.0):全局框架中地图的x原点（以米为单位）
  - `origin_y`(double, default: 0.0):全局框架中地图的y原点（以米为单位）

### obstacle_layer

http://wiki.ros.org/costmap_2d/hydro/obstacles#Map_management_parameters

### static_layer

https://blog.csdn.net/x_r_su/article/details/53420130

### inflation_layer

https://blog.csdn.net/x_r_su/article/details/53420209


## 5.ROS AMCL参数配置

[amcl配置文件说明](https://www.cnblogs.com/dyan1024/p/7825988.html)

[里程计相关参数对amcl的影响](https://blog.csdn.net/qq_29796781/article/details/80001355)

[协方差对amcl的影响](https://blog.csdn.net/weixin_40114832/article/details/78346953)

## 6.rviz
参考http://wiki.ros.org/rviz/DisplayTypes/Odometry和https://www.cnblogs.com/hiram-zhang/p/10398534.html

`odometry`配置

- shape:箭头的形状
- topic:订阅的话题
- position tolerance：位置公差（以米为单位，里程表必须要改变的直线距离以产生新的箭头），默认0.1
- angle tolerance:角度公差（里程计必须要改变的角距离，以产生新的箭头）默认0.1
- keep:在新箭头开始导致旧箭头消失之前要保留的箭头数量。默认100
- covariance:描述里程计误差的协方差矩阵,如果启用Covariance来描述Odometry将导致显示效果很难看，所以建议去掉。

## reference

[1] http://slidegur.com/doc/221751/lesson-7
[2] https://blog.csdn.net/u013158492/article/category/2308493
[3] [ROS基础教程--CostMap_2D包](<https://blog.csdn.net/jinking01/article/details/79455962>)
[4] [costmap_2d包源码学习](<https://blog.csdn.net/qq_41986495/article/details/86489567>)
[5] [Costmap: inflation 层](https://congleetea.github.io/blog/2018/09/10/costmap-inflation-layer/)
[6] [ObstacleLayer](https://www.cnblogs.com/flyinggod/p/9083484.html)
[7] [costmap_2D](https://blog.csdn.net/x_r_su/article/details/53420310)
[8] [costmap_2d框架函数](https://blog.csdn.net/jinking01/article/details/79447495)
<https://www.processon.com/view/link/5cef94c5e4b0123c616771ff>
[9] [MoveBase源码阅读笔记](https://blog.csdn.net/flyinsilence/article/details/82853766)
