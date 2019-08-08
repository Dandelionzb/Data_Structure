# DWA分析

## 1.体系结构
​	（1）主要成员

  - base_local_planner::LocalPlannerUtil planner_util_;  用来存储运动控制参数以及costmap2d、tf等，会被传入dp__
  - _costmap_2d::Costmap2DROS* costmap_ros_; 规划器所需代价地图
  - base_local_planner::OdometryHelperRos odom_helper_; 用来辅助获取odom信息，会被传入dp__
  - _boost::shared_ptr<DWAPlanner> dp_; dwa运动控制类
  - base_local_planner::LatchedStopRotateController latchedStopRotateController_;  到达目标点后的停止旋转运动控制类

### 2.DWAPlanner::findBestPath()

```shell
@`brief` Given the current position and velocity of the robot, find the best trajectory to exectue 给出当前位置和机器人的速度，找到最优的轨迹去执行
@param `global_pose` The current position of the robot 
@param `global_vel` The current velocity of the robot 
@param `drive_velocities` The velocities to send to the robot base
@`return` The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute. 得分最高的轨迹。成本大于等于0意味着执行轨迹是合法的
```
该函数获取最优局部路径对应的速度指令，先利用`SimpleTrajectoryGenerator::initialise()`初始化轨迹产生器，即产生速度空间.然后，利用`SimpleScoredSamplingPlanner::findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored = 0)`查找最优的局部路径。

### 3.SimpleScoredSamplingPlanner::findBestTrajectory()

```shell
  /**
   * Calls generator until generator has no more samples or max_samples is reached.
   * For each generated traj, calls critics in turn. If any critic returns negative
   * value, that value is assumed as costs, else the costs are the sum of all critics
   * result. Returns true and sets the traj parameter to the first trajectory with
   * minimal non-negative costs if sampling yields trajectories with non-negative costs,
   * else returns false.
   * 对于每一个生成的traj，依次称之为critics.如果任何critic返回负值，则该值被假定为成本，否则成本是所有critic结果的总和。
   * 如果采样生成具有非负成本的轨迹，返回true并将traj参数设置为具有最小非负成本的第一个轨迹，否则返回false。
   * @param `traj` The container to write the result to 待将结果写入的容器
   * @param `all_explored` pass NULL or a container to collect all trajectories for debugging (has a penalty) 传递空值或一个容器以收集所有用于调试的轨迹（有一个惩罚）
   */
```
先调用每个打分项的prepare函数进行准备，而后调用SimpleScoredSamplingPlanner里面存储的TrajectorySampleGenerator轨迹产生器列表，分别进行轨迹产生和打分，并根据打分选出最优轨迹。
对于每个轨迹的打分，调用SimpleScoredSamplingPlanner::scoreTrajectory执行，在该函数中，遍历各个打分项进行打分并进行求和，如果某个打分项打分小于0，则将该负值作为路径打分项进行立刻返回。在对每个打分项打分之前，先获取对应的scale参数，如果scale为0证明该打分项无效，则跳过。如果获取到正常的打分项打分后（利用打分项的scoreTrajectory函数），将打分项乘以对应的scale作为最终的打分项打分

## 4.各个代价函数的打分项

1. 打分对象
   - base_local_planner::OscillationCostFunction **oscillation_costs_**（摆动打分）
   
   - base_local_planner::ObstacleCostFunction **obstacle_costs_**（避障打分）
   
   - base_local_planner::MapGridCostFunction **path_costs_**（路径跟随打分）
   
   - base_local_planner::MapGridCostFunction **goal_costs_**（指向目标打分）
   
   - base_local_planner::MapGridCostFunction **goal_front_costs_**（前向点指向目标打分）
   
   - base_local_planner::MapGridCostFunction **alignment_costs_**（对齐打分）
   
     摆动打分和避障打分项是独立的类型，后四个打分项是同一类型
   
     打分计算过程中出现的负的值可以认为是错误代码（用于指示具体的出错原因），而不是得分
   
     如果轨迹为空（在产生轨迹时出错，例如不满足最大最小速度），则各个打分项对应的得分都为0。

2.打分对象初始化及更新

   - oscillation_costs_

     - 在DWAPlanner的构造函数中，利用oscillation_costs_.resetOscillationFlags()重置摆动标志位(侧移、旋转、前进方向的相关参数都设为false)，并将oscillation_costs传入打分项列表`critics`(也会加入下面介绍的其它打分项),并将打分项列表`critics`和采样规划器列表`generator_list`传入打分采样规划器base_local_planner::SimpleScoredSamplingPlanner中
	- 在DWAPlannerROS的动态配置回调函数中，会将动态配置参数再传入DWAPlanner::reconfigure函数，在该函数中利用oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle)初始化该打分项。
     - 在DWAPlanner::findBestPath函数中，在该方法中会先采样计算出最优路径，然后利用`oscillation_costs_.updateOscillationFlags()`更新摆动打分项的标志位。在打分项更新标志位的函数中，如果判断最优路径的代价也小于0（即无效），则不更新，否则，则根据最优路径和最小平移速度限制设置打分标志`setOscillationFlags()`。如果在某些方向上有了限制，则检查是否能够复位标志位`resetOscillationFlagsIfPossible()`。
   - obstacle_costs_
	- 在DWAPlanner的构造函数中，利用local costmap初始化避障打分项，然后通过sum_scores参数（默认为false）设置避障打分项的打分汇总方式。
	- 利用obstacle_costs_.setScale函数设置比例参数为occdist_scale,并在obstacle_costs中记录为 scale_。然后利用obstacle_costs_.setParams函数设置max_trans_vel, max_scaling_factor, scaling_speed(这些参数可以引起避障打分项中的footprint变化，从而根据不同的速度膨胀不同的大小）。
	- 在每一次通过DWAPlanner的findBestPath函数计算最优局部路径时，都会利用obstacle_costs_.setFootprint为obstacle_costs_设置一次footprint参数（带padding），会被obstacle_costs_的footprintr_spec_（std::vector<geometry_msgs::Point>类型）参数记录。
```shell
`path_costs_`,`goal_costs`,`goal_front_costs_`, `alignment_costs_`四个打分项都是MapGridCostFunction类型，因此打分思路都一致，只是通过设置不同参数即可实现不同的打分方式。

MapGridCostFunction类的构造函数中，`path_costs`和`alignment_costs_`都是参考整个路径信息，因此`is_local_goal_function` 参数为false，同理`goal_costs_` 和`goal_front_costs_`对应的该参数为true。

另外，`goal_front_costs_`和`alignment_costs_`两个打分项都是参考前向打分点，因此还要利用MapGridCostFunction::setXShift函数记录了`forward_point_distance_`参数。其他两个打分项`path_costs`和`goal_costs`对应的该参数为0。

`goal_fronts_cost` 和`alignment_costs`两个打分项还利用MapGridCostFunction::setStopOnFailure函数设置了stop_on_failure参数为false。
```

   - path_costs_

     - path_costs_的初始化，在DWAPlanner的构造函数中，利用local costmap初始化路径打分项

     - path_costs_.setScale函数设置比例参数为`pdist_scale`，pdist_scale_ = resolution * config.path_distance_bias;
     - 在DWAPlanner::updatePlanAndLocalCosts函数（每个控制周期都会将映射到local costmap中的路径信息在这里更写到相关的打分项或者其他部件中）中，会利用path_costs_.setTargetPoses()将全局路径在局部地图中的映射传入到path_costs中，作为局部的打分参考路径，MapGridCostFunction::setTargetPoses函数将路径信息存储为target_poses_。

   - goal_costs_

     - goal_costs_的初始化，在DWAPlanner的构造函数中，利用local costmap初始化该打分项，同时也设置了`is_local_goal_function`标志位true。
     - goal_costs_.setScale函数设置比例参数为`gdist_scale`，gdist_scale_ = resolution * config.goal_distance_bias;

   - goal_front_costs_

     - goal_front_costs_的初始化，在DWAPlanner的构造函数中，利用local costmap初始化该打分项，同时也设置了is_local_goal_function标志位为true。另外，设置了stop_on_failure标志位为false。
     - goal_front_costs_.setScale函数设置比例参数为`gdist_scale`，gdist_scale_ = resolution * config.goal_distance_bias; 而后利用goal_front_costs_.setXShift函数将forward_point_distance_设为该打分项的xshift_参数（即打分点相对于机器人原点的偏移）。
     - 在DWAPlanner::updatePlanAndLocalCosts函数（每个控制周期都会将映射到local costmap中的路径信息在这里更写到相关的打分项或者其他部件中）中，为了给前向打分点确定它的“goal”，采用的方法是先计算当前位置和局部路径末端点之间的连线朝向，然后在局部路径末端点的基础上沿该方向延伸forward_point_distance_距离，并将这个”goal“替换掉局部路径末端点。然后利用goal_front_costs_.setTargetPoses函数将全局路径在局部地图中的映射传入到goal_front_costs_中，作为局部的打分参考路径，MapGridCostFunction::setTargetPoses函数将路径信息存储为target_poses_。

   - alignment_costs_

     - alignment_costs_的初始化，在DWAPlanner的构造函数中，利用local costmap初始化该打分项。同时设置stop_on_failure标志位为false。
     - alignment_costs_.setScale函数设置比例参数为`pdist_scale_`, pdist_scale_ = resolution * config.path_distance_bias; 利用alignment_costs_.setXShift函数将forward_point_distance_设为该打分项的xshift_参数（即打分点相对于机器人原点的偏移）。
     - 在DWAPlanner::updatePlanAndLocalCosts函数（每个控制周期都会将映射到local costmap中的路径信息在这里更写到相关的打分项或者其他部件中）中，为了让前向打分点合理的靠近路径，需要针对不同的情况考虑。如果当前位置和局部路径末端点之间的距离比forward_point_distance_大时，alignment_costs_.setTargetPoses(global_plan_)，即不做特殊处理，打分时参考前向打分点和局部地图路径的距离。否则，设置alignment_costs_的scale为0。
3. 代价打分

   所有的打分都是基于各个打分项的scoreTrajectory(Trajectory &traj)函数实现。
- oscillation_costs_

  主要利用了轨迹中的xv_, yv_, thetav_三个参数进行摆动判断。摆动打分较为简单，如果xxx_xxx_only设置为了true，但是对应的规划路径速度方向相反，则返回代价值-5，否则返回0。

- obstacle_costs_

  - 对轨迹上的每一点计算出对应的footprint（利用了x, y ,theta, footprint四个信息计算），计算footprintCost，最后汇总得分。如果sum_scores_参数为true，则加和所有得分，否则只取最后一个点的得分（覆盖计算的方式）。但是，如果某个点的footprintCost为负，则直接返回该负的得分。

  - 在进行对轨迹各个点计算footprintCost之前，会先计算缩放因子getScalingFactor（）。如果当前平移速度小于等于scaling_speed，则缩放因子为1.0，否则，缩放因子为(vmag - scaling_speed) / (max_trans_vel - scaling_speed) * max_scaling_factor + 1.0。然后，该缩放因子会被用于计算轨迹中各个点的footprintCost。

  - 对于footprintCost（），其传入参数scale并没有使用，即目前还不能根据速度改变打分效果。该函数首先计算利用world_model->footprintCost计算footprint_cost，如果footprint_cost小于0，则footprintCost直接返回-6。接着，将x，y转换为地图坐标，如果转化失败则footprintCost返回-7。得到地图坐标后，会对比footprint的代价值和机器人中心点（x，y）的代价值（可通过costmap直接获取），并以较大的值作为返回值。

    ```shell
    `world_model->footprintCost`，是对footprint中的每个线段进行独立计算代价的。首先尝试获取线段的地图坐标，如果获取失败则返回-1。然后，计算利用lineCost函数计算线段代价值，并用footprint_cost参数保存最大的线段代价值，最终如果正常，则返回的也是footprint_cost。但是，如果某个线段的代价值小于0，则world_model->footprintCost直接返回-1。对于线段的代价计算函数lineCost，其实是对线段中的所有像素点（通过bresenham算法得到）进行代价计算，如果像素点的代价为LETHAL_OBSTACLE或者NO_INFORMATION，则该点代价为-1。如果某个点的代价为-1，则该线段的代价也为-1，否则则取所有点中最大的代价作为线段的代价。
    ```
```shell
  以下4个打分项，在prepare函数会对局部地图进行预计算。预计算的思想在于维护一个跟局部地图尺寸对应的路径计算地图（MapGrid类型），该地图中的每个元素为MapCell类型，该类型记录了该元素距离目标点的距离信息或者距离路径的距离信息。在每次打分前，先利用局部路径更新该地图信息，并根据is_local_goal_function参数决定记录距离目标点的距离信息还是距离路径的距离信息，这些信息在后面打分时可以直接使用。
```

- path_costs_

  在prepare函数中，首先复位整个路径计算地图。然后，根据is_local_goal_function参数，分别利用MapGrid::setLocalGoal()和MapGrid::setTargetCells()两个函数分别计算对应的预处理信息。

  - `setLocalGoal`函数，是用来计算轨迹路径计算地图中所有点到局部地图路径末端点的最短距离。首先对映射到局部地图中的路径进行像素级的插补，从而使得整个路径是像素连续的。然后，查找路径在局部地图中的最后一个点，并将路径计算地图中的该点标记为已访问（标记为已访问的点可以进行最短路径计算），然后调用computeTargetDistance函数采用类似dijkstra算法的逐步探索的方式，计算出路径计算地图中所有点（像素级）相对于与标记点的最短距离。

  - `setTargetCells`函数，是用来计算轨迹路径计算地图中所有点到局部地图路径的最短距离。首先将局部地图路径中所有的点标记为已访问，然后调用computeTargetDistance函数就会计算路径计算地图中所有点到整个局部地图路径的最短距离。

  在经过预计算后，对于path_costs_和alignment_costs这两个考虑与路径距离的打分项，其内部的路径计算地图中存储的是所有点到路径的最短距离；对于goal_costs_和goal_front_costs_这两个考虑与局部目标点距离的打分项，其内部的路径计算地图中存储的是所有点到局部目标点的最短距离。

  - 在MapGridCostFunction::scoreTrajectory函数中，依次获取路径中的各个点坐标，并尝试将坐标转为地图坐标，如果转化失败，则scoreTrajecotry函数直接返回-4。否则，从路径计算地图中找到距离局部地图路径的距离，并将该距离（像素级）值作为该点的得分。
  - 对于path_costs_和goal_costs_两个打分项的stop_on_failure参数为true，即如果轨迹中的某个点未障碍物或者未再路径计算地图中探索到，则表示查找该点的得分失败，这会造成scoreTrajectory函数直接返回对应的负的得分。
  - 对于goal_front_costs和alignment_costs_两个打分项对应的stop_on_failure参数为false，这是因为前向打分点有可能会出现计算距离出错的情况，例如超出地图范围，但由于前向打分点的打分失败不会带来危险，因此可以不立刻返回负得分。
  - 轨迹所有点得分的汇总方式有Last、Sum、Product三种方式，默认为Last（且没有对应的配置参数，除非自己修改源码实现），即只考虑轨迹最后一个点的打分。

- goal_costs_

- goal_front_costs_

- alignment_costs_

## Reference
[1] [局部路径规划之DWAPlannerROS分析](https://www.cnblogs.com/sakabatou/p/8297479.html)

