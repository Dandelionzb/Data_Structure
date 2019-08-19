# `A*`算法

## A*算法总结
```shell
1.把起点加入`open list`。
2.重复如下过程：
	a.遍历`open list`，查找F值最小的节点，把它(F值最小)作为当前要处理的节点，然后移到close list中（自己认为：open中只有起点时，先不查找最小F）
	b.对当前方格的 8 个相邻方格一一进行检查，如果它是不可抵达的或者它在close list中，忽略它。否则，做如下操作：
		- 如果它(相邻方格)不在open list中，把它加入open list，并且把当前方格设置为它的父亲。
		- 如果它(相邻方格)已经在open list中，检查这条路径 (即经由当前方格到达它那里) 是否更近，用G值作参考。更小的 G 值表示这是更好的路径。如果更近，把它的父亲设置为当前方格，并重新计算它的G和F值。如果你的open list是按F值排序的话，改变后你可能需要重新排序。
	c. 遇到下面情况停止搜索：
		- 把终点加入到了 open list 中，此时路径已经找到了。
		- 查找终点失败，并且open list 是空的，此时没有路径。
3.从终点开始，每个方格沿着父节点移动直至起点，形成路径。
		 
```

## Reference
1. [A*算法](https://www.cnblogs.com/21207-iHome/p/6048969.html)
2. [A*算法详解](https://blog.csdn.net/qq_36946274/article/details/81982691)

1.[global_planner源码阅读笔记](https://blog.csdn.net/flyinsilence/article/details/82898797)

2.[ROS Navigation的global_planner类继承关系与实现算法](https://blog.csdn.net/Nksjc/article/details/78812066)

3.[global_planner源码学习](https://blog.csdn.net/qq_41986495/article/details/86013587)

4.[navigation中Global_planner进阶](https://blog.csdn.net/qq_41906592/article/details/89185808)

5.[ros如何及时清除障碍物层或者超声波层的的消息](https://blog.csdn.net/u010918541/article/details/78151704)

6.[costmap_2d: obstacle_layer中关于激光雷达障碍物清除不干净的解决](https://blog.csdn.net/xinmei4275/article/details/88760505)

7.[Costmap2DROS分析](https://www.cnblogs.com/sakabatou/p/8297736.html)

8.[global_planner学习](https://zhuanlan.zhihu.com/p/46212318)
