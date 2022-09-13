# Astar-Path-Planner-for-2D-grid-map-in-ROS
A*-Path-Planner-for-2D-grid-map-in-ROS

ROS: nav_msgs::OccupancyGrid 格式 二维栅格地图的A星全局规划算法和实现



### 数据集

Astar-Path-Planner-for-2D-grid-map-in-ROS\data\data_2022-09-06-17-21-48.bag

### 运行

```
roslaunch  grid_path_searcher demo.launch
rosbag play data_2022-09-06-17-21-48.bag
```

### 效果

![PIC1](.\PIC\PIC1.png)

### 输入输出

launch文件

```
<launch>

  <node pkg="grid_path_searcher" type="demo_node" name="demo_node" output="screen" required = "true">
      <remap from="~waypoints"       to="/waypoint_generator/waypoints"/>
      <param name="mapTopic" type="string" value="/grid_map_global" />

  </node>

  <node pkg="waypoint_generator" name="waypoint_generator" type="waypoint_generator" output="screen">
      <remap from="~goal" to="/goal"/>
      <param name="waypoint_type" value="manual-lonely-waypoint"/>    
  </node>

  <node name="rvizvisualisation" pkg="rviz" type="rviz" output="log" args="-d $(find grid_path_searcher)/launch/rviz_config/demo_wx.rviz" />

</launch>
```

输入：

mapTopic：" /grid_map_global" （二维占据栅格话题）

输出：

nav_msgs::Path："/global_path"  (全局路径点)