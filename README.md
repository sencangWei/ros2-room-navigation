# ROS 2 房间自主避障入门

这个练习使用 **ROS 2 Humble + RViz**，不需要 Gazebo。它是二维运动学仿真：房间为 8 m × 5 m，蓝色箭头机器人从 `(-3.25, 0)` 出发，绕开中央障碍，到绿色圆形终点 `(3.25, 0)`。地图、激光和里程计由仿真节点发布；导航节点用 A* 规划，并用激光测距限制过近时的前进。它没有真实底盘动力学或 SLAM。

## 1. 构建和运行

在本目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select room_nav
source install/setup.bash
ros2 launch room_nav room.launch.py
```

约 30–60 秒后终端应出现 `REACHED_GOAL`，机器人停在房间另一端。关闭窗口后按 `Ctrl+C` 停止。若机器没有图形桌面，可用：

```bash
ros2 launch room_nav room.launch.py rviz:=false
```

若从 **Snap 版 VS Code 的集成终端**启动 RViz 时出现 `/snap/core20/.../libpthread.so.0` 错误，可在本目录用干净的图形环境启动：

```bash
env -i HOME="$HOME" USER="$USER" DISPLAY="$DISPLAY" PATH=/usr/bin:/bin \
  bash --noprofile --norc -c \
  'source /opt/ros/humble/setup.bash; source install/setup.bash; exec ros2 launch room_nav room.launch.py'
```

## 2. 看懂 ROS 2 中的数据流

在**另一个终端**同样运行两条 `source` 命令，然后尝试：

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic info /cmd_vel -v
rqt_graph
```

| 名称 | 消息类型 | 用途 |
| --- | --- | --- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 房间和障碍的已知静态地图 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 模拟 360° 测距，每 5° 一束 |
| `/odom` | `nav_msgs/msg/Odometry` | 机器人的位置、朝向和速度 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom` 到 `base_link` 的坐标变换 |
| `/planned_path` | `nav_msgs/msg/Path` | A* 规划出的绕行路线 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 导航节点发给仿真节点的速度命令 |

`/map` 和 `/planned_path` 使用 transient local QoS，晚启动的 RViz 仍能收到；其他运动数据持续发布。仿真节点超过 0.5 秒没有收到 `/cmd_vel` 就停止，避免导航节点退出后机器人继续移动。

## 3. 按顺序动手改

1. 打开 [world.py](src/room_nav/room_nav/world.py)，修改 `OBSTACLES` 的第二个矩形；重启程序，看路径是否变化。每个矩形为 `(左, 下, 右, 上)`，单位米。
2. 打开 [navigator.py](src/room_nav/room_nav/navigator.py)，找到 `clearance`，试着从 0.38 m 调到 0.48 m；观察规划路径离障碍更远。
3. 找到 `command.linear.x`，将最大速度从 0.32 m/s 调到 0.20 m/s；用 `ros2 topic hz /odom` 和终端到达时间比较。
4. 用 `ros2 topic info /scan -v` 与 `ros2 topic info /cmd_vel -v` 确认谁发布、谁订阅、QoS 是否匹配。

修改 Python 文件后，本项目使用 `--symlink-install`，重新运行 launch 即可；修改安装元数据或 launch/RViz 文件后重新构建。新增障碍后若 `No collision-free path`，检查是否堵死通道或距离起点/终点太近。

## 范围

这个版本教学重点是节点、话题、消息、QoS、坐标系、规划和闭环控制。路径规划使用已知地图，激光用于近距离前进保护；没有未知地图探索、动态障碍预测或物理引擎碰撞模型。若作业明确要求 Gazebo、URDF、Nav2 或 SLAM，需要在此基础上换成对应的三维仿真和导航栈。
