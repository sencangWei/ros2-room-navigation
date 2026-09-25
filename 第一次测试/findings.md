# 发现

- 系统：Ubuntu 22.04.5；`/opt/ros/humble` 存在，但登录 shell 未 source。主机未安装 Gazebo、SLAM Toolbox、Nav2。
- 当前仓库 `HEAD=da9ed8c` 的旧二维演示文件在工作树已全部删除；这次不触碰它们。
- 上游克隆：`https://github.com/6-robot/wpr_simulation2.git` main `78644dc16371738bcfba9dc7dd0df84c4b1ace3d`，路径 `src/wpr_simulation2`。
- 上游 README 明确支持 Ubuntu 22.04/ROS 2 Humble。`slam.launch.py` 使用 `slam_toolbox`；`navigation.launch.py` 使用 `nav2_bringup`。机器人模型 `wpb_home_lidar.model` 有 `/scan` LiDAR 和 `/odom`、`/cmd_vel` Gazebo 插件。
- 上游 `scripts/install_for_humble.sh` 要求 sudo；本机 `sudo -n true` 返回需要密码。Docker daemon 可用。
- 指定 Bilibili 视频网页内容尚未直接核对；与上游 README 的 ROS 2 流程为可核实依据。
- 已找到 ROS 1 上游 `wpr_simulation` README：Gmapping 是 `roslaunch wpr_simulation wpb_gmapping.launch`，导航是 `roslaunch wpr_simulation wpb_navigation.launch`；仅适用 Noetic/catkin。ROS 2 官方仓库使用 SLAM Toolbox/Nav2。
- 建图配置以 SLAM Toolbox 的 `humble` 分支官方 `mapper_params_online_async.yaml` 为底，仅调整本房间的激光最大距离与触发建图的位移、角度阈值。
- 最终配置下 Gazebo 世界通过 `gz sdf -k`，两包 `colcon build` 成功；14 段巡航完成，保存地图为 238 × 158、0.05 m/像素；Nav2 从西端到东端到达，终点一次进展恢复后成功。单次场景已验证，随机目标及实体机器人未测。
