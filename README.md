# Piper ROS2 双臂数据采集流程
## 采集数据

采集时建议分别打开 3 个终端。`start_robot.sh` 和 `camera.sh` 都是常驻进程，不能在同一个终端里顺序执行完。

终端 1：启动双臂控制节点

```bash
cd /home/agilex/piper_ros
bash start_robot.sh
```

终端 2：启动三路 RealSense 相机

```bash
cd /home/agilex/piper_ros
bash camera.sh

rqt_image_view &
rqt_image_view &
rqt_image_view &
```

终端 3：开始采集 episode

```bash
cd /home/agilex/piper_ros
bash collect.sh
```

采集脚本实际入口是：

```bash
collect_data/collect_data_ros2.py
```

`collect.sh` 当前默认保存到：

```text
/home/agilex/data/clean_desktop_519/519/episode_<idx>.hdf5
```

采集过程中的按键：

```text
Enter       结束当前 episode 并保存
ScrollLock  给当前帧打 subtask 标记
Esc         停止键盘监听
```

注意：`collect.sh` 目前只传入了 depth topic，但没有开启 `--use_depth_image True`，所以默认不会保存深度图。如需保存深度图，需要在 `collect.sh` 的 python 参数中加上：

```bash
--use_depth_image True
```

## 查看数据

查看指定 HDF5 文件：

```bash
cd /home/agilex/piper_ros
bash run_view_data.sh --file /home/agilex/data/clean_desktop_519/519/episode_0.hdf5
```

循环重播并显示速度、力矩信息：

```bash
cd /home/agilex/piper_ros
bash run_view_data.sh --file /home/agilex/data/clean_desktop_519/519/episode_0.hdf5 --loop --vel --effort
```

按任务名和 episode 编号查看：

```bash
cd /home/agilex/piper_ros
bash run_view_data.sh --task 519 --episode 0 --dataset-dir /home/agilex/data/clean_desktop_519
```

如果采集时保存了深度图，可以加：

```bash
bash run_view_data.sh --file /home/agilex/data/clean_desktop_519/519/episode_0.hdf5 --depth
```

## 回放数据

回放前先启动机械臂控制节点。建议分别打开 2 个终端。

终端 1：启动回放用双臂控制节点

```bash
cd /home/agilex/piper_ros
bash eval.sh
```

终端 2：选择一种回放模式

末端位姿回放：

```bash
cd /home/agilex/piper_ros
bash replay.sh eef
```

关节数据回放：

```bash
cd /home/agilex/piper_ros
bash replay.sh joint
```

`eef` 和 `joint` 是两种模式，通常二选一执行，不需要连续都跑。

注意：`replay.sh` 里也有 `DATASET_DIR`、`TASK_NAME`、`EPISODE_IDX` 配置。回放前需要确认它们和要回放的数据文件一致；如果刚用当前 `collect.sh` 采集，默认任务目录是 `519`。

## 推理

```bash
cd ~/openpi
source /home/agilex/piper_ros/install/setup.bash
PYTHONPATH=$PWD:$PWD/packages/openpi-client/src:$PYTHONPATH \
  /usr/bin/python3.8 -m examples.aloha_real.main \
    --host 58.199.191.141 \
    --port 8000 \
    --control-mode end_pose
```

也可以指定其他服务地址：

```bash
PYTHONPATH=$PWD:$PWD/packages/openpi-client/src:$PYTHONPATH \
  /usr/bin/python3.8 -m examples.aloha_real.main \
    --host 58.199.191.141 \
    --port 8000
```


## 注意事项

`start_robot.sh` 和 `eval.sh` 当前传参使用的是 `girpper_exist`，launch 文件也沿用了这个拼写；但控制节点代码里声明的是 `gripper_exist`。因为默认值是 `True`，目前开启夹爪时通常不受影响。如果后续需要关闭夹爪，建议统一修正为 `gripper_exist`。

采集依赖这些 topic 正常发布：

```text
/camera/top/color/image_raw
/camera/left/color/image_raw
/camera/right/color/image_raw
/joint_states_left
/joint_states_right
/end_pose_left
/end_pose_right
```

可以用下面命令检查：

```bash
ros2 topic list
ros2 topic hz /joint_states_left
ros2 topic hz /camera/top/color/image_raw
```
