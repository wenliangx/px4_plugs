# px4_plugs

PX4 自用小插件仓库。

## px4_log_manager

嵌入 ROS 1 的 PX4 飞行日志管理包，支持：
- 通过 MAVLink 从飞控下载 `.ulg` 飞行日志
- 解析日志内容（姿态、GPS、电池、飞行距离等）
- 下载完成后自动删除飞控上的旧日志

### 依赖

```bash
pip install pymavlink pyulog
```

需要 ROS 1 (noetic / melodic) 环境。

### 编译

```bash
cd <your_catkin_ws>/src
ln -s /path/to/px4_plugs/px4_log_manager .
cd ..
catkin_make
source devel/setup.bash
```

### 启动

```bash
# SITL 仿真（默认 UDP 14550）
roslaunch px4_log_manager log_manager.launch

# 串口直连
roslaunch px4_log_manager log_manager.launch connection_url:=/dev/ttyACM0

# 不自动删除飞控日志
roslaunch px4_log_manager log_manager.launch auto_erase:=false
```

### 调用 ROS 服务

| 服务名 | 功能 |
|--------|------|
| `~download_logs` | 下载飞控上所有日志 |
| `~parse_last_log` | 解析本地最新日志 |
| `~erase_logs` | 删除飞控上所有日志 |
| `~full_cycle` | 下载 → 解析 → 删除（一键） |

```bash
# 一键完成：下载 + 解析 + 删除
rosservice call /px4_log_manager/full_cycle

# 仅下载
rosservice call /px4_log_manager/download_logs
```

解析结果会发布到 `/px4_log_manager/log_summary` 话题。
