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

---

## px4_param_migrator

嵌入 ROS 1 的 PX4 参数迁移包，支持：
- 从飞控导出参数到 YAML 文件
- 从 YAML 文件导入参数到飞控
- 通过 **过滤器文件** 精确控制哪些参数参与导出/导入
- 支持 whitelist（白名单）和 blacklist（黑名单）模式
- 参数比对（FC vs 文件）查看差异

### 依赖

```bash
pip install pymavlink pyyaml
```

### 过滤器文件格式 (`config/param_filter.yaml`)

```yaml
mode: whitelist          # whitelist = 仅导出匹配项, blacklist = 排除匹配项

params:
  - MC_*                 # 通配符: 所有多旋翼控制参数
  - MPC_XY_*             # 水平位置控制
  - CAL_*                # 传感器校准参数
  - SYS_AUTOSTART        # 精确匹配单个参数
```

支持 shell 风格通配符：`*, ?, [...]`

### 启动

```bash
roslaunch px4_param_migrator param_migrator.launch

# 自定义过滤器
roslaunch px4_param_migrator param_migrator.launch filter_file:=/path/to/my_filter.yaml

# 导入后重启飞控持久化
roslaunch px4_param_migrator param_migrator.launch reboot_after_import:=true
```

### 调用 ROS 服务

| 服务名 | 功能 |
|--------|------|
| `~export_params` | 从飞控导出参数（经 filter 过滤）到 YAML 文件 |
| `~import_params` | 从 YAML 文件导入参数（经 filter 过滤）到飞控 |
| `~compare_params` | 比对飞控当前参数与本地文件差异 |
| `~reload_filter` | 热加载过滤器文件（无需重启节点） |

```bash
# 导出参数
rosservice call /px4_param_migrator/export_params

# 导入参数（自动使用最新导出文件）
rosservice call /px4_param_migrator/import_params

# 比对差异
rosservice call /px4_param_migrator/compare_params

# 修改过滤器文件后热加载
rosservice call /px4_param_migrator/reload_filter
```

导出文件保存在 `~/.px4_params/px4_params_YYYYmmdd_HHMMSS.yaml`，状态发布到 `/px4_param_migrator/status` 话题。
