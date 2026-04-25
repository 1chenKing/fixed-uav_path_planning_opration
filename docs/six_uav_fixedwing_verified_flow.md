# 六机固定翼已验证流程

本文记录当前项目中已经实际验证通过的 `6 架固定翼 + PX4 + Gazebo Classic + MAVROS + GUI` 基本流程。

## 当前已验证能力

- 6 架固定翼 SITL 实例可同时启动
- 6 个 MAVROS 命名空间可稳定连接 PX4
- GUI 可上传编队任务
- 6 架飞机可完成：
  - 任务上传
  - 解锁
  - 切换 `AUTO.MISSION`
  - `MISSION_START`
  - 检测到起飞并开始执行任务

## 已验证的关键修复

当前工程已经内置以下 fixed-wing 兼容补丁：

- 解锁前自动设置：
  - `SYS_HAS_NUM_ASPD=0`
  - `CBRK_SUPPLY_CHK=894281`
  - `NAV_DLL_ACT=0`
- 上传任务时自动补充落地航点
- 落地航点会按固定翼下滑角自动拉长进近距离，避免任务被 PX4 判定为无效

## 推荐启动顺序

### 1. 启动六机主链路

```bash
cd /home/chen/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch swarm_bringup swarm_multi_uav_6.launch
```

### 2. 启动独立地面站

```bash
cd /home/chen/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch swarm_bringup swarm_ui.launch
```

## GUI 推荐操作顺序

建议按下面顺序操作，不要跳步：

1. 设置编队类型、间距、锚点、高度
2. 添加任务点
3. 点击 `应用编队`
4. 点击 `上传编队任务`
5. 点击 `全体解锁`
6. 点击 `开始任务`

说明：

- `任务模式` 现在仍可作为辅助按钮使用
- 主路径以 `开始任务` 为准，因为它会按已验证流程触发 `AUTO.MISSION + MISSION_START`

## 推荐第一轮测试参数

- 队形：`横队`
- 间距：`35`
- 航向：`90`
- 高度：`70`

任务点建议：

- 任务点 1：`X=120, Y=0, Z=70`

这套参数已经用于自动验证脚本，适合作为第一轮冒烟测试。

## 已验证脚本

下面这些脚本可用于后续回归：

- `scripts/wsl_diag_uav1_quick_check.sh`
- `scripts/wsl_diag_uav1_instance_logs.sh`
- `scripts/wsl_diag_uav1_arm_gate_patch.sh`
- `scripts/wsl_diag_uav1_mission_with_landing.sh`
- `scripts/wsl_diag_six_mission_flow.sh`

其中最重要的是：

- `scripts/wsl_diag_six_mission_flow.sh`

它已经验证过 6 架飞机全部完成：

- mission push
- arm
- AUTO.MISSION
- mission start

## 当前仍待继续完善

- 让 6 机真实执行“动态换队形”而不只是执行一次任务目标
- 把 2D 避障真正并入 6 机任务生成
- 把 Gazebo 障碍物同步做得更完整
- 进一步减少 PX4 启动初期的瞬时 preflight 告警
