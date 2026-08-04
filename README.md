# Mini Robot ROS 2 工作空间

这是一个用于学习 ROS 2 节点、话题、参数、自定义消息和服务通信的小型机器人示例工作空间。目前包含 `mini_robot_interfaces` 和 `mini_robot_driver` 两个软件包。

## 软件包

### mini_robot_interfaces

自定义通信接口包，为机器人驱动和其他节点提供统一的数据类型。

- `RobotStatus.msg`：机器人状态消息，包含时间戳、机器人 ID、运行模式、电量、线速度、角速度和急停状态。
- `SetRobotMode.srv`：机器人模式设置服务，请求中携带目标模式，响应中返回是否成功及说明信息。
- `ExecuteTask.action`：任务执行 Action，支持分步反馈执行进度和取消任务。

支持的机器人模式：

- `IDLE`
- `RUNNING`
- `CHARGING`
- `ERROR`
- `AUTO`
- `EMERGENCY`

### mini_robot_driver

机器人驱动示例包，使用 C++ 编写，代码统一位于 `Mrb` 命名空间，并采用头文件与源文件分离的结构。

包含以下可执行程序：

| 程序 | 节点名称 | 说明 |
| --- | --- | --- |
| `robot_driver_node` | `/robot_driver_node` | 模拟机器人电量和状态，提供模式设置服务 |
| `robot_monitor_node` | `/robot_monitor_node` | 订阅机器人状态，根据电量输出正常、警告或错误日志 |
| `robot_mode_client` | `/robot_mode_client` | 发送一次模式设置请求并输出服务响应 |
| `robot_controller_node` | `/robot_controller_node` | 执行机器人任务并每秒反馈一步进度 |
| `robot_controller_client` | `/robot_controller_client` | 发送任务、接收反馈和结果，并可在指定步数取消任务 |

## 通信接口

### 话题

| 名称 | 类型 | 发布者/订阅者 | 说明 |
| --- | --- | --- | --- |
| `/robot/battery` | `std_msgs/msg/Float32` | 驱动节点发布 | 当前电量百分比 |
| `/robot/status` | `mini_robot_interfaces/msg/RobotStatus` | 驱动节点发布，监控节点订阅 | 完整机器人状态 |

### 服务

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/robot/set_mode` | `mini_robot_interfaces/srv/SetRobotMode` | 设置机器人运行模式 |

模式切换遵循以下规则：

- 电量低于 15% 时不能进入 `AUTO` 模式。
- 急停生效时只能切换到 `EMERGENCY` 模式。
- 请求未定义的模式时返回失败及允许的模式列表。

### Action

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/robot/execute_task` | `mini_robot_interfaces/action/ExecuteTask` | 按指定步数执行任务，每秒执行并反馈一步 |

支持以下任务：

- `MOVE_TO_POINT`
- `PATROL`
- `RETURN_HOME`

目标包含任务名称和总步数；反馈包含当前步数、0～100 的进度百分比和当前状态；结果包含是否成功及说明信息。节点通过 ROS 2 定时器每秒执行一步，同一时刻只执行一个任务。非法任务名称、小于等于 0 的总步数或已有任务正在执行时，新任务会被拒绝。

### 驱动节点参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `robot_id` | `mini_robot_01` | 机器人唯一标识 |
| `publish_frequency` | `10.0` | 状态发布频率，单位 Hz |
| `initial_battery` | `100.0` | 初始电量百分比，范围为 0～100 |
| `battery_consumption_rate` | `0.1` | 每次状态更新减少的电量 |
| `emergency_stop` | `false` | 急停是否生效 |
| `max_linear_velocity` | `1.5` | 最大线速度，范围为 0～5 m/s |

## 构建

在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mini_robot_interfaces mini_robot_driver
source install/setup.bash
```

## 运行示例

分别在不同终端中启动驱动节点和监控节点：

```bash
ros2 run mini_robot_driver robot_driver_node
```

```bash
ros2 run mini_robot_driver robot_monitor_node
```

设置机器人模式：

```bash
ros2 run mini_robot_driver robot_mode_client AUTO
```

启动任务控制器：

```bash
ros2 run mini_robot_driver robot_controller_node
```

发送一个巡逻任务，并显示每秒返回的进度反馈：

```bash
ros2 action send_goal /robot/execute_task \
  mini_robot_interfaces/action/ExecuteTask \
  "{task_name: 'PATROL', target_steps: 5}" --feedback
```

使用 C++ Action Client 发送任务并获取反馈和结果：

```bash
ros2 run mini_robot_driver robot_controller_client PATROL 5
```

第三个参数用于在收到指定步数的反馈后取消 Goal。例如在第 3 步取消：

```bash
ros2 run mini_robot_driver robot_controller_client PATROL 10 3
```

也可以直接调用服务：

```bash
ros2 service call /robot/set_mode \
  mini_robot_interfaces/srv/SetRobotMode "{mode: 'RUNNING'}"
```

设置和解除急停状态：

```bash
ros2 param set /robot_driver_node emergency_stop true
ros2 param set /robot_driver_node emergency_stop false
```

查看机器人状态：

```bash
ros2 topic echo /robot/status
```

## 目录结构

```text
src/
├── mini_robot_interfaces/
│   ├── msg/RobotStatus.msg
│   ├── srv/SetRobotMode.srv
│   └── action/ExecuteTask.action
└── mini_robot_driver/
    ├── include/mini_robot_driver/
    └── src/
```
