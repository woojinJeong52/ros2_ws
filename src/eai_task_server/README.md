# eai_task_server

ROS 2 Python package that publishes hardcoded EAI task messages.

**Note:** Examples in this package are created for testing purposes. There is no guarantee for identical tasks during competitions.

## Dependencies

- ROS 2 (tested on Humble; expected to work on Jazzy and newer distros as well)
- [`sml_messages`](https://github.com/robocup-sml/sml_messages) (must be available in your workspace and built before this package)

## Build + Setup

```bash
cd ~/ros2_ws && colcon build --packages-select sml_messages eai_task_server
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
```

Run the `source` command in each new terminal before using `ros2 run` / `ros2 launch`.

## Run

```bash
ros2 run eai_task_server task_publisher --ros-args -p scenario:=production -p stage:=beginner
```

In a second terminal, run:

```bash
ros2 run eai_task_server task_listener
```

## Parameters

- `scenario`: `production`, `recycling`, or `lifecycle`
- `stage`: `entry`, `beginner`, or `advanced`
- `topic_name`: full task topic (default `/eai/task`)
- `side_a_topic_name`: side A only task topic (default `/eai/task/side_a`)
- `side_b_topic_name`: side B only task topic (default `/eai/task/side_b`)
- `publish_period_sec`: publish period in seconds (default `1.0`)
- `publish_once`: publish a single message and exit (default `false`)

## Publishing Topics

- `/eai/task`: full task (orders + stations from side A and side B)
- `/eai/task/side_a`: same orders, but only side A stations
- `/eai/task/side_b`: same orders, but only side B stations

## How to Adapt

Teams can adjust task definitions for local testing by editing the task builders directly in [./eai_task_server/task_publisher.py](./eai_task_server/task_publisher.py#L77).

Useful places to modify:

- Scenario/stage task contents (orders + required materials): [./eai_task_server/task_publisher.py](./eai_task_server/task_publisher.py#L77)
- Arena layout station map (side A + side B station list): [./eai_task_server/task_publisher.py](./eai_task_server/task_publisher.py#L21)
- Scenario/stage routing map (`scenario` + `stage` to builder function): [./eai_task_server/task_publisher.py](./eai_task_server/task_publisher.py#L239)

After changes, rebuild and source again:

```bash
cd ~/ros2_ws && colcon build --packages-select sml_messages eai_task_server
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
```

## Launch Server

```bash
ros2 launch eai_task_server task_server.launch.py scenario:=lifecycle stage:=advanced
```

One-shot launch example:

```bash
ros2 launch eai_task_server task_server.launch.py scenario:=production stage:=beginner publish_once:=true
```

## Launch Server + Listener

```bash
ros2 launch eai_task_server task_server_with_listener.launch.py scenario:=recycling stage:=advanced
```

Combined one-shot example:

```bash
ros2 launch eai_task_server task_server_with_listener.launch.py scenario:=production stage:=beginner publish_once:=true
```
