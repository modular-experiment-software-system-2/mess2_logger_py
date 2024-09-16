# mess2_logger_py
ROS2 package for logging messages on topics directly to .csv files.

## Overview
This package contains a general purpose ROS2 logger node that dynamically creates subscription instances for topics without specifying the message types of said topics.

## License
This package is released under an [Apache 2.0 license](https://github.com/marinarasauced/mess2_logger_py/blob/main/LICENSE).

**Authors:** [Marina Nelson](https://github.com/marinarasauced), [Vik](https://github.com/Vik095) <br />
**Maintainer:** Marina Nelson, marinarasauced@outlook.com

This package has been tested in ROS2 Humble and Jazzy on Ubuntu 22.04 and 24.04, respectively.

## Installation
### Prerequisites
- Ubuntu 22.04 or Ubuntu 24.04
- ROS2 Humble or Jazzy
- ROS2 Workspace

These instructions assume that you meet the above prerequisites. Do not proceed if you have not already installed ROS2 Humble or Jazzy on Ubuntu 22.04 or 24.04, respectively, and if you have not created a ROS2 workspace.

### Building
Clone the repository into your ROS2 workspace's `src` directory:

```zsh
cd ~/your_ws/src
git clone https://github.com/marinarasauced/mess2_logger_py.git
```

Update your ROS2 dependencies:

```zsh
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
```

Compile the package using `colcon`:

```
cd ~/your_ws
colcon build --symlink-install --packages-select mess2_logger_py
```

Source the setup script:

```
source ~/your_ws/install/setup.zsh
```

## Usage

> [!WARNING]  
> Before running the node, please read through the below documentation.

Run the `log_topics_to_csvs` node with:

```zsh
ros2 run mess2_logger_py log_topics_to_csvs
```


## Config Files

## Launch Files

## Nodes

### log_topics_to_csvs

Create subscriptions to topics dynamically and records messages on said topics to respective log .csv files.

#### Subscriptions

Subscriptions are created dynamically using the `topics` parameter.

#### Parameters

- **`dir_logs`** (string, default: "Projets/testing")

	The relative path from the home directory to the logs directory.

- **`dirs_sub`** (list\[string\], default: \["actor", "actor1"\])

	The subdirectories in which the log .csv files are written; i.e., actor/actor1/.

- **`topics`** (list\[string\], default: \["/topic1", "topic2"\])

	The topics to log.

- **`period_timer`** (double, default: 5.0)

	The period at which the node checks if initially unadvertised topics are now advertised.
