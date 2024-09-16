# mess2_logger_py
ROS2 package for logging messages on topics directly to .csv files.

## Overview
This package contains a general purpose ROS2 logger node that dynamically creates subscription instances for topics without specifying the message types of said topics.

## License
This package is released under an [Apache 2.0 license](https://github.com/marinarasauced/mess2_logger_py/blob/main/LICENSE).

**Authors:** [Marina Nelson](https://github.com/marinarasauced),  <br/>
**Affiliation:** [ACE Lab](https://rvcowlagi-research.owlstown.net/) <br/>
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
Start the `log_topics_to_csvs` node with:

> [!CAUTION]  
> Before running the node, please read the below documentation. The directory in which the log .csv files are written to is CLEARED on run.

```zsh
ros2 launch mess2_logger_py template.launch.py file_config:=config/template.yaml
```

When you start the node,

## Config Files

- **`config/template.yaml`**

    Defines the default values for the `dir_logs`, `dirs_sub`, `topics`, and `period_timer` parameters for use with `template.launch.py`.

## Launch Files

- **`template.launch.py`**

    Provides a template for building launch files for the `log_topics_to_csvs` node using config files.

## Nodes

### log_topics_to_csvs

Create subscriptions to topics dynamically and records messages on said topics to respective log .csv files.

Parameters are used to define the path at which the log .csv files are written and the topics which are subscribed to. On run, a save directory is created at `~/dir_logs/dirs_sub[i]/dirs_sub[i+1]/` for `n` elements of the `dirs_sub` parameter. If this directory contains files, those files are cleared. For each topic, a log .csv file is created where `/` characters are replaced with `_` characters; i.e., `/your/topic` becomes `_your_topic.csv`. All messages for logged topic are appended to the corresponding .csv file. If topics are initially unadvertised when the node is run, they are appended to a list of unadvertised topics. A timer callback periodically checks if these initially unadvertsied topics are now advertised, and if they are, initializes the logging of those now advertised topics.

#### Subscriptions

Subscriptions are created dynamically using the `topics` parameter.

#### Parameters

- **`dir_logs`** (string, default: "Projets/testing")

	The relative path from the home directory to the logs directory.

- **`dirs_sub`** (list\[string\], default: \["actor", "actor1"\])

	The subdirectories in which the log .csv files are written; i.e., actor/actor1/. The relative path from home to `dir_logs` and subsequently `dirs_sub` is CLEARED when the node starts.

- **`topics`** (list\[string\], default: \["/topic1", "topic2"\])

	The topics to log.

- **`period_timer`** (double, default: 5.0)

	The period at which the node checks if initially unadvertised topics are now advertised.
