# dVRK simulation operations for WPI RBE 580

# Pre-Requirement And Installation

**Make sure you have a system with Ubuntu 20.04 and ROS Noetic installed.**

**Note**: The installation is path-sensitive. If you have enough knowledge about Linux system and path setup. 
You don't have to follow the procedure but please make suer your path dependence is correct.

## Install AMBF (Asynchronous Multi-Body Framework) Simulator:

### [AMBF](https://github.com/WPI-AIM/ambf)

To install AMBF, you may need some dependencies:

```bash
sudo apt install libasound2-dev libgl1-mesa-dev xorg-dev
```

### Installation and Compile

```bash
cd ~
git clone https://github.com/WPI-AIM/ambf.git
cd ambf && mkdir build
cd build
cmake ..
make
```

### Setup Environment Parameters

```bash
echo "source ~/ambf/build/devel/setup.bash" >> ~/.bashrc
```

**Optional** Create an Alias

In your `~/.bashrc` file, add the following line (assume you are on Ubuntu 20.04 OS):

```
alias ambf_simulator=~/ambf/bin/lin-x86_64/ambf_simulator
```

## Set up Surgical Simulation Environment

### Clone the Repository

```bash
cd ~
git clone https://github.com/surgical-robotics-ai/surgical_robotics_challenge.git
```

### Install the local package

```bash
cd ~/surgical_robotics_challenge/scripts
pip3 install -e .
```

# Run Demonstrations

Firstly, clone the repository:

```bash
cd ~
git clone https://github.com/jackzhy96/wpi_dvrk_simulation.git
```


Then, enable ROS, open a terminal and run:

```bash
roscore
```

## Grasp Needle

In a new terminal, run:

```bash
cd ~/surgical_robotics_challenge
./run_env_SIMPLE_LND_420006.sh
```

In another new terminal, run:

```bash
cd ~/wpi_dvrk_simulation/bagReplay
python3 grasp_needle.py
```

This script will let the left Patient Side Manipulator (PSM 1) to grasp the needle.

## Run Recording

### With PSM

In a new terminal, run:

```bash
cd ~/surgical_robotics_challenge
./run_env_3D_MED_COMPLEX_LND_420006.sh
```

In another new terminal, run:

```bash
cd ~/wpi_dvrk_simulation/bagReplay
python3 record_replay_ambf.py
```

### Needle Only

In a new terminal, run:

```bash
cd ~/surgical_robotics_challenge
./run_env_3D_MED_STRAIGHT_LND_420006.sh
```

**Important: Click the AMBF interface and press `2` in your keyboard.**

It will pop out that the gravity is OFF. Then you can move to the next step.

In another new terminal, run:

```bash
cd ~/wpi_dvrk_simulation/bagReplay
python3 record_replay_needle_ambf.py
```

## Optional: Recording

**You need to run dVRK to enable the Master Tool Manipulators Firstly**

Minor modifications may be required to achieve desired performance.

### Run a simulation environment

For example, use the complex 3D MED suturing training phantom. In a new terminal, run:

```bash
cd ~/surgical_robotics_challenge
./run_env_3D_MED_COMPLEX_LND_420006.sh
```

### Run Teleoperation

In a new terminal, run:

```bash
cd ~/surgical_robotics_challenge/scripts/surgical_robotics_challenge/teleoperation
./mtm_psm_pair_teleop_420006.sh
```

### Run recording

In another new terminal, run: 
```bash
cd ~/wpi_dvrk_simulation/bagRecord/launch
roslaunch simple_rawdata_recorder.launch
```