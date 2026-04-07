# Members:

| Name               | UMN Email        |
| ------------------ | :--------------- |
| Samantha Fider     | fider009@umn.edu |
| Finn Hamilton      | hamil876@umn.edu |
| Owen Hobbs         | hobbs178@umn.edu |
| Max Yantes         | yante011@umn.edu |

The only supported versions of Python are 3.11.X.

**You must be using 3.11.X or the code will not run**

# Installation Instructions

On Windows, to **install the USB driver**, download the USB driver from [here](http://zadig.akeo.ie/). Plug in the radio and run the program.
Select the correct device in the dropdown and set libusbK as the target driver, and install. To access the drone while plugged in with USB, repeat the same procedure, except plug in the drone instead of the radio.

After setting up your drones, make sure every drone you wish to use in a swarm is on a different address.
It may help to also have the drones on separate channels. This can be changed by plugging in the drone with USB,

- Using a supported python version: Run `python -m venv .venv` at the root of this repository. On some windows versions, this may be achieved by running `py -3.11 -m venv .venv`.
- **Activate the virtual environment** by sourcing the appropriate script in `.venv/bin` on POSIX systems, or running the appropriate script in `.venv\Scripts\activate` on Windows. See [python documentation](https://docs.python.org/3/library/venv.html) of virtual environments for more details.
- Verify your python version is 3.11.X by running `python --version`. If it is not, run `deactivate`, delete the `.venv` folder and restart.
- Run `python -m pip install -r requirements.txt` to **install python dependencies**.
- Run any script using `python <script>` while the virtual environment is activated.
- You can **deactivate** the virtual environment early by running `deactivate` at any time.
- On subsequent sessions, remember to **reactivate your virtual environment** before running scripts.
- In order to use DroneControlLib in a separate file, add `from DroneControlLib import *` to your file and **make sure DroneControlLib.py is in the same directory** as your new file.

# Code Overview

The drone control is split into several layers to facilitate easy library use.

## Mission Control

This layer...

## Flight Control

This layer implements a PD position and heading controller for the drone. It takes recent Vicon position/yaw samples, compares them to a desired goal state (x, y, z, heading), and outputs roll, pitch, yaw-rate, and thrust commands for the Crazyflie.

#### Main functions
- `add_sample(...)`
    - Stores the latest tracked position, yaw, and timestamp.
- `compute_command(goal)`
    - Main control step:
        - computes position error to the goal
        - rotates x/y error from world frame into the drone’s local frame
        - estimates error rates using recent samples
        - applies PD control to generate roll, pitch, and thrust
        - optionally computes yaw-rate control to match a target heading
        - clamps outputs to safe command limits
- `reset()`
    - Clears stored history.

#### Supporting data classes

- `Goal` – desired position and optional heading
- `ControlCommand` – final command output
- `PIDGains` – tunable controller gains and limits

## Flight Service

This layer manages the live flight loop. It connects the Vicon motion capture, controller, Crazyflie command interface, and logging system into a single background service.

#### Main functions

- `start()`
    - Connects to the Crazyflie, starts telemetry, resets the controller, and launches the background control loop.
- `stop()`
    - Stops the loop, safely shuts down the drone connection, stops telemetry, and saves flight logs.
- `set_goal(goal)` / `clear_goal()`
    - Updates or removes the current target position/heading for the drone.
- `get_latest_*()` methods
    - Provide access to the latest mocap frame, runtime, pose, and most recent control command.
- `_run_loop()`
    - Main background loop:
        - waits for new Vicon frames
        - reads the drone’s current pose
        - sends zero commands if no goal is active
        - feeds pose data into the controller
        - computes a new control command
        - logs pose, goal, command, and telemetry
        - sends the command to the Crazyflie

This class acts as the central flight coordinator, allowing the rest of the program to simply set a goal while flight control runs continuously in the background.

## Flight Logger

This layer handles flight data logging and post-run analysis. It records drone state, control commands, goals, and Crazyflie telemetry during flight, then saves the data and plots for review.

#### Main functions

- `log_sample(...)`
    - Stores one timestep of flight data, including:
        - drone position and yaw
        - goal position/heading
        - controller outputs
        - optional Crazyflie telemetry
- `save_all()`
     - Saves all logged data at the end of a run, including:
        - a CSV flight log
        - position vs. goal plots
        - controller command plots
        - Crazyflie telemetry plots

#### Output files

- `flight_log.csv` – raw recorded data
- `position_vs_goal.png` – actual vs. target trajectory/heading
- `commands.png` – roll, pitch, yaw-rate, and thrust commands
- `crazyflie_telemetry.png` – battery, attitude, thrust, and motor telemetry

This class provides a simple way to record, visualize, and analyze flight performance after each run.

> Layer summaries generated by ChatGPT