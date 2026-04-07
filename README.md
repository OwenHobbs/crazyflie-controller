## Members:

| Name               | UMN Email        |
| ------------------ | :--------------- |
| Samantha Fider     | fider009@umn.edu |
| Finn Hamilton      | hamil876@umn.edu |
| Owen Hobbs         | hobbs178@umn.edu |
| Max Yantes         | yante011@umn.edu |

The only supported versions of Python are 3.11.X.
**You must be using 3.11.X or the code will not run**

## Installation Instructions

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

## Mission Control

## Flight Control

## Flight Service

## Flight Logger
