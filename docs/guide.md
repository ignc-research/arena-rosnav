In this documentation we will guide you through the installation process and explain how to use arena-rosnav for simulation, training and evaluation.

# Installation
## Set up Ubuntu in Windows (For Windows users only)
Note: If you already have a Linux distribution, skip Step 1 and 2.

### 1. Installation of WSL2
Please follow the steps in this [WSL installation guide for Windows 10](https://docs.microsoft.com/en-us/windows/wsl/install-win10) to install WSL2 on your computer.

Note: If you already have a Linux distribution, skip this step.

#### Troubleshooting
You might encounter this problem during installation:
```
Installing, this may take a few minutes...
WslRegisterDistribution failed with error: 0x80370102
Error: 0x80370102 The virtual machine could not be started because a required feature is not installed.
```
This problem can be resolved by enabling CPU virtualization in your BIOS. How you can achieve this depends on your hardware.
[This page](https://www.bleepingcomputer.com/tutorials/how-to-enable-cpu-virtualization-in-your-computer-bios/) might help you with that.

### 2. Installation of Windows-X-Server
To use WSL with graphical programs, an X-server will need to be installed on the Windows 10 system and the DISPLAY variable will need to be set in Bash/Zsh.
One possible program to use is [VcXsrv](https://sourceforge.net/projects/vcxsrv/).

#### Set up DISPLAY variable
After installing the X-server you need to set the DISPLAY variable in your Bash/Zsh.
Use ```nano ~/.bashrc``` or ```nano ~/.zshrc``` and insert the following code on the bottom of the file.

```export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0```

Exit via Ctrl+X. Agree with Y. Press Enter to save the file name (don't change it).

#### Xlaunch Settings
Start Xlaunch and configure it the following way. In the end the configuration can be saved.
##### Display Settings
- Choose Option: Multiple Windows
- Set Display Number to 0
##### Client Settings
- Choose Option: Start no Client
##### Extra Settings
- Choose Option: Disable access control
#### Trouble Shooting
If you encounter problems, you might go to Windows Defender Firewall -> Communication between Applications and Windows Firewall.
Look for VcXsrv and change the Settings to both private and public checked.

## Installation of Arena-Rosnav
For a detailed step-by-step instruction please refer to [Installation.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Installation.md).

