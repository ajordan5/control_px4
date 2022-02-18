# control_px4

## Setup Workspace
1. Install ROS Noetic: http://wiki.ros.org/ROS/Installation

2. Create a workspace (i.e. rtk_ws): http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

3. Clone the following in your src directory:
    1. This repo: https://github.com/ajordan5/control_px4
    2. The boat_estimator repo: https://github.com/ajordan5/boat_estimator
    3. The airsim_publish repo: https://github.com/ajordan5/airsim_publish
    4. The MAGICC lab's UBLOX_read repo: https://github.com/byu-magicc/UBLOX_read

4. Run the following command in the UBLOX directory: 
    
      ```bash
        $ git submodule update --init --recursive
      ```

5. Make a directory and subdirectory called "data/airsim" in your home directory (one level above your workspace directory)

6. Install the following python packages, if needed:

      ```bash
        $ python3 -m pip install --user numpy scipy
        $ pip3 install mavsdk
        $ pip3 install navpy 
        $ pip3 install masgpack-rpc-python
        $ pip3 install airsim
      ```

7. Run the following command in your workspace directory to build it:
      ```bash
        $ catkin_make
      ```

8. Add the following lines to your .zsh or .bashrc (you may want or need to comment them out when you are working on other stuff or run those commands every time you launch the simulator rather than adding them to your source file):
      ```bash
        source /opt/ros/noetic/setup.bash
        source rtk_ws/devel/setup.bash
      ```

## Get AirSim Source Code

1. Download and extract the latest Airsim source code available on the Airsim Github repo: https://github.com/microsoft/AirSim/releases

2. Navigate to the Airsim source code directory and run ./setup.sh followed by ./build.sh.

## Install PX4 SITL

1. Run the following commands in a location where you like to clone git repositories:
      ```bash
        $ mkdir -p PX4
        $ cd PX4
        $ git clone https://github.com/PX4/PX4-Autopilot.git --recursive
        $ bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
        $ cd PX4-Autopilot
      ```
    See https://microsoft.github.io/AirSim/px4_sitl/ and https://microsoft.github.io/AirSim/px4_build/ for further questions.

2. Change your AirSim settings.json file  (by default in the Documents/Airsim directory):
      ```bash
        {
          "SettingsVersion": 1.2,
          "SimMode": "Multirotor",
          "ClockType": "SteppableClock",
          "Vehicles": {
              "PX4": {
                  "VehicleType": "PX4Multirotor",
                  "UseSerial": false,
                  "LockStep": true,
                  "UseTcp": true,
                  "TcpPort": 4560,
                  "ControlPortLocal": 14540,
                  "ControlPortRemote": 14580,
                  "Sensors":{
                      "Barometer":{
                          "SensorType": 1,
                          "Enabled": true,
                          "PressureFactorSigma": 0.0001825
                      }
                  },
                  "Parameters": {
                      "COM_RC_IN_MODE": 1,
                      "COM_RCL_EXCEPT": 4,
                      "NAV_RCL_ACT": 0,
                      "NAV_DLL_ACT": 0,
                      "COM_OBL_ACT": 1,
                      "LPE_LAT": 47.641468,
                      "LPE_LON": -122.140165,
                      "EKF2_AID_MASK": 1,
                      "EKF2_HGT_MODE": 3,
                      "EKF2_EV_DELAY": 175.0,
                      "EKF2_EV_POS_X": 0.0,
                      "EKF2_EV_POS_Y": 0.0,
                      "EKF2_EV_POS_Z": 0.0,
                      "ATT_MAG_DECL_A": 1,
                      "ATT_MAG_DECL": 0.0
                  }
              }
          }
        }
      ```
    Note the additional parameters than what the link in Step 1 requires. These are necessary to run 
    the simulation without an RC.

3. Run PX4 in the PX$-Autopilot directory:
      ```bash
        $ make px4_sitl_default none_iris
      ```

## Launch the simulation

1. Run the simulation in UnrealEngine
    1. Download and extract the precompiled environment for the BoatLanding project from GitHub: https://gitlab.magiccvs.byu.edu/lab/airsim_environments
    2. (Note: Do not do this last step if you use Jupyter Notebooks or iPython. We are working on a fix for this, but for now there is a breaking dependency conflict with these packages. This can be avoided by using vitual python environments, which is more complex than this guide goes.) Run pip install Airsim.
    3. In the environment directory, run the .sh file within the LinuxNoEditor folder to launch the simulation

2. Run PX4 in the PX4-Autopilot directory:
      ```bash
        $ make px4_sitl_default none_iris
      ```

3. Run the launch file for boat landing:
        
      ```bash
        $ roslaunch control_px4 airsim_w_boat.launch
      ```
  
## Running in hardware
   
When running outdoor or in mocap, the setup scripts will need to be run (found in control_px4/scripts/setup). For running airsim, no setup script is needed; just make sure these airsim settings as defined in the Install PX4 SITL section are correct.
