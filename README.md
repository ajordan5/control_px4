# control_px4

## Setup Workspace
1. Install some prerequisites

    ```bash
    $ sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
    $ sudo apt-get install python3-catkin-tools python3-rosinstall-generator -y
    $ sudo apt-get install ros-noetic-mavlink
    $ sudo apt-get install ros-noetic-mavros
    $ pip3 install --user jinja2    
    $ sudo apt-get install libgstreamer-plugins-base1.0-dev
    $ sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y                     
    
    
    # Geographic library
    $ sudo apt install geographiclib-tools -y
    $ install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -) 
    $ sudo bash -c "$install_geo"

    ```
2.Clone the following in your src directory:
  1. This repo
  2. The boat_estimator repo: git@github.com:ajordan5/boat_estimator.git
  3. The boat lanfing version of PX4-Autopilot: git@github.com:ajordan5/PX4-Autopilot.git
  4. The boat landing version of PX4-SITL_gazebo: git@github.com:ajordan5/PX4-SITL_gazebo.git
  5. This listener repo: git@github.com:ajordan5/listener.git
  6. The MAGICC lab's UBLOX_read repo: git@github.com:byu-magicc/UBLOX_read.git
3. Run the following command in the PX4-Autopilot, the UBLOX and the listener directories: 
    
      ```bash
        $ git submodule update --init --recursive
      ```
4. Add the following lines to your .zsh or .bashrc (you may want or need to comment out the python path lines when you are working on other stuff or run those commands every time you launch the simulator rather than adding them to your source file):
      ```bash
        $ export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
        $ export PYTHONPATH=$PYTHONPATH:~/(PATH TO YOUR WORKSPACE)/src/listener/scripts
        $ export PYTHONPATH=$PYTHONPATH:~/(PATH TO YOUR WORKSPACE)/src/boat_estimator/scripts/structs
        $ export PYTHONPATH=$PYTHONPATH:~/(PATH TO YOUR WORKSPACE)/src/boat_estimator/params
      ```
5. Build the gazebo models and plugins by running this command in the PX4-Autopilot directory:
    
      ```bash
        $ make px4_sitl gazebo
      ```
    
6. Build your workspace. 
7. Try launching the simulation:
        
      ```bash
        $ roslaunch control_px4 p2u_w_boat.launch
      ```
  
   

