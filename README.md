# control_px4
1. Install prerequisites

    ```bash
    $ sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
    $ sudo apt-get install python3-catkin-tools python3-rosinstall-generator -y
    $ sudo apt-get install ros-noetic-mavlink
    $ sudo apt-get install ros-noetic-mavros                     
    # Geographic library
		
    $ sudo apt install geographiclib-tools -y
    $ install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -) 
    $ sudo bash -c "$install_geo"

    ```
    
2. Make a Catkin Workspace, then navigate to your workspace's src folder and clone:
	1. this repo
	2. the boat_estimator repo: git@github.com:ajordan5/boat_estimator.git
	3. the listener repo: git@github.com:ajordan5/listener.git
	4. the PX4-SITL_gazebo repo: git@github.com:ajordan5/PX4-SITL_gazebo.git 
	5. the PX4-Autopilot repo: git@github.com:ajordan5/PX4-Autopilot.git
	6. the control_px4 repo: git@github.com:ajordan5/control_px4.git
	7. the UBLOX_read repo: git@github.com:byu-magicc/UBLOX_read.git
3. Then navigate to the listener and PX4-Autopilot directories and run the following command:   
    
  ```bash
        	$ git submodule update --init --recursive
  ```     
4. Add the following lines to your .zshrc or .bshrc:

	```bash
  	$export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
	$export PYTHONPATH=$PYTHONPATH:~/ws2/src/listener/scripts
  	```
5. navigate to the PX4-Autopilot directory and run: 

				```bash
        $ make px4_sitl_gazebo
				```
