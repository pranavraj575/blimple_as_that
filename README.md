# Blimp Swarm
Create a base case blimp environment to build upon for other experimetns in CoppeliaSim

Tested on Windows WSL2 running Ubuntu 22.04 with CoppeliaSim v4.6.0-rev18, Python 3.8, and ROS2 Humble

## Installation

1. ### Install [Coppeliasim](https://www.coppeliarobotics.com/)

    * sudo apt-get update && apt-get upgrade
    * sudo apt-get install libgl1
    * sudo apt-get install python3 pip
    * sudo apt-get install '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev
    * python3 -m pip install pyzmq cbor2
    * wget https://downloads.coppeliarobotics.com/V4_6_0_rev16/CoppeliaSim_Edu_V4_6_0_rev18_Ubuntu22_04.tar.xz
    * tar -xvf CoppeliaSim_Edu_V4_6_0_rev16_Ubuntu22_04.tar.xz
    * ./copelliaSim.sh
    
    Update your ```.bashrc``` with the following (the alias is not required, it just makes it easier to run Coppeliasim)
    
    Replace ```<path to coppelia folder>``` with the path. In our case, it was ```/home/<username>/Downloads/CoppeliaSim_Edu_V4_6_0_rev18_Ubuntu22_04```. Unfortunately it looks like using ```~``` does not work for later build commands, so you need to put the full directory.
    ```bash
    export COPPELIASIM_ROOT_DIR="<path to coppelia folder>"
    alias coppeliaSim="$COPPELIASIM_ROOT_DIR/coppeliaSim.sh"
    ```
   
    **Update Copellia Sim Configuration settings:**
    *  Locate the usrset.txt settings file. For me it was in ```<coppelia folder path>/system/usrset.txt```
    * Change the following parameters
        * doNotShowCrashRecoveryMessage = 'true' (To remove the annoying error messages on setup).
        * additionalLuaPath =  "~/sim/blimpFlow/lua" (To find model lua files)


2. ### Install [ROS2](https://docs.ros.org/)

    Tested with [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) on Ubuntu 22.04
     
    Make sure the following line is in the .bashrc file, or is run every terminal (with `<distro>` replaced by ROS distribution, e.g. `foxy`)
    
    ```bash
    source /opt/ros/<distro>/setup.bash
    ```
    Remember to source the bashrc file after:
   
    ```bash
    source ~/.bashrc
    ```

3. ### Set up the [ZMQ package](https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteApiOverview.htm) into Coppeliasim
    ```bash
    pip3 install pyzmq cbor
    ```
    Then add the following to your ```.bashrc```. Note: since this references COPPELIASIM_ROOT_DIR, it must be on a line after the definition made in step 1
    ```bash
    export PYTHONPATH="$PYTHONPATH:$COPPELIASIM_ROOT_DIR/programming/zmqRemoteApi/clients/python"
    ```
    Again, remember to source the bashrc file after:
   
    ```bash
    source ~/.bashrc
    ```
   
4. ### Install the ROS2 Coppelia package "according to the [tutorial](https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm)"

    However, the tutorial sucks, so following the directions below will work
    
    * Install dependencies
      ```bash
      sudo apt update
      sudo apt-get install xsltproc
      pip3 install xmlschema
      ```
    * make a ROS2 workspace (the name can probably be different)
      ```bash
      cd ~
      mkdir -p ros2_ws/src
      ```
    * clone the [sim_ros2_interface](https://github.com/CoppeliaRobotics/simExtROS2) directory and install dependencies.
      * The best way to do this is with the folder we made
          ```bash
          cp -r /<path to blimple_as_that>/setup_files/sim_ros2_interface ros2_ws/src
          ```
      * However, you can try setting up according to the [tutorial](https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm) like this
          ```bash
          cd ros2_ws/src
          git clone https://github.com/CoppeliaRobotics/simExtROS2
          cd sim_ros2_interface
          git checkout coppeliasim-<your-version-of-coppeliasim>
          ```
    * Build the ROS2 package (note: should be run from the workspace directory)
      * Note: this command is buggy, if it fails try looking at the compile instructions in the [github tutorial](https://github.com/CoppeliaRobotics/simROS2)
        ```bash
        cd ~/ros2_ws
        colcon build --symlink-install
        ```
      * The following alternate build command works better in cases where there are a lot of compiled interfaces for some reason
        ```bash
        cd ~/ros2_ws
        sudo apt install clang
        export CXX=clang++
        colcon build --symlink-install
        ```
        
      * Note: if using a conda environment, you might get an error like [this](https://github.com/colcon/colcon-ros/issues/118)

        running the following might help:
        ```bash
        conda install -c conda-forge catkin_pkg empy lark
        ```
        
5. ### Install Python Packages for this Project
   Clone this directory, copy all the ```.lua``` files into the correct place (replace ```<path to coppelia>``` with the path to the Coppeliasim folder). This should be run from wherever you want the repo to be.

    ```bash
    pip3 install -e .
    ```


## Examples
* ### Coppelia installation test
  The obvious test to see if Coppeliasim is actually installed:
  ```bash
  coppeliaSim
  ```
  This will open the coppelia simulator. Use the ```-h``` argument to test headless mode.

  Note: if you did not alias in installation step 1, you need to type in the path to the ```coppeliaSim.sh``` file
* ### ZMQ test
  Tests if the python coppelia interface works

  First open any coppelia scene with the ```coppelia``` command. Though not necessary, it might be useful to [spawn some objects](https://www.coppeliarobotics.com/helpFiles/index.html) to see the physics work.

  Then run the following file from the ```blimple_as_that``` directory
  ```bash
  python3 tests/testZeroMQRemoteAPI.py
  ```
  This will just play the scene, wait a bit, then pause.

* ### ROS blimp control test (Pranav)
  Tests if coppelia ROS package is set up properly using Pranav's blimp

  ```bash
  python3 src/blimp_agent.py
  ```
  This test will spawn a blimp, then move the blimp up, then after three seconds stop the scene. 

* ### ROS Keyboard blimp control test (Tristan)
  Runs a wall climbing simulation using Tristan's Blimp Model
  ```bash
  python3 runExperiment.py -a 10 -st n -mt 20 -et 300
  ```

* ### ROS blimp control test (Cameron)
  Runs a wall climbing simulation using Cameron's Blimp Model
  ```bash
  python3 runExperiment.py -a 10 -st n -mt 20 -et 300
  ```
