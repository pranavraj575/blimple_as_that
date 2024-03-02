# simple blimp
Create a basic blimp agent

Tested on Ubuntu 20.04 with Python 3.8.

```\pymaze``` contains a modified version of the [pymaze](https://github.com/jostbr/pymaze) github. 

## Installation

1. ### Install [Coppeliasim](https://www.coppeliarobotics.com/)

    Tested with [version 4.3.0 rev 12](https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04.tar.xz) on Ubuntu 20.04
    
    We left the extracted folder in the ```~/Downloads``` folder.

    Example setup:
    ```bash
    cd ~/Downloads
    wget https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04.tar.xz
    tar -xvf CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04.tar.xz
    rm CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04.tar.xz
    ```
    
    Update your ```.bashrc``` with the following (the alias is not required, it just makes it easier to run Coppeliasim)
    
    Replace ```<path to coppelia folder>``` with the path. In our case, it was ```/home/<username>/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04```. Unfortunately it looks like using ```~``` does not work for later build commands, so you need to put the full directory.
    ```bash
    export COPPELIASIM_ROOT_DIR="<path to coppelia folder>"
    alias coppelia="$COPPELIASIM_ROOT_DIR/coppeliaSim.sh"
    ```

    * Note: to remove the annoying error messages on setup, go to ```<coppelia folder path>/system/usrset.txt```. Scroll down to the 'Messaging' section and change 'doNotShowCrashRecoveryMessage' to 'true'.


2. ### Install [ROS2](https://docs.ros.org/)

    Tested with [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) on Ubuntu 20.04
     
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

4. ### Set up this project
   Clone this directory, copy all the ```.lua``` files into the correct place (replace ```<path to coppelia>``` with the path to the Coppeliasim folder). This should be run from wherever you want the repo to be.

    ```bash
    git clone --recurse-submodules https://github.com/pranavraj575/blimple_as_that
    cp blimple_as_that/lua/* /<path to coppelia>/lua/
    cd blimple_as_that
    pip3 install -e .
    ```
    
    For some reason there is no way to tell Coppeliasim to look in different folders for ```.lua``` files.
    Not sure if this is because Coppeliasim is silly or because I am. Either way, this method works.

5. ### Install the ROS2 Coppelia package "according to the [tutorial](https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm)"

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
          git checkout coppeliasim-v4.3.0-rev12
          ```
    * Build the ROS2 package (note: should be run from the workspace directory)
      * Note: this command is buggy, if it fails try looking at the compile instructions in the [github tutorial](https://github.com/CoppeliaRobotics/simROS2)
        ```bash
        cd ~/ros2_ws
        colcon build --symlink-install
        ```
      * The following altarnate build command works better in cases where there are a lot of compiled interfaces for some reason
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

## Tests
* ### Coppelia installation test
  The obvious test to see if Coppeliasim is actually installed:
  ```bash
  coppelia
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

* ### ROS blimp control test
  Tests if coppelia ROS package is set up properly

  First open coppelia with the ```coppelia``` command

  Then run the following file from the ```blimple_as_that``` directory
  ```bash
  python3 src/blimp_agent.py
  ```
  This test will spawn a blimp, then move the blimp up, then after three seconds stop the scene. 

