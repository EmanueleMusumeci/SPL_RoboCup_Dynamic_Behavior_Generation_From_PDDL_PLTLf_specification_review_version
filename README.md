# Dynamic Behaviors Generation in RoboCup SPL from PDDL+PLTLf formal specification

## Installation

### Supported distros:
* Ubuntu 16.04/16.10/17.04/17.10/18.04/18.10/19.04/19.10/20.04
* Linux Mint 18.x / 20.x

### Install needed libraries
Open a terminal `Ctrl`+`Alt`+`t` (usually) and type the followings commands: <br>
```
sudo apt install build-essential cmake clang make qtbase5-dev libqt5opengl5-dev libqt5svg5-dev libglew-dev net-tools graphviz xterm qt5-default libvtk6-dev zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libopenexr-dev libgdal-dev libtbb-dev libpng16-16:i386 libtbb2:i386 libjpeg62:i386 libtiff5:i386 libv4l-0:i386
```
<br>

### Configure GitHub :
  
  Install git
  ```
  sudo apt-get install git git-core git-gui git-doc gitg
  ```
  
  Follow [this guide](http://help.github.com/linux-set-up-git) to set up git (Attention: "Your Name here" means "Name Surname")

  Colorize git: 
  ```
  git config --global color.status auto
  ```
<br>


### Update and upgrade your pc and reboot.
```
sudo apt-get update
sudo apt-get upgrade
sudo reboot
```
<br>

### Install the SPQR Team RoboCup framework:
Clone the repository (need to have permissions on GitHub) 
```
mkdir RoboCup
cd RoboCup
git clone https://github.com/SPQRTeam/spqrnao2021.git
```


### Install OpenCV3.1
Dowload source of OpenCV:
 `$ wget https://github.com/opencv/opencv/archive/3.1.0.zip`

Unzip the files
 `$ unzip 3.1.0.zip`

Generate the make file
```
cd opencv-3.1.0
mkdir build && cd build
cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DWITH_CUDA=OFF ..
```

**NOTICE**: if you have trouble with _stdlib_ add this flag
```
-DENABLE_PRECOMPILED_HEADERS=OFF
```

Compile
```
make -j<num of cores of your cpu>
```

Run OpenCV install script 
```
sudo make install
``` 
and then
```
sudo ldconfig
```

#### Possible problems at this step

The following errors may happen for some users, but not others. *If you incur into one, please report it to us as soon as possible, so we can update this wiki with the proper error message.*

* **IMPORTANT FOR ANACONDA/MINICONDA USERS:** Either of them can mess with the installation process by generating seemingly unrelated errors *(e.g. an undefined reference to libtiff)*. The easiest way is to temporarily disable them by following this procedure: [(source)](https://github.com/colmap/colmap/issues/188#issuecomment-440665679)
  - Open your .bashrc: `$ nano ~/.bashrc`
  - Locate the line that adds <ana/mini>conda to your PATH. It may look like `export PATH="/home/<user>/anaconda3/bin:$PATH` for anaconda, and similarly for miniconda. If you can't find it in .bashrc, try your .profile as well.
  - Comment out that line *(add a `#` as the first character of that line)*.
  - *Open a new terminal window* and carry out the OpenCV installation procedure as above.
  - Un-comment that line in your .bashrc
* **If you get an error message related to ffmpeg:** Install ffmpeg with the command `sudo apt install ffmpeg`
* **If the compiler says some flag (one of which may be CODEC_FLAG_GLOBAL_HEADER) was not declared:** You may have to add the missing defines to the OpenCV code. Open the file `<opencv folder>/modules/videoio/src/cap_ffmpeg_impl.hpp` and add the folowing lines: [(source)](https://stackoverflow.com/a/47005401)
```c++
    #define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
    #define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
    #define AVFMT_RAWPICTURE 0x0020
```
* **If you get errors related to libasound:** Install the corresponding headers with the command `sudo apt install libasound2-dev`

### Compile the code 
#### Ubuntu 16 or Mint 18 users
Set Clang-6.0 before you compile: 
```
sudo apt-get install -y clang-6.0 
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-3.8 100 
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-6.0 1000 
sudo update-alternatives --install /usr/bin/clang++ clang /usr/bin/clang-3.8 100 
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.8 100 
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-6.0 1000 
sudo update-alternatives --config clang 
sudo update-alternatives --config clang++
```
#### All users
To compile **(for all OS versions)**:
Compile the code (move to the `Make/Linux` folder of the repo)
```
cd RoboCup/master_thesis/spqrnao2021/Make/Linux
make CONFIG=<Develop/Debug/Release>
```

### Install Python dependencies
Install the ssl library:
```
sudo apt-get install libssl-dev
```

Using a python package manager install the `twisted` package and the `service_identity` package (e.g. `pip install twisted service_identity`)

### Install NodeJS dependencies
* Install NodeJS
* Using npm install the `websocket` package (e.g. `npm install websocket`)




### Install MONA
* Clone the following repo `https://github.com/cs-au-dk/MONA`
* Follow the install instructions at [this page](https://github.com/cs-au-dk/MONA/blob/master/INSTALL)

HINT:
```
cd <MONA_DIR>
sudo make install-strips
```

NOTICE: you might need to install **flex**, **bison** and **byacc**: 
```
sudo apt-get install flex bison byacc
```
Then, go into the MONA directory and uninstall:
```
sudo make uninstall
sudo make clean
```
then rebuild:
```
sudo make install-strip
```

### Install dependencies for next repo 
Install OpenJDK 8 (NOTICE: the next repo can not be built with Java versions above 8):
```
sudo apt-get install openjdk-8-jdk
```
Install swig:
```
sudo apt-get install swig3.0
```
Install latexmk:
```
sudo apt-get install latexmk
```

### Download and install the `planning-for-past-temporal-goals` repo from the [Whitemech research group](https://github.com/whitemech)
Follow the [install guide for this repo](https://github.com/whitemech/planning-for-past-temporal-goals/tree/benchmark)
<!--* Clone the repo (anywhere):
```
git clone -b benchmark https://github.com/whitemech/planning-for-past-temporal-goals
```
* Install the dependencies required for pygraphviz: 
```
sudo apt-get install python3-dev graphviz libgraphviz-dev pkg-config
```
* Install the pip package:
```
cd planning-for-past-temporal-goals
pip install .
```
* Go to `third_party` directory and manually clone the required repositories:
```
cd third_party
git clone https://github.com/aibasel/downward; git clone https://github.com/robertmattmueller/myND; git clone https://github.com/whitemech/pddl; git clone https://github.com/QuMuLab/planner-for-relevant-policies
```

* Install the dependencies required for these repos:
```
sudo apt-get install gcc-multilib g++-multilib
```
* Move to the main repo directory and run the `build.sh` script
```
cd ..
./scripts/build.sh
```
-->



## Running

The network infrastructure is set to run on localhost. Make sure you're connected to a network anyway, even if not connected to the internet (to have the loopback work anyway)

<!--
### Task-based controller
#### Run SimRobot
Move to `Develop/` folder
```
cd RoboCup/master_thesis/spqrnao2021/Build/Linux/SimRobot/<Develop/Debug/Release>/
```

Run SimRobot
```
./SimRobot
```

Click on File->Open and then move to the `RoboCup/master_thesis/spqrnao2021/Config/Scenes` folder and open the `1vs3Dummies.ros2` scene. This scene features one playing robot and 3 draggable inactive robots acting as obstacles.

#### Run the python server
Move to `robocup_spl_temporal_goals/` folder
```
cd RoboCup/master_thesis/spqrnao2021/robocup_spl_temporal_goals/
```

Run the python server 
```
python async_socket_NAO.py
```

#### Run the NodeJS server
Move to `robocup_spl_temporal_goals/web_interface` folder
```
cd RoboCup/master_thesis/spqrnao2021/robocup_spl_temporal_goals/web_interface
```

Run the python server
```
node clientUDP.js
```

#### Open the GUI in the browser
Open any browser and connect to 127.0.0.1:3000
-->

### Planning mode
Any experiment can be run using the SimRobot utility from the B-Human framework, which will emulate the robot in a simulator.

#### Run SimRobot
Move to `Develop/` folder
```
cd RoboCup/master_thesis/spqrnao2021/Build/Linux/SimRobot/<Develop/Debug/Release>/
```

Run SimRobot
```
./SimRobot
```

Click on File->Open and then move to the `RoboCup/master_thesis/spqrnao2021/Config/Scenes` folder and open the `1vs3Dummies.ros2` scene. This scene features one playing robot and 3 draggable inactive robots acting as obstacles (NOTICE: you can specify the full path to the `.ros2` file to avoid navigating to it in SimRobot)

#### Run the Python server with a specific experiment
To run an experiment:
  1) *[Skip this step if you want to run an existing experiment file]* 
  Create an experiment file in a subfolder of the "experiments" folder:
      The file has to contain a `setup_experiment()` function. This function
      will have to setup the registries appropriately and build a plan/policy/DFA
      for each required role, then return them in a dictionary like:

          <robot_role> -> <plan/policy/DFA handler instance>
  
  2) The `__init__.py` file in the "experiments" folder, when imported, will dinamically
      load all experiment files in the subfolders of the "experiments" folder and then
      collect them as modules into an "experiments" list, from which they can be accessed
      to run the setup_experiment() function
  
  3) Then the `setup()` method of the `setup_env` module will be called so that the network modules will be run.
  
  4) Finally plan/policy/DFA handlers will be passed to the CommunicationManager manager
      through the update_plan() method

  To run an experiment execute the following command on a terminal:

  ```
  python run_experiment.py <EXPERIMENT_SUBFOLDER.EXPERIMENT_NAME>
  ```

  where:
  * The module run_experiment.py is located in the spqrnao2021/robocup_spl_temporal_goals folder
  * `EXPERIMENT_SUBFOLDER` is a subfolder of the "experiments" folder
  * `EXPERIMENT_NAME` is the name of a file providing the `setup_experiment()` function