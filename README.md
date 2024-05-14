# VIP_ROBOT

## Mapping

### Dependencies
+ `C++` >= 17, `OpenMP` >= 4.5, `CMake` >= 3.10.0, `Eigen` >= 3.2, `Boost` >= 1.54
+ `ROS`
+ [`GTSAM`](https://github.com/borglab/gtsam)
    ```shell
    wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
    unzip gtsam.zip
    cd gtsam-4.1.1/
    mkdir build && cd build
    cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
    sudo make install -j4
    ```
+ [`Teaser++`](https://github.com/MIT-SPARK/TEASER-plusplus)
    ```shell
    git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
    cd TEASER-plusplus && mkdir build && cd build
    cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
    sudo make install -j4
    sudo ldconfig
    ```
+ `tbb` (is used for faster `Quatro`)
    ```shell
    sudo apt install libtbb-dev
    ```

### How to build and use
+ Get the code and then build the main code.
    ```shell
    cd ~/<your_workspace>/src
    git clone https://github.com/engcang/FAST-LIO-SAM-QN --recursive

    cd ~/<your_workspace>
    # nano_gicp, quatro first
    catkin build nano_gicp -DCMAKE_BUILD_TYPE=Release
    # Note the option!
    catkin build quatro -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON
    catkin build -DCMAKE_BUILD_TYPE=Release
    cd ./src/FAST-LIO-SAM-QN/third_party/FAST_LIO
    git branch
    git pull origin <branch>
    cd ./<your_workspace>
    source ./devel/setup.bash
    ```
+ Then run (change config files in third_party/`FAST_LIO`)
    ```shell
    roslaunch fast_lio_sam_qn run.launch lidar:=ouster
    ```

<br>

## Localization

### How to build and use
+ Get the code and then build the main code.
    ```shell
    cd ~/<your_workspace>/src
    git clone https://github.com/engcang/FAST-LIO-Localization-QN --recursive

    cd ~/your_workspace
    # nano_gicp, quatro first
    catkin build nano_gicp -DCMAKE_BUILD_TYPE=Release
    # Note the option!
    catkin build quatro -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON
    catkin build -DCMAKE_BUILD_TYPE=Release
    source ./devel/setup.bash
    cd ./src/FAST-LIO-SAM-QN/third_party/FAST_LIO
    git branch
    git pull origin <branch>
    cd ./<your workspace>
    catkin build
    source ./devel/setup.bash
    ```
+ Then run
    ```shell
    roslaunch fast_lio_localization_qn run.launch lidar:=ouster
    ```
+ Change config files in
    + third_party/`FAST_LIO`/config
    + fast_lio_localization_qn/config
 

## autonomous_exploration_development_environment

### How to build and use
+ Get the code and then build the main code.
    ```shell
    cd ~/<your_workspace>/src
    git clone git@github.com:Kai0139/autonomous_exploration_development_environment.git
    git branch -a
    git checkout -b vipbot origin/vipbot

    cd ~/<your_workspace>
    catkin build -DCMAKE_BUILD_TYPE=Release
    source ./devel/setup.bash
    ```
+ Then run
    ```shell
    roslaunch 
    ```
