# FAST-LIO-SAM

  + This repository is a SLAM implementation combining [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) with pose graph optimization and loop closing based on [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) paper
      + Loop-detection is based on radius search and ICP is used to calc matching
  + Note: similar repositories already exist
      + [FAST\_LIO\_LC](https://github.com/yanliang-wang/FAST_LIO_LC): FAST-LIO2 + SC-A-LOAM based SLAM
      + [FAST\_LIO\_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM): FAST-LIO2 + ScanContext based SLAM
      + [FAST\_LIO\_SAM](https://github.com/kahowang/FAST_LIO_SAM): FAST-LIO2 + LIO-SAM
  + Note2: main code (PGO) is modularized and hence can be combined with any other LIO / LO
      + This repo is to learn GTSAM myself\!
      + and as GTSAM tutorial for beginners - [GTSAM 튜토리얼 한글 포스팅](https://engcang.github.io/2023/07/15/gtsam_tutorial.html)

## Note

  + For better loop-detection and transform calculation, [FAST-LIO-SAM-QN](https://github.com/engcang/FAST-LIO-SAM-QN) is also coded and opened.
      + It adopts [Quatro](https://github.com/url-kaist/Quatro) - fast, accurate and robust global registration which provides great initial guess of transform
      + and [Nano-GICP](https://github.com/vectr-ucla/direct_lidar_odometry) - fast and accurate ICP combining [FastGICP](https://github.com/SMRT-AIST/fast_gicp) + [NanoFLANN](https://github.com/jlblancoc/nanoflann)

-----

## Dependencies

Before you begin, ensure you have **ROS** installed (e.g., Noetic on Ubuntu 20.04). ROS provides essential packages like `Eigen` and `PCL`. The following dependencies must be installed manually.

### 1\. System Libraries & Build Tools

Install `catkin_tools`, `libgeographic-dev`, and other tools needed for building the dependencies.

```shell
sudo apt-get update
sudo apt-get install -y python3-catkin-tools libgeographic-dev git cmake build-essential
```

### 2\. GTSAM (Version 4.1.1)

This project relies on GTSAM for graph optimization.

```shell
cd ~
wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
unzip gtsam.zip
cd gtsam-4.1.1/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j$(nproc)
sudo make install
```

### 3\. nlohmann/json

This JSON library is a required dependency.

```shell
cd ~
git clone https://github.com/nlohmann/json.git
cd json
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

-----

## How to build

Follow these steps to create your workspace, clone the repository with its submodules, and build the project.

1.  **Create and navigate to your catkin workspace:**

    ```shell
    mkdir -p ~/isds_slam_ws/src
    cd ~/isds_slam_ws/src
    ```

2.  **Clone the repository:**
    Use the `--recursive` flag to ensure submodules like `FAST-LIO` are downloaded correctly.

    ```shell
    git clone https://github.com/Kodifly/FAST-LIO-SAM_GPS.git --recursive
    ```

3.  **Build the project:**
    Navigate to the workspace root and use `catkin build`.

    ```shell
    cd ~/isds_slam_ws/
    catkin build -DCMAKE_BUILD_TYPE=Release
    ```

4.  **Source the environment:**
    After the build is complete, source the new setup file.

    ```shell
    source devel/setup.bash
    ```

    **Tip:** Add this command to your `~/.bashrc` to avoid running it in every new terminal:

    ```shell
    echo "source ~/isds_slam_ws/devel/setup.bash" >> ~/.bashrc
    ```

-----

## How to run

  + Then run (remember to change config files in `third_party/FAST_LIO` if needed):
    ```shell
    roslaunch fast_lio_sam run.launch lidar:=ouster
    roslaunch fast_lio_sam run.launch lidar:=velodyne
    roslaunch fast_lio_sam run.launch lidar:=livox
    ```

<!-- end list -->

  - In particular, we provide a preset launch option for specific datasets:
    ```shell
    roslaunch fast_lio_sam run.launch lidar:=kitti
    roslaunch fast_lio_sam run.launch lidar:=mulran
    roslaunch fast_lio_sam run.launch lidar:=newer-college20
    ```

-----

## How to save slam map

  + Send a `save_dir` ROS topic message to the node:
    ```shell
    rostopic pub /save_dir std_msgs/String "/path/to/save/your/map.pcd"
    ```

-----

### Structure

  + **odomPcdCallback**
      + Publishes real-time pose in the corrected frame.
      + Detects keyframes. If a keyframe is found, it's added to the pose graph and saved to a queue.
      + Performs pose graph optimization using iSAM2.
  + **loopTimerFunc**
      + Processes a saved keyframe from the queue.
      + Detects loop closures. If a loop is found, a factor is added to the pose graph.
  + **visTimerFunc**
      + Visualizes all components. **(Note: The global map is only visualized once. Uncheck/re-check the `mapped_pcd` topic in RViz to refresh the view and save computational resources.)**

-----

## License

\<a rel="license" href="[http://creativecommons.org/licenses/by-nc-sa/4.0/](http://creativecommons.org/licenses/by-nc-sa/4.0/)"\>\<img alt="Creative Commons License" style="border-width:0" src="[https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png](https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png)" /\>\</a\>

This work is licensed under a [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-nc-sa/4.0/).