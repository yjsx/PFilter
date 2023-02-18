# PFilter
## Fast LOAM (Lidar Odometry And Mapping)
This work is the official implementation of "PFilter: Building Persistent Maps through Feature Filtering for Fast and Accurate LiDAR-based SLAM", which saves 20.9% processing time per frame and improves the
accuracy by 9.4% than FLOAM.

This code is modified from [FLOAM](https://github.com/wh200720041/floam).

#### [[Demo Video](https://youtu.be/Kx5lWBIlFLA)] [[Preprint Paper](https://arxiv.org/abs/2208.14848)] 

**Modifier:** [Yifan Duan](http://yjsx.top), University of Science and Technology of China, China

## 1. The map compared with FLOAM
<p align='center'>
<a href="https://youtu.be/Kx5lWBIlFLA">
<img width="65%" src="/img/example.gif"/>
</a>
</p>

## 2. Evaluation
### 2.1. run time  evaluation on KITTI dataset
Platform: Intel® Core™ i7-8700 CPU @ 3.20GHz 
| Dataset                                      | ALOAM                      | FLOAM                  |
|----------------------------------------------|----------------------------|------------------------|
| `KITTI`                                      | 151ms                      | 59ms                   |

Localization error:
| KITTI sequence             | FLOAM                      | PFilter                  |
|----------------------------------------------|----------------------------|------------------------|
| `00`                          | 0.7007%                      | 0.6208%                  |
| `01`                          | 1.9504%                      | 1.8055%                  |
| `02`                          | 0.9549%                      | 0.8042%                  |
| `03`                          | 0.9549%                      | 0.8941%                  |
| `04`                          | 0.6875%                      | 0.6420%                  |
| `05`                          | 0.4910%                      | 0.5085%                  |
| `06`                          | 0.5435%                      | 0.4906%                  |
| `07`                          | 0.4159%                      | 0.3740%                  |
| `08`                          | 0.9349%                      | 0.9326%                  |
| `09`                          | 0.7031%                      | 0.6242%                  |
| `10`                          | 1.0257%                      | 0.9206%                  |
|`all`                          | 0.8511%                      | 0.7833%                  |

| KITTI sequence             | FLOAM                      | PFilter                  ||||
|-----------------|--------|---------------|----|-----------------|--------|---------------|
| `00`                          | 0.7007%                      | 0.6208%                  | `06`                          | 0.5435%                      | 0.4906%                  |
| `01`                          | 1.9504%                      | 1.8055%                  | `07`                          | 0.4159%                      | 0.3740%                  |
| `02`                          | 0.9549%                      | 0.8042%                  | `08`                          | 0.9349%                      | 0.9326%                  |
| `03`                          | 0.9549%                      | 0.8941%                  | `09`                          | 0.7031%                      | 0.6242%                  |
| `04`                          | 0.6875%                      | 0.6420%                  | `10`                          | 1.0257%                      | 0.9206%                  |
| `05`                          | 0.4910%                      | 0.5085%                  |`all`                          | 0.8511%                      | 0.7833%                  |






### 2.2. localization result
<p align='center'>
<img width="65%" src="/img/kitti_example.gif"/>
</p>

### 2.3. mapping result
<p align='center'>
<a href="https://youtu.be/w_R0JAymOSs">
<img width="65%" src="/img/floam_mapping.gif"/>
</a>
</p>

## 3. Prerequisites
### 3.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 3.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 3.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

### 3.4. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

## 4. Build 
### 4.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/wh200720041/floam.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
### 4.2 Download test rosbag
Download [KITTI sequence 05](https://drive.google.com/file/d/1eyO0Io3lX2z-yYsfGHawMKZa5Z0uYJ0W/view?usp=sharing) or [KITTI sequence 07](https://drive.google.com/file/d/1_qUfwUw88rEKitUpt1kjswv7Cv4GPs0b/view?usp=sharing)

Unzip compressed file 2011_09_30_0018.zip. If your system does not have unzip. please install unzip by 
```
sudo apt-get install unzip 
```

And this may take a few minutes to unzip the file
```
	cd ~/Downloads
	unzip ~/Downloads/2011_09_30_0018.zip
```

### 4.3 Launch ROS
```
    roslaunch floam floam.launch
```
if you would like to create the map at the same time, you can run (more cpu cost)
```
    roslaunch floam floam_mapping.launch
```
If the mapping process is slow, you may wish to change the rosbag speed by replacing "--clock -r 0.5" with "--clock -r 0.2" in your launch file, or you can change the map publish frequency manually (default is 10 Hz)


## 5. Test on other sequence
To generate rosbag file of kitti dataset, you may use the tools provided by 
[kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) or [kitti2bag](https://github.com/tomas789/kitti2bag) 

## 6. Test on Velodyne VLP-16 or HDL-32
You may wish to test FLOAM on your own platform and sensor such as VLP-16
You can install the velodyne sensor driver by 
```
sudo apt-get install ros-melodic-velodyne-pointcloud
```
launch floam for your own velodyne sensor
```
    roslaunch floam floam_velodyne.launch
```
If you are using HDL-32 or other sensor, please change the scan_line in the launch file 


## 7.Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).


## 8. Citation
If you use this work for your research, you may want to cite
```
@inproceedings{duan2022pfilter,
  title={PFilter: Building Persistent Maps through Feature Filtering for Fast and Accurate LiDAR-based SLAM},
  author={Duan, Yifan and Peng, Jie and Zhang, Yu and Ji, Jianmin and Zhang, Yanyong},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={11087--11093},
  year={2022},
  organization={IEEE}
}
```
