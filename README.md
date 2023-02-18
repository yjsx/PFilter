# PFilter
+ This work is the official implementation of "PFilter: Building Persistent Maps through Feature Filtering for Fast and Accurate LiDAR-based SLAM", which saves 20.9% processing time per frame and improves the
accuracy by 9.4% than FLOAM.

+ This code is modified from [FLOAM](https://github.com/wh200720041/floam).

#### [[Demo Video](https://youtu.be/Kx5lWBIlFLA)] [[Preprint Paper](https://arxiv.org/abs/2208.14848)] 

**Modifier:** Yifan Duan, University of Science and Technology of China, China

## 1. Demo
+ Edge map and surface map on KITTI dataset:
<p align='center'>
<a href="https://youtu.be/Kx5lWBIlFLA">
<img width="65%" src="/img/example.gif" alt="in KITTI"/>
</a>
</p>

+ Map in USTC with 32-lines LiDAR:
<p align='center'>
<a href="https://youtu.be/Kx5lWBIlFLA">
<img width="65%" src="/img/32.gif" alt="in USTC campus"/>
</a>
</p>

## 2. Prerequisites and build
+ It's the same environment as FLOAM, please see [FLOAM](https://github.com/wh200720041/floam) for the build details.

## 3. Usage
+ To test on KITTI dataset and get the same ATEs as the shown in the Sec. 4:
  + ```roslaunch pfilter pfilter_kitti.launch bag_filename:=/YOUR/BAG/PATH``` 
+ To test on other data:
  + ```roslaunch pfilter pfilter.launch bag_filename:=/YOUR/BAG/PATH k_new:=xx theta_p:=xx theta_max:=xx topic:=xx```
  + The xx should be replaced with suitable value. 
    + "*k_new* " is Int $\in [0,\infty)$, but [0,5] are recommanded. 
    + "*theta_p* " is Float $\in [0,\infty)$, but [0,3] are recommanded.
    + "*theta_max* " is Int $\in [0,255]$.
    + "topic" is the ros topic of LIDAR points.

+ Some notes:
  + The parameters "*k_new*", "*theta_p*" and "*theta_max*" can control how hard  **PFilter** works, which means how many noise points will be removed from the map. 
  + *k_new* $\downarrow$, *theta_p* $\uparrow$, *theta_max* $\uparrow$ means more points are removed. 
  + Generally speaking, if the robot is moving slow, I usually let **PFilter** remove more points to keep the map tiny. For example, I use a low-speed unmanned vehicle on campus, equipped with a 32-lines LiDAR for mapping, and the parameters I use are "0, 1, 200". In comparison, the parameters used on KITTI dataset are "0, 0.4, 75".
  + To get the performance of origin FLOAM, you can set the parameters to "0,0,0". 

## 4. Evluation
| KITTI sequence   | FLOAM     | PFilter   |KITTI sequence   | FLOAM    | PFilter   |
|--------|--------|--------|--------|--------|--------|
| `00`                          | 0.7007%                      | 0.6208%                  | `06`                          | 0.5435%                      | 0.4906%                  |
| `01`                          | 1.9504%                      | 1.8055%                  | `07`                          | 0.4159%                      | 0.3740%                  |
| `02`                          | 0.9549%                      | 0.8042%                  | `08`                          | 0.9349%                      | 0.9326%                  |
| `03`                          | 0.9549%                      | 0.8941%                  | `09`                          | 0.7031%                      | 0.6242%                  |
| `04`                          | 0.6875%                      | 0.6420%                  | `10`                          | 1.0257%                      | 0.9206%                  |
| `05`                          | 0.4910%                      | 0.5085%                  |`all`                          | 0.8511%                      | 0.7833%                  |

+ The tool for evaluation is [KITTI odometry evaluation tool](https://github.com/LeoQLi/KITTI_odometry_evaluation_tool.git).



## 5.Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and [F-LOAM](https://github.com/wh200720041/floam).


## 6. Citation
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
