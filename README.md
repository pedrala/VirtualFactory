## Overview
Use the ArUco marker to calculate the relative coordinates and move Turtlebot3 to the corresponding location. Recognize a small box as an object with yolo8, pick it up with the manipulator, and place it on the conveyor belt. Pick up the box again at the end of the conveyor belt and move it to the unloading area. 

## Capture

### GUI: Login, MainApplication Window  
<p align="center">
  <img src="capture/login_vf.png" alt="Login" width="800"> 
</p>

#### Login Account
ID: rokey,   Password: 1234

<p align="center">
  <img src="capture/loggedGUI.png" alt="Login" width="800"> 
</p>

#### How to operate
When an employee presses the play button, the Turtlebot issues Aruco markers in the order of Aruco marker 4->7->15 and moves to the corresponding marker locations. After arriving at each location, it stops and saves and displays the status as "arrived at marker 2".

An employee can also manually move Turtlebot to the desired marker by selecting the Aruko marker directly from Dropbox.

When an employee selects a job from dropbox, a 'target_counts' ros topic is published, which is received by the manipulator and the job is executed.

Below is job list which manipulator must conduct when employee selects.

##### Job1 : Red*2,   Blue*1,
##### Job2 : Red*1,   Blue*2,
##### Job3 : Red*1,

In an emergency situation, if an employee presses the red Stop button, the conveyor belt, AMR, and Manipulator will all stop.

### Running the ArUco node in the terminal
<p align="center">
 <img src="capture/aruco_detecting.png" alt="ArUco Detecting" width="800">
</p>

### Calculating the coordinates of the target marker (ID:15) using the base marker (ID:2)
(기준마커(ID:2)를 통해 목표마커(ID:15)의 좌표 계산하기)
벽에 수직으로 붙은 ArUco 마커의 z, x 값을 이용하여 목표좌표의 상대거리(tvecs)및 방향(rvecs)을 계산합니다.

```python

# 기준 마커의 회전 행렬 계산
R_base, _ = cv2.Rodrigues(base_rvec)  # 기준 마커의 회전 행렬
R_base_inv = np.linalg.inv(R_base)    # 회전 행렬의 역행렬

# 타겟 마커의 이동 벡터를 기준 좌표계로 변환
tvec_relative = R_base_inv @ (target_tvec.T - base_tvec.T)

# z, x 성분만 추출
tvec_relative_xz = tvec_relative[[0, 2]]  # z, x 좌표 (상대 이동)
distance = np.linalg.norm(tvec_relative_xz)  # 거리 계산
direction_vector = tvec_relative_xz / distance  # 방향 벡터 (단위 벡터)

# z, x를 기반으로 각도 계산 (atan2 사용)
angle_rad = np.arctan2(tvec_relative_xz[1], tvec_relative_xz[0])  # 라디안 단위
```
<p align="center">
 <img src="capture/result_04_44_28.jpg" alt="Aruco Detecting" width="800">
</p>

### Virtual Factory from World Eye
<p align="center">
 <img src="capture/VirtualFactory.jpeg" alt="Virtual Factory.jpeg" width="800">
</p>

### Turtlebot3 Manipulating Video
[Turtlebot3 Manipulating Demo Video](capture/manipulating.MOV)

### Pick and Place On Conveyorbelt Demo Video
[Pick and Place On Conveyorbelt Demo Video](capture/PickandPlaceOnConveyorbelt.mp4)

## How to execute

GUI Server Node
```console
ros2 run virtual_factory gui
```

Aruco Marker Detecting Node
```console
ros2 run virtual_factory aruco
```

AMR Node
```console
ros2 run virtual_factory amr
```

Manipulator Node
```console
ros2 run virtual_factory manipulator
```
