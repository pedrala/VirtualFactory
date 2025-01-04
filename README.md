## Overview
Use the Aruko marker to calculate the relative coordinates and move Turtlebot3 to the corresponding location. Recognize a small box as an object with yolo8, pick it up with the manipulator, and place it on the conveyor belt. Pick up the box again at the end of the conveyor belt and move it to the unloading area. 

## Capture

### GUI 로 Login 창을 띄운 모습
<p align="center">
  <img src="capture/login.png" alt="Login" width="500"> 
</p>

#### Login 입력정보
E-mail: jsh10198@naver.com,   Password: password

### Login 후 World View 로 Aruco 마커를 감지하는 모습
<p align="center">
 <img src="capture/aruco_detecting.png" alt="Aruco Detecting" width="800">
</p>

### 기준마커(ID:2)를 통해 목표마커(ID:15)의 좌표를 계산하는 모습
벽에 수직으로 붙은 아루코 마커는 z, x 값을 사용하여 목표좌표의 상대거리(tvecs)및 방향(rvecs)을 계산합니다.
<p align="center">
 <img src="capture/result_04_44_28.jpg" alt="Aruco Detecting" width="800">
</p>

### Virtual Factory 전체 조망도
<p align="center">
 <img src="capture/VirtualFactory.jpeg" alt="Virtual Factory.jpeg" width="800">
</p>

### Turtlebot3 Manipulating Video
[Turtlebot3 Manipulating Demo Video](capture/manipulating.MOV)

### Pick and Place On Conveyorbelt Demo Video
[Pick and Place On Conveyorbelt Demo Video](capture/PickandPlaceOnConveyorbelt.mp4)

## How to execute

GUI 서버 실행시키기
```console
ros2 run virtual_factory gui
```

Aruco 마커 탐지 노드 실행시키기
```console
ros2 run virtual_factory aruco
```

AMR 실행시키기
```console
ros2 run virtual_factory amr
```

Manipulator 실행시키기
```console
ros2 run virtual_factory manipulator
```
