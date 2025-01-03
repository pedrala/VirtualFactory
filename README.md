## Overview
Use the Aruko marker to calculate the relative coordinates and move Turtlebot3 to the corresponding location. Recognize a small box as an object with yolo8, pick it up with the manipulator, and place it on the conveyor belt. Pick up the box again at the end of the conveyor belt and move it to the unloading area. 

## Capture

### GUI 로 Login 창을 띄운 모습
<p align="center">
  <img src="capture/login.png" alt="Login" width="500"> 
</p>

### Login 후 World View 로 Aruco 마커를 감지하는 모습
<p align="center">
 <img src="capture/aruco_detecting.png" alt="Aruco Detecting" width="800">
</p>

### Turtlebot3 Manipulating video
[turtlebot3 manipulating video](capture/manipulating.mp4)

## How to execute

GUI 서버 실행시키기
```console
ros2 run virtual_factory gui
```
Login 창에 다음을 입력
E-mail: jsh10198@naver.com,   Password: password

Aruco 마커 탐지 노드 실행시키기
```console
ros2 run virtual_factory aruco

AMR 실행시키기
```console
ros2 run virtual_factory amr
```

Manipulator 실행시키기
```console
ros2 run virtual_factory manipulator
```
