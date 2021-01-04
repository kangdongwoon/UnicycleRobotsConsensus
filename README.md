#  Multi Agent System Project

## Project : UnicycleTurtleBot3 Consensus Problem (feat.BARAM)  
  
참여 기간 : 2020.08 ~ 2020.11

## Main Feature  
1. Gazebo를 이용한 Simulation
2. Turtlebot Localization with Camera
  
## Using Platform 
1. ROS
2. Gazebo
3. Turtlebot3

## ROS PACKAGES
1. turtlebot3 [kinetic-devel]
2. turtlebot3_msgs [master]
3. turtlebot3_simulations [master]
 - http://wiki.ros.org/Robots/TurtleBot
4. usb_cam [develop]
 - https://wiki.ros.org/usb_cam
5. ar_track_alvar [kinetic-devel]
 - http://wiki.ros.org/ar_track_alvar
6. mas_ctrl {MY OWN}

## Reference Paper  
Formation stabilization of unicycle-type mobile robots via consensus algorithm, 2010, 김홍근, 심형보, 백주훈 prof.
  
## System Architecture
<img src="https://user-images.githubusercontent.com/52377778/103460258-39603800-4d58-11eb-80c9-fb5eb60af130.PNG" width="800" height="500" />  

## Project GIF  
![터틀봇](https://user-images.githubusercontent.com/52377778/103460458-a6c09880-4d59-11eb-879e-18587f697bae.gif)

## Developing Tips
1. Gazebo 환경과 ROS 환경의 Time Sync 필요!
2. AR_Marker를 사용할 때, Marker는 흰색 바탕이 적당히 있어야 인식이 됨! 

