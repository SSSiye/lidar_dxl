# 장애물 회피 알고리즘
스캔영상(로봇중심으로 반경1m내의 장애물만 표시)을 만들고 영상의 전방 180도 영역에서 장애물을 위치를 인식 
<br/>-> 좌측 ½영역에서 최단거리 장애물 검출
<br/>-> 우측1/2영역에서 최단거리 장애물 검출
<br/>-> 2개의 최단거리 장애물의 중앙방향을 구함
<br/>-> 로봇 정면방향과 각도 차이를 에러(부호있는 정수)로 정의함
<br/><br/>

## ⚙️시스템 구성
Jetson Nano (pub.cpp)<br/>
├ mp4 영상 파일 재생<br/>
├ 프레임 단위로 이미지 추출<br/>
└─ ROS2 토픽 (/video_frames) 퍼블리시<br/>

WSL (sub_wsl.cpp)<br/>
├ ROS2 토픽 수신<br/>
├ OpenCV로 디코딩<br/>
└ 영상 이진화, 객체 중심 좌표 추출, 진행 방향 시각화<br/>
<br/>

## 📋주요 기능
pub.cpp: Jetson Nano에서 .mp4 영상 파일을 불러와 프레임별로 ROS2 이미지 메시지로 퍼블리시
<br/><br/>
sub_wsl.cpp: WSL 환경에서 해당 토픽을 구독하여 이미지 수신/ OpenCV를 활용한 영상처리 (Otsu 이진화 → 객체 중심 좌표 추출 → 좌우 방향 판단 → 화살표로 시각화)


<br/>

![이미지 2025  7  3  오후 1 43](https://github.com/user-attachments/assets/bc6b2b0e-25ee-48d2-b69e-f72cb4a86a68)

<br/>
참고유튜브 링크: https://www.youtube.com/watch?v=HvWfm4Xtzbs
<br/>
![이미지 2025  7  3  오후 1 47](https://github.com/user-attachments/assets/a574db22-541b-42ce-9392-30ec2be60aea)


