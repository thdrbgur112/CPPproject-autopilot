# Webots 기반 차선 유지 제어 시뮬레이션 프로젝트

---
<br>


## 1. 프로젝트 개요 (Project Overview)
본 프로젝트는 **Webots 시뮬레이터 환경**에서 차량의 비전 센서만을 사용하여 안정적인 차선 유지 제어를 구현하는 것을 목표로 합니다.

카메라로 획득한 전방 도로 영상을 CNN기반 모델로 분석하여 차선을 실시간으로 인식하며, 차량 동역학 제어 알고리즘과 ROS2 통신 아키텍처를 결합하여 Webots 시뮬레이션 내에서 안정적으로 주행할 수 있는 시스템을 구축합니다.

* **개발 기간:** 2026.03 ~ 진행 중 (2026.06 완료예정)

---

<br>
<br>

## 2. 팀원 소개 및 역할 (Team Members)
| 이름 | 소속 | 역할 및 담당 기능 |
| :---: | :---: | :--- |
| **김민호** | 숭실대학교 AI소프트웨어학부 | 팀장, 차량 동역학 기반 제어 알고리즘 구현 |
| **박경수** | 숭실대학교 AI소프트웨어학부 | 데이터셋 수집 및 기본 전처리 |
| **김철현** | 숭실대학교 AI소프트웨어학부 | CNN 기반 차선 인식 모델 학습 및 검증 |
| **송규혁** | 숭실대학교 AI소프트웨어학부 | 이미지 전처리 , PIPELINE에 따른 각 코드 수정후 ROS2를 이용해 통합 |

---

<br>
<br>

## 3. 기술 스택 (Tech Stack)

### Environment & Simulation
<img src="https://img.shields.io/badge/Webots-FF0000?style=for-the-badge&logo=Webots&logoColor=white"/> <img src="https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=Ubuntu&logoColor=white"/>

### Programming Languages
<img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=Python&logoColor=white"/> <img src="https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=C%2B%2B&logoColor=white"/>

### Libraries & Frameworks
<img src="https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ROS&logoColor=white"/> <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=OpenCV&logoColor=white"/> <img src="https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=PyTorch&logoColor=white"/> <img src="https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=NumPy&logoColor=white"/>

---

<br>
<br>


# 4. FILE TREE

```
~/ros2_ws/
├── build/      (빌드하면 자동 생성되는 찌꺼기 폴더 - 꼬이면 지움)
├── install/    (빌드 완성본이 들어가는 폴더)
├── log/        (로그 폴더)
└── src/
    └── auto_pkg/               ✅ 우리가 만드는 핵심 패키지
        ├── CMakeLists.txt      [필수] 어떤 파일을 어떻게 빌드할지 적어두는 설명서
        ├── package.xml         [필수] 패키지 의존성 정보
        │
        ├── src/                ✅ C++ 소스 코드 방
        │   ├── autonomous_vehicle.cpp  (1번 노드: Webots 카메라 전송 & 모터 구동)
        │   ├── cv.cpp                  (2번 노드: 이미지 전처리)
        │   └── mpc_node.cpp            (4번 노드: Waypoint 수신 후 조향/속도 계산)
        │
        └── scripts/            ✅ Python 소스 코드 방 (CNN은 주로 파이썬 사용)
            └── cnn_node.py             (3번 노드: 전처리 이미지 수신 -> Waypoint 좌표 계산)
```
<br>
<br>
# 5. PIPELINE 및 DATA FLOW

1. autonomous_vehicle.cpp ( Webots node)
   -- 출력 토픽 : /image_raw

2. cv.cpp (CV node)
   -- 출력 토픽 : /image_porcessed

3. cnn_node.py (CNN node)
   -- 풀력 토픽 : /waypoints

4. mpc_node.cpp (MPC node)
   -- /cmd_vel ( 조향 및 속도 명령)

5. autonomous_vehicle.cpp (귀환)
   -- /cmd_vel 명령을 받아서 webots 차량의 바퀴와 핸들모터를 돌림

	 
<br>
<br>
