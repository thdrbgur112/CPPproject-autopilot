#FILe TREE

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
            └── cnn_node.py             (3번 노드: 전처리 이미지 수신 -> Waypoint 좌표 계산)\


#DATA FLOW
autonomous_vehicle.cpp 
