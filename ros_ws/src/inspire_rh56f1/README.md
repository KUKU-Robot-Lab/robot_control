# inspire_rh56f1

Inspire RH56F1 손(손당 액추에이터 6개)의 ROS 2 Humble 드라이버. 손 하나가 자기 포트 하나를 쓰고, 손마다 벤더 드라이버 프로세스를 하나씩 띄운다.

## 출처

| 항목 | 값 |
|---|---|
| 원본 | `sim2real/robot/vendor/inspire_ws/src` (KUKU-Robot-Lab/sim2real) |
| 복사 시점 sim2real HEAD | `3465c5b69361b255bad3f94051522f2b4e1a301f` |
| 원본 경로를 마지막으로 바꾼 커밋 | `0d231c13daf5f786205b94035112ba87b3e72212` |
| 복사일 | 2026-09-29 |
| 메타데이터 | `robot_control/vendor_metadata/inspire/UPSTREAM.yaml` |

`vendor/` 는 원본과 바이트 단위로 같다(`local_modifications: []`). 빠진 것은 비 ROS 예제용 최상위 `CMakeLists.txt`(colcon 이 패키지로 잡는다), 미리 빌드된 `serial_hand_control_node`, `examples/`, `config/`, docx·pdf 매뉴얼, RH5DG2 프로토콜 문서다. 대조:

```bash
cd robot_control
python3 -c "from pathlib import Path; from tools.verify_vendor_snapshot import verify_snapshot; \
print(verify_snapshot(Path('vendor_metadata/inspire/UPSTREAM.yaml'), \
Path('../sim2real/robot/vendor/inspire_ws/src'), Path('ros_ws/src/inspire_rh56f1/vendor')))"
# [] 이면 일치
```

## 구성

| 패키지 | 위치 | 역할 |
|---|---|---|
| `rh56f1_interfaces` | `vendor/ros2/src/interfaces/RH56F1` | 메시지·서비스 (벤더 원본) |
| `rh5dg2_interfaces` | `vendor/ros2/src/interfaces/RH5DG2` | 드라이버 CMake 가 `find_package` 하므로 같이 빌드 (벤더 원본) |
| `inspire_control_ros2` | `vendor/ros2/src/driver` | 노드 `inspire_control_node` (벤더 원본, `../../../include`·`../../../src` 코어 소스를 같이 컴파일) |
| `rh56f1_driver` | `rh56f1_driver/` | 이 저장소 몫. 손별 launch 와 YAML 생성 |

## 빌드

```bash
cd robot_control/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select rh56f1_interfaces rh5dg2_interfaces inspire_control_ros2 rh56f1_driver
```

시스템 의존: `libboost-system-dev libboost-thread-dev libyaml-cpp-dev libspdlog-dev`. 시리얼 접근은 사용자가 `dialout` 그룹에 있어야 한다.

## 실행

벤더 노드는 ROS 파라미터가 아니라 YAML 두 개(장치 설정, 컨트롤러 설정)를 읽는다. `rh56f1_driver.launch.py` 가 인자로 두 파일을 `<runtime_dir>/<side>/` 에 쓰고(기본 `~/.ros/rh56f1/right/`) 그 파일로 노드를 띄운다. 벤더 로그 파일도 같은 곳(`hand_right.log`)에 쌓인다.

```bash
source robot_control/ros_ws/install/setup.bash

# RS485 (USB-RS485 어댑터, 손마다 자기 포트)
ros2 launch rh56f1_driver rh56f1_right_driver.launch.py transport:=rs485 port:=/dev/ttyUSB0 baud:=115200 hand_id:=1
ros2 launch rh56f1_driver rh56f1_left_driver.launch.py  transport:=rs485 port:=/dev/ttyUSB1 baud:=115200 hand_id:=1

# CANFD (USB-CANFD 변환기가 만든 시리얼 포트, 아래 "벤더 한계" 참고)
ros2 launch rh56f1_driver rh56f1_right_driver.launch.py transport:=canfd port:=/dev/ttyACM0 baud:=115200 hand_id:=1
ros2 launch rh56f1_driver rh56f1_left_driver.launch.py  transport:=canfd port:=/dev/ttyACM1 baud:=115200 hand_id:=1
```

기본 포트(`/dev/ttyUSB0` 오른손, `/dev/ttyUSB1` 왼손)와 `hand_id:=1` 은 자리표시자다. arm4090 에서 확인한 값으로 넘기고, 가능하면 `/dev/serial/by-id/...` 를 쓴다(USB 순서가 바뀌어도 좌우가 뒤바뀌지 않는다).

| 인자 | 기본 | 뜻 |
|---|---|---|
| `transport` | `rs485` | `rs485` -> `RH56F1_485`, `canfd` -> `RH56F1_canfd` |
| `port` | 오른 `/dev/ttyUSB0`, 왼 `/dev/ttyUSB1` | 이 손의 시리얼 장치 |
| `baud` | `115200` | 시리얼 보레이트(손 공장값 115200) |
| `hand_id` | `1` | 손의 Hand_ID 레지스터(1..254) |
| `update_rate` | `50` | 읽기·발행 주기 Hz (벤더 기본) |
| `enable_touch` | `true` | 촉각 68 byte 블록을 매 주기 읽기 |
| `enable_current` | `false` | 전류 읽기 전용 토픽 추가 |
| `admin_services` | `false` | 설정 쓰기·동작 시퀀스 서비스 노출 |
| `log_level` | `WARN` | 벤더 spdlog 레벨 |
| `runtime_dir` | `~/.ros/rh56f1` | 생성 YAML·로그 위치 |

인자가 계약 밖이면(`transport:=can`, `hand_id:=0` 등) launch 가 노드를 띄우기 전에 실패한다.

## 토픽과 서비스 (오른손 기준, 왼손은 `/hand_left/...`)

| 이름 | 방향 | 타입 | 단위·범위 (RH56F1 매뉴얼 V1.0.0) |
|---|---|---|---|
| `/hand_right/angle_set` | 구독 | `rh56f1_interfaces/msg/SetAngle1` | 0.1 deg. 네 손가락 900..1740, 엄지 굽힘 1100..1350, 엄지 회전 약 600..1750. 값이 클수록 펴짐. `-1` 은 그 축을 움직이지 않음 |
| `/hand_right/angle_actual` | 발행 | `rh56f1_interfaces/msg/GetAngleAct1` | 0.1 deg, 위와 같은 범위. `joint_names` 에 정준 이름이 슬롯 순서로 들어감 |
| `/hand_right/force_set` | 구독 | `rh56f1_interfaces/msg/SetForce1` | 힘 제어 임계값, g (센서 위치 기준, 손끝의 약 10배) |
| `/hand_right/force_actual` | 발행 | `rh56f1_interfaces/msg/GetForceAct1` | g |
| `/hand_right/speed_set` | 구독 | `rh56f1_interfaces/msg/SetSpeed1` | 0..4000 무차원 (2000 = 무부하 전 행정 1000 ms) |
| `/hand_right/touch_data` | 발행 | `rh56f1_interfaces/msg/TouchData1` | 손가락 5개(새끼, 약지, 중지, 검지, 엄지)의 법선·접선·각도·근접, 손바닥 9칸. 원시값 |
| `/hand_right/current_actual` | 발행 (`enable_current:=true`) | `rh56f1_interfaces/msg/GetCurrentAct1` | mA |
| `/hand_right/get_errorCode` | 서비스 | `rh56f1_interfaces/srv/Geterror` | 비트: 0 스톨, 1 과온, 2 과전류, 3 모터 이상, 4 통신 |
| `/hand_right/get_status` | 서비스 | `rh56f1_interfaces/srv/Getstatus` | 0 펴는 중, 1 쥐는 중, 2 위치 도달, 3 힘 도달, 5 전류 보호, 6 스톨, 7 고장 |
| `/hand_right/get_temp` | 서비스 | `rh56f1_interfaces/srv/Gettemp` | 섭씨 |
| `/hand_right/set_clearError` | 서비스 | `rh56f1_interfaces/srv/Setclearerror` | 오류 해제 |
| `/hand_right/set_pause`, `/hand_right/set_stop` | 서비스 | `Setpause`, `Setstop` | 일시정지, 비상정지 레지스터 |

오류·온도·상태는 벤더 노드가 토픽으로 내지 않고 서비스로만 준다(벤더 제어 루프는 angle·force·current·touch 만 발행한다).

`admin_services:=true` 일 때만 추가: `set_id`, `set_baudRate`, `set_resetPara`, `set_gestureForceClb`, `set_defaultSpeed`, `set_defaultForceSet`(이 둘은 쓰고 나서 `save` 로 플래시 기록), `set_mode`, `set_actionSeqIndex`(쓰고 나서 `actionSeqRun=1` 을 써서 동작 시퀀스를 실행한다, 곧 손이 움직인다). `set_angle` 서비스와 `set_actionLibraryIndex` 는 넣지 않았다(RH56F1 레지스터 표에 actionLibraryIndex 가 없다).

모든 메시지의 `hand_id` 는 0 이거나 이 노드의 Hand_ID 와 같아야 받아들여진다.

## 슬롯 순서와 관절 이름

6값 배열(angle, force, speed, current, error, status, temp)의 인덱스 0..5:

| 슬롯 | 벤더 | 정준 이름 (오른 / 왼) | profile 범위 (rad) |
|---|---|---|---|
| 0 | 새끼 | `r_hj_pinky_1` / `l_hj_pinky_1` | 0..1.52856 |
| 1 | 약지 | `r_hj_ring_1` / `l_hj_ring_1` | 0..1.52856 |
| 2 | 중지 | `r_hj_middle_1` / `l_hj_middle_1` | 0..1.52856 |
| 3 | 검지 | `r_hj_index_1` / `l_hj_index_1` | 0..1.52856 |
| 4 | 엄지 굽힘 | `r_hj_thumb_2` / `l_hj_thumb_2` | 0..0.47456 |
| 5 | 엄지 회전 | `r_hj_thumb_1` / `l_hj_thumb_1` | 0..2.0944 |

엄지 두 축의 대응은 이동 범위로 맞춘 추정이다. 엄지 굽힘은 약 25 deg(110..135) 로 `thumb_2`(27.2 deg) 와, 엄지 회전은 약 115 deg 로 `thumb_1`(120 deg) 과 비슷하다. 실기에서 확인해야 한다. 0.1 deg 에서 rad 로의 변환(펴짐 = 0 rad 기준, 부호, 영점)은 이 패키지가 하지 않는다.

## 시작할 때 드라이버가 하는 일

- 설정 파일을 읽고 시리얼 포트를 연다(8N1, 흐름 제어 없음).
- 주기 타이머로 읽기 요청(명령 0x11)만 보낸다: `angleAct`(1064), `forceAct`(1070), 켜진 경우 `currentAct`(1076), `touchAct`(3000).
- 쓰기(명령 0x12)는 명령 토픽에 메시지가 오거나 set 서비스가 불릴 때만 한다. 시작 시 모드 설정, 오류 해제, 기본 자세 같은 쓰기는 없다. 이 launch 도 아무 것도 발행하지 않는다.
- 2026-09-29 개발 PC 에서 가상 터미널(pty)에 붙여 3 초 동안 나간 프레임을 모두 파싱해 확인했다: 0x11 읽기 113 개(1064, 1070, 3000), 0x12 쓰기 0 개. 하드웨어 장치는 열지 않았다.

주의: `SetAngle1` 을 기본값(전부 0)으로 보내면 범위 아래 값이라 손이 최대로 굽을 수 있다. 움직이지 않을 축은 `-1` 로 채운다.

## 벤더 한계

- 프로세스 하나가 포트 하나를 연다. 벤더 설정은 장치를 포트 이름으로 묶어서(`unordered_map<port, ...>`) 같은 포트에 손 두 개를 적으면 하나가 덮인다. 손마다 포트가 따로인 이 구성에서는 문제가 없다. 같은 포트를 두 프로세스가 열면 프레임이 섞이니 좌우 포트를 반드시 다르게 준다.
- CANFD 도 벤더 코드는 시리얼 포트(boost::asio serial_port)로만 말한다. `RH56F1_canfd` 는 485 와 같은 `EB 90` 프레임을 시리얼로 쓰고, 데이터 길이만 CAN FD 합법 길이(최대 64 byte)로 맞춘다. SocketCAN(`can0`) 경로는 없다. 따라서 시리얼 포트로 보이면서 이 프레임을 CAN FD 확장 프레임(29 bit ID: Hand_ID, 주소, 길이, 읽기/쓰기)으로 바꿔 주는 변환기가 필요하다. 매뉴얼의 CANFD 기본 비트레이트는 중재 1 Mbps, 데이터 5 Mbps 다.
- 쓰기 한 번마다 응답을 최대 25 ms 기다린다. 읽기도 응답이 없으면 레지스터마다 25 ms 를 쓴다. 손이 응답하지 않으면 주기가 50 Hz 에서 약 13 Hz 로 떨어지고 오류 로그가 매 주기 찍힌다(pty 시험에서 확인). 명령 토픽을 높은 주기로 보내면 읽기 주기가 같이 밀린다.
- 벤더 노드의 타이머와 구독 콜백은 같은 기본 콜백 그룹이라 버스 접근은 직렬화된다.

## arm4090 에서 확인할 것

1. 좌우 손의 실제 장치 경로(`ls -l /dev/serial/by-id/`)와 어느 쪽이 오른손인지.
2. 각 손의 Hand_ID 와 보레이트(공장값 1, 115200 로 가정).
3. CANFD 로 쓸 때 변환기 모델과 그 변환기가 벤더 `EB 90` 시리얼 프레임을 받아 주는지, 손의 CAN 비트레이트 설정.
4. 엄지 두 축의 슬롯 대응(슬롯 4 = 굽힘 = `thumb_2`, 슬롯 5 = 회전 = `thumb_1`)과 각 축의 펴짐·굽힘 방향.
5. 촉각 센서 유무와 `touch_version: 1`(정전식) 이 맞는지. 없으면 `enable_touch:=false`.
6. 50 Hz 에서 실제 `angle_actual` 주기(`ros2 topic hz`).
7. 드라이버 기동은 실기 명령이다. 기동과 첫 `angle_set` 발행은 매번 사용자 허락 뒤에 한다.
