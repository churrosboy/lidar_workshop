# SOSLAB GL5 ROS 2 실습

이 저장소의 루트가 ROS 작업 공간입니다. 클론한 디렉토리에서 `bash scripts/build.sh`, `bash scripts/run.sh`를 실행합니다. `src/`는 ROS 패키지, `SOSLAB_SDK/`는 제조사 SDK, `tools/`는 단독 수신·검증 도구, `config/`는 저장된 감지 영역입니다. 빌드 결과는 `build/`, `install/`, `log/`에 생성되고 검증 기록은 `artifacts/`에 저장됩니다.

Ubuntu 22.04 / ROS 2 Humble에서 GL5 SDK를 빌드하고, UDP 데이터를 `/scan`과 `/points`로 발행합니다. 센서 IP·포트는 장치 설정에 맞춰야 합니다.

## Docker 이미지 빌드

Docker Desktop 또는 Docker Engine이 설치된 호스트에서는 Ubuntu 22.04와 ROS 2 Humble을 직접 설치하지 않고 이미지를 빌드할 수 있습니다. SDK 서브모듈을 포함해 저장소를 클론한 뒤 저장소 루트에서 실행합니다.

```bash
git clone --recurse-submodules https://github.com/churrosboy/lidar_workshop.git
cd lidar_workshop
docker build --platform linux/arm64 -t lidar-workshop:humble-arm64 .
```

Apple Silicon에서는 `linux/arm64`를, 일반적인 Intel/AMD Linux와 Windows PC에서는 `linux/amd64`를 사용합니다.

```bash
docker build --platform linux/amd64 -t lidar-workshop:humble-amd64 .
```

빌드 결과를 확인하려면 다음을 실행합니다. 컨테이너 시작 시 ROS 2 Humble과 이 작업 공간의 `install/setup.bash`가 자동으로 적용됩니다.

```bash
docker run --rm --platform linux/arm64 lidar-workshop:humble-arm64 \
  ros2 pkg prefix gl5_driver

docker run --rm --platform linux/arm64 lidar-workshop:humble-arm64 \
  file /opt/lidar_workshop/SOSLAB_SDK/_archive_/lib/libLidar_x64_release.so
```

현재 Apple Silicon의 `linux/arm64` 이미지에서 SDK, 단독 수신 도구와 `gl5_driver`의 소스 빌드를 확인했습니다. macOS·Windows에서 실물 Ethernet LiDAR의 UDP를 컨테이너로 전달하는 방법과 RViz GUI 실행 방법은 별도 검증이 필요합니다. `scripts/network.sh`는 호스트의 NetworkManager를 조작하므로 Docker Desktop 컨테이너 안에서 실행하지 않습니다.

## 1. 최초 환경 준비와 빌드

Ubuntu 22.04에 ROS 2 Humble이 설치되어 있어야 합니다. `/opt/ros/humble/setup.bash`가 없는 PC는 ROS 설치를 먼저 완료하세요. 아래 설치 명령은 ROS 패키지 저장소가 이미 설정된 환경을 전제로 합니다.

```bash
sudo apt update
sudo apt install build-essential cmake git qtbase5-dev network-manager \
  python3-colcon-common-extensions python3-rosdep python3-yaml python3-matplotlib \
  ros-humble-desktop ros-humble-rosbag2 ros-humble-ament-cmake-gtest
```

클론은 원하는 상위 디렉토리에서 실행합니다. `<저장소 URL>`은 배포받은 주소로 바꿉니다.

```bash
git clone <저장소 URL> ros2_ws
cd ros2_ws
source /opt/ros/humble/setup.bash
```

rosdep을 처음 사용하는 PC에서만 `sudo rosdep init`을 실행한 뒤, 의존성을 설치합니다.

```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
bash scripts/build.sh
```

`build.sh`가 제조사 SDK → 단독 수신 도구 → ROS 패키지를 순서대로 빌드합니다. **이하 명령은 모두 클론한 `ros2_ws` 디렉토리 기준**입니다. 기존 작업을 다시 열 때는 자신의 클론 경로로 이동하세요. 개발 PC의 경로는 다음과 같습니다.

```bash
cd /home/dhkim/soslab/ros2_ws
bash scripts/build.sh
```

SDK 결과는 `SOSLAB_SDK/_archive_`, 단독 수신 실행 파일은 `build/standalone/gl5_receive`, ROS 2 패키지는 `install`에 생성됩니다. 시스템 라이브러리 디렉토리에 설치하지 않습니다. 제공 SDK의 `copy_api2example.sh`도 실행할 필요가 없습니다.

현재 개발 PC에서 빌드 및 변환 테스트를 수행했습니다. 새 PC에서의 최초 의존성 설치는 별도로 확인해야 합니다.

## 2. Ethernet 설정

먼저 `ip -br link`로 유선 인터페이스 이름을 확인합니다. 아래의 `enp0s31f6`은 개발 PC의 이름이므로 자신의 PC에 맞춰 바꾸세요.

```bash
ip -br link
export GL5_INTERFACE=enp0s31f6
bash scripts/network.sh show
```

센서 주소가 확인되지 않으면 아래 명령으로 장치가 보내는 ARP/UDP 패킷을 확인합니다. 관리자 암호는 로컬 터미널에만 입력합니다.

```bash
sudo timeout 15 tcpdump -i enp0s31f6 -nn -c 20 'arp or udp'
```

`who-has <PC IP> tell <센서 IP>` 형태의 ARP 또는 UDP 출발지·목적지에서 주소를 확인할 수 있습니다. 아무 패킷도 없다면 전원·케이블·장치 설정 자료를 확인해야 합니다. Ethernet 링크 UP만으로 데이터 송신을 보장하지 않습니다.

이 GL5에서 검증한 연결은 센서 `10.110.1.2:2000` → PC `10.110.1.3:3000`입니다. 유선 PC 주소는 다음과 같이 지정합니다.

```bash
GL5_PC_CIDR=10.110.1.3/24 bash scripts/network.sh up
```

이 명령은 `gl5-lab-temp`라는 임시 NetworkManager 프로필을 만듭니다. 기본 게이트웨이를 추가하지 않으며 Wi-Fi 인터넷 경로를 유지합니다. 이미 PC 주소가 설정되어 있으면 생략합니다. 프로필이 이미 존재하면 `nmcli connection up gl5-lab-temp`로 활성화합니다. 복원은 다음과 같습니다.

```bash
bash scripts/network.sh down
```

`src/gl5_driver/config/gl5.yaml`에는 위 실측 설정이 반영되어 있습니다. 사용자 제공 후보 `2020`은 UDP 연결 거부가 발생했고, `2000`에서 SDK 스트리밍 응답과 거리 데이터가 확인됐습니다. 다른 GL5를 사용할 때에는 `sensor_ip`, `sensor_port`, `pc_ip`, `pc_port`를 해당 장치에 맞게 수정하세요. 현재 장치는 PC 수신 포트 `3000`을 사용합니다.

## 3. ROS 없이 수신 확인

선택 단계입니다. ROS 드라이버와 SDK 수신 프로그램을 동시에 실행하지 마세요.

```bash
mkdir -p artifacts
./build/standalone/gl5_receive 10.110.1.2 2000 10.110.1.3 3000 30 artifacts/sdk.json
```

인자 순서는 센서 IP, 센서 UDP 포트, PC IP, PC UDP 포트, 측정 시간(초), 결과 JSON입니다. UDP 소켓 생성 로그는 장치 연결 성공을 뜻하지 않습니다. 스트리밍 명령 응답과 실제 수신 프레임을 확인해야 합니다. 종료 코드는 0=수신 성공, 2=입력/실행 오류, 3=소켓 연결 실패, 4=명령 응답 또는 유효 데이터 수신 실패입니다.

## 4. ROS 2와 RViz

```bash
export ROS_DOMAIN_ID=42
bash scripts/run.sh params_file:="$PWD/src/gl5_driver/config/gl5.yaml"
```

GUI 없이 노드만 실행하려면 `rviz:=false`를 추가합니다. 기본 실행은 설치된 YAML을 사용하므로, 소스 YAML 수정 후에는 위처럼 `params_file`을 지정하거나 다시 빌드하세요.

종료는 실행 터미널에서 `Ctrl+C`를 누릅니다. RViz 창만 닫으면 감지 노드가 계속 실행될 수 있습니다. 재실행 전에 기존 실행을 종료하세요. 장애물 번호는 감지 노드를 새로 시작할 때 1부터 부여합니다. 여러 실습 PC가 같은 네트워크를 사용하면 PC마다 서로 다른 `ROS_DOMAIN_ID`를 사용하고, 같은 PC의 실행·조회 터미널에서는 같은 값을 지정하세요.

RViz Fixed Frame은 `laser`입니다. `GL5 scan`과 `GL5 points` 표시를 각각 켜거나 끌 수 있습니다. 두 표시 모두 Best Effort QoS를 사용합니다. 전방에 물체를 놓고 +X 방향에 나타나는지, 좌측 물체가 +Y에 나타나는지 확인하세요. 처음에는 센서 프레임 자체로 표시하므로 TF가 필요하지 않습니다.

별도 터미널에서:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic hz /scan
ros2 topic echo /scan --once --qos-reliability best_effort
```

SDK는 GL5 샘플 각도를 -45°~225°로 계산합니다. 드라이버는 -90° 회전시켜 중앙을 +X, 각도 범위를 -135°~135°로 표현합니다. `/points`도 같은 변환을 사용합니다. 장착 방향은 실물로 확인하고 필요하면 `angle_offset`(라디안)을 조정합니다.

거리 단위는 m, 0 및 범위 밖 거리는 `/scan`의 `+inf`, `/points`의 NaN으로 표현합니다. 초기 `range_max: 60.0`은 SDK 파서 상한이며 센서가 모든 표면에서 60 m를 측정한다는 뜻이 아닙니다. 시각은 PC 수신 시각이며 첫 `scan_time`은 0, 이후는 수신 간격입니다. 빔별 시간은 미확인이라 `time_increment`를 0으로 둡니다. 파라미터는 시작 시 적용하며 변경 후 노드를 재시작합니다.

5초 동안 유효 형식의 프레임이 없으면 노드는 오류 코드 4로 종료합니다. 자동 재연결은 하지 않으며 연결 상태를 확인한 뒤 다시 실행합니다. 각 프레임의 거리가 모두 무효인 경우에도 메시지는 발행되지만 실기 검증에서는 유효 거리 수를 별도로 확인합니다.

## 5. 자동 검증과 기록

```bash
# 클론한 작업 공간에서 단위 테스트
source /opt/ros/humble/setup.bash
colcon test --base-paths src --packages-select gl5_driver
colcon test-result --verbose

# 실행 중인 노드의 실제 토픽을 60초 관찰
export ROS_DOMAIN_ID=42
bash scripts/verify.sh --seconds 60

# 노드가 꺼진 상태에서 SDK 30초 → ROS 60초 → bag·그림 저장
bash scripts/test_hardware.sh
```

`test_hardware.sh`는 실제 장치만 사용합니다. 같은 센서에 접속하는 다른 노드·제조사 프로그램을 종료한 뒤 실행하세요. 성공 시 `artifacts/hardware_<실행시각>/`에 SDK·ROS 통계, YAML, rosbag, 정적 스캔 그림을 저장합니다. RViz 화면에서의 물체 방향 검증은 별도로 수행합니다.

토픽 검증은 메시지 수, 수신 중단, 유효 거리, 배열 길이, 270° 각도 범위, 메시지 시각, `/scan`과 `/points` 좌표·intensity 일치를 검사합니다. 검증 도구는 데이터를 발행하지 않습니다.

현재 검증 결과와 제한은 `VALIDATION.md`를 참고하세요.

## 6. 직접 지정한 영역의 장애물 감지

### 파라미터 YAML

`src/gl5_driver/config/obstacles.yaml`에서 클러스터링·경고 지연·추적·글자 크기를 조절합니다. 각 항목에 단위와 조정 효과를 주석으로 적었습니다. `bash scripts/run.sh`는 이 소스 파일을 직접 읽으므로 **파일 수정 후 재빌드 없이 노드를 재시작**하면 적용됩니다. 실행 중에는 자동 반영되지 않습니다. 다른 파일은 다음과 같이 지정합니다.

```bash
bash scripts/run.sh obstacle_params_file:=/절대경로/obstacles.yaml
```

조정은 `cluster_gap`과 `min_points`부터 시작하세요. 현재 YAML의 `cluster_gap`은 `0.30` m입니다. 한 물체가 여러 박스로 쪼개지면 조금씩 늘려 비교하고, 작은 잡음을 줄이려면 `min_points`를 `5 → 8`로 올려 비교합니다. 무효점은 건너뛰고 스캔 순서상 앞뒤 유효점 사이 거리로 군집을 나눕니다. `min_points`에는 유효점만 셉니다. 영역 밖 점은 거리 설정과 관계없이 군집을 끊습니다. 가까운 사람과 상자가 합쳐질 수 있으므로 한 번에 하나의 값만 바꾸세요. 현재 글자 크기는 `label_height: 0.14`입니다.

이하 기본 실행에는 같은 YAML이 사용됩니다. 단독 `ros2 run gl5_driver gl5_obstacle_node.py` 실행 시에는 `--ros-args --params-file /절대경로/obstacles.yaml`을 붙여야 파일을 읽습니다.

`bash scripts/run.sh`는 GL5 드라이버, 장애물 노드, RViz를 함께 실행합니다. 기존에 실행 중인 드라이버가 있으면 먼저 종료하세요. 저장된 영역이 있으면 자동으로 적용합니다. 저장 파일이 없으면 영역을 지정할 때까지 장애물 판정을 하지 않습니다.

**RViz 왼쪽 고정 패널:** `GL5 Region Controls`에서 `Draw Region`을 누르면 점 선택 도구가 활성화됩니다. 꼭짓점을 클릭한 뒤 `Finish Region`을 누르세요. `Clear Region`은 편집 점·적용 영역·저장 영역을 모두 지웁니다. `Undo Last Point`, `Cancel Edit`, `Load Saved Region`도 같은 패널에 있습니다. 상태와 오류 이유도 패널에 표시되어 스캔을 가리지 않습니다. 확대·축소는 `Move Camera (Zoom)` 버튼을 누른 뒤 휠을 사용하거나 `Views → Scale`을 조절합니다. 패널이 닫혔다면 `Panels → Add New Panel → gl5_driver/RegionPanel`로 다시 추가합니다. 이 플러그인을 사용하려면 `bash scripts/run.sh` 또는 workspace의 `install/setup.bash`를 source한 터미널에서 RViz를 실행하세요.

장애물 박스는 영역이 확정된 뒤에만 표시됩니다. 편집 중 상태가 계속 보이면 첫 점 근처를 클릭하거나 패널의 `Finish Region`을 누르세요. 변이 서로 교차하면 확정이 거부되므로 전체 삭제 후 겹치지 않는 사각형 4점으로 시작하는 것을 권합니다.

1. RViz의 Fixed Frame을 `laser`로 유지하고 `Detection region and obstacles` 표시를 켭니다.
2. 패널의 **Draw Region**을 눌러 Publish Point 도구로 전환합니다. 스캔 점이 없는 곳도 선택할 수 있도록 평면이 표시됩니다.
3. 감지할 구역의 둘레를 따라 꼭짓점을 3개 이상 클릭합니다. 시계·반시계 방향 모두 가능하며 오목한 다각형도 가능합니다.
4. 첫 꼭짓점에서 15 cm 이내를 다시 클릭하면 영역이 닫히고 저장됩니다. 또는 `bash scripts/region.sh finish`를 실행합니다. 아주 작은 영역은 마지막 꼭짓점이 첫 점에 가까울 수 있으므로 큰 영역부터 연습하세요.
5. 영역 안에 물체를 넣으면 감지 점과 축에 나란한 직사각형 박스가 나타납니다. 0.2초간 감지가 이어지면 `OCCUPIED`, 0.5초간 사라지면 `CLEAR`가 됩니다. 박스는 관측된 점의 범위이며 물체의 전체 크기가 아닙니다.

영역을 다시 그리거나 실수를 되돌리는 명령:

```bash
bash scripts/region.sh edit    # 기존 영역 보존 후 새 영역 편집 시작 (판정 중지)
bash scripts/region.sh undo    # 마지막 꼭짓점 취소
bash scripts/region.sh finish  # 영역 검증·확정·저장
bash scripts/region.sh cancel  # 편집 취소, 기존 영역 복원
bash scripts/region.sh clear   # 영역 삭제 후 저장
bash scripts/region.sh load    # 저장된 영역 불러오기
bash scripts/region.sh status  # 현재 상태 한 번 출력
```

영역은 `config/gl5_region.json`에 저장되며 재실행 시 자동으로 읽습니다. 다른 파일은 `bash scripts/run.sh region_file:=/절대경로/region.json`으로 지정할 수 있습니다. 단독 `ros2 launch`에서는 `region_file`을 지정하지 않으면 현재 디렉토리의 `gl5_region.json`을 사용합니다. 교차하는 변, 중복점, 면적 없는 다각형, 좌표계가 다른 파일은 거부합니다. 저장이 실패하면 편집 상태를 유지합니다.

| 토픽 | 내용 |
|---|---|
| `/gl5/obstacle_state` | `NO_REGION`, `EDITING`, `NO_DATA`, `CLEAR`, `OCCUPIED` |
| `/gl5/obstacle_detected` | 상태가 `OCCUPIED`일 때만 true. false만으로 안전 또는 정상 수신을 판단하지 않음 |
| `/gl5/region` | 적용된 다각형 (`PolygonStamped`, 단위 m) |
| `/gl5/obstacle_markers` | 선택 평면·영역·감지 점·직사각형·장애물 라벨 (`MarkerArray`) |

영역 안에서 스캔 순서상 연속되고 점 사이 간격이 `cluster_gap` 이하인 점을 묶습니다(현재 YAML: 30 cm). 5점 이상인 묶음만 장애물 후보로 표시합니다. 영역 밖 점·무효점은 묶음을 끊습니다. 사람과 상자를 구별하지 않으며 영역 안에 들어온 사람도 장애물입니다. 데이터가 1초간 없거나 프레임이 잘못됐거나 모든 거리가 무효이면 `NO_DATA`로 전환하고 이전 박스와 라벨을 제거합니다. `CLEAR`는 관측된 점으로 조건을 만족하는 장애물이 없다는 뜻이며 가려진 공간의 비어 있음을 보장하지 않습니다.

각 박스 위의 작은 빨간 라벨은 `#ID`, `X방향 폭 x Y방향 폭`, `속도 m/s` 세 줄입니다. 크기 단위는 m입니다. 폭은 영역 안에서 관측한 점들의 범위로, 물체의 실제 전체 크기나 수직 높이가 아닙니다. 박스 중심의 예측 위치와 가까운 후보를 일대일로 연결해 짧은 시간 동안 ID를 유지합니다. 최근 0.4초의 박스 중심 위치에 선형 회귀를 적용해 센서 기준 속도를 추정합니다. 최초 0.15초 미만, 가림 후 재검출 등 정보가 부족하면 `--m/s`를 표시합니다. 0.5초 이상 사라진 추적은 삭제하고 영역 변경·데이터 끊김 시 추적을 초기화합니다. 사람이 겹치거나 군집이 갈라지고 합쳐지면 ID가 바뀔 수 있고, 보이는 단면이 바뀌면 정지 물체에도 속도가 추정될 수 있습니다. 지면 기준 이동 속도로 해석하려면 센서를 고정해야 합니다. 글자 높이는 `label_height` 파라미터(현재 YAML 0.14 m)로 조정합니다.

현재 YAML의 판정 파라미터는 `min_points=5`, `cluster_gap=0.30`, `enter_delay=0.2`, `exit_delay=0.5`, `data_timeout=1.0`, `close_radius=0.15`입니다. 단독 노드는 `ros2 run gl5_driver gl5_obstacle_node.py --ros-args -p ...`로 조정할 수 있습니다. 감지 노드의 `frame_id`는 `/scan`과 RViz Fixed Frame에 맞아야 합니다.

실습에서는 먼저 비어 있는 좁은 구역을 선택하고, 상자 진입·퇴장과 구역 옆 사람 통과를 비교하세요. 배경 집기가 영역 안에 있으면 계속 감지되는 것이 정상입니다.

합성 데이터를 이용한 ROS 통합 테스트는 실제 센서와 분리된 도메인에서만 실행합니다.

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=91 ROS_LOG_DIR=/tmp/gl5-roi-test-logs python3 tools/test_roi_integration.py
```

## SDK 별도 배포와 Git 설정

`ros2_ws/.gitignore`는 빌드 결과, 실행 기록, Python 캐시와 개인 감지 영역을 제외합니다. 이미 Git으로 추적하는 파일은 이 설정만으로 추적 해제되지 않습니다.

SDK는 서브모듈로 배포하면 실습생이 같은 커밋을 받아 빌드할 수 있습니다. 현재 `SOSLAB_SDK/`는 자체 `.git`이 있는 별도 저장소이며 아직 서브모듈로 등록하지 않았습니다. 현재 검증한 SDK 커밋은 `9a1f4c46f0842a8a23e62cab7e8f39da9c09f9ef`입니다.

강사는 `ros2_ws`에서 다음을 실행합니다. `git init`은 이 폴더를 처음 저장소로 만들 때만 필요합니다. 기존 SDK 저장소를 그대로 재사용합니다.

```bash
git init
git submodule add https://github.com/SOSLAB-github/SOSLAB_SDK.git SOSLAB_SDK
git add .gitmodules SOSLAB_SDK .gitignore
```

나머지 프로젝트 소스와 함께 커밋해서 배포합니다. SDK의 전체 소스 대신 원본 URL과 선택된 커밋을 기록합니다. SDK 내부의 빌드 결과나 로컬 `COLCON_IGNORE`는 SDK에 커밋할 필요가 없습니다. 빌드 스크립트는 `--base-paths src`로 ROS 패키지만 빌드합니다.

실습생은 일반 클론 명령 대신 다음을 사용합니다. 의존성 설치는 위 환경 준비 절차와 동일합니다.

```bash
git clone --recurse-submodules <저장소 URL> ros2_ws
cd ros2_ws
bash scripts/build.sh
```

이미 일반 클론했다면 작업 공간에서 다음을 실행합니다.

```bash
git submodule update --init --recursive
```

서브모듈 없이 SDK를 따로 클론하는 방식도 가능하지만, 버전과 다운로드 절차를 별도로 관리해야 합니다. 실습 배포에는 검증된 커밋을 고정하는 서브모듈 방식이 적합합니다.
