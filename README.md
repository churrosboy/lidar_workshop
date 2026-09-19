# SOSLAB GL5 ROS 2 실습

macOS(Apple Silicon) + Docker Desktop 기준.
ROS 2·SDK·드라이버는 Docker 이미지 안에서 설치·빌드됩니다.

## 1. 준비 (최초 1회)

- 터미널에서 `git --version`을 입력해 git이 설치돼 있는지 확인합니다.
  버전이 나오면 그대로 넘어갑니다.
  설치되어 있지 않으면 "커맨드 라인 개발자 도구를 설치하시겠습니까?" 팝업이 뜨는데,
  **설치**를 눌러 진행합니다. (팝업이 뜨지 않으면 터미널에 `xcode-select --install`을 입력합니다.)
  설치는 몇 분 걸리며, 끝난 뒤 `git --version`으로 다시 확인합니다.

- [Docker Desktop](https://www.docker.com/products/docker-desktop/)을 설치하고 실행합니다.
  준비됐는지 터미널에서 확인합니다. `linux aarch64`가 나오면 정상입니다.

  ```bash
  docker info --format '{{.OSType}} {{.Architecture}}'
  ```

  `command not found`가 나오면 설치되지 않은 것이고, `Cannot connect to the Docker daemon`이 나오면 Docker Desktop이 꺼져 있는 것입니다. 앱을 켜고 잠시 기다린 뒤 다시 실행합니다.

- 터미널에서 `python3 --version`이 나오는지 확인합니다.
  없으면 위의 커맨드 라인 도구(`xcode-select --install`)를 설치하면 함께 들어옵니다.

중계기용 `tcpdump`와 RViz 화면용 화면 공유(VNC)는 macOS에 기본으로 들어 있습니다.

## 2. 프로젝트 준비 및 이미지 빌드

**터미널에서 실행합니다.** 홈 폴더 아래 `~/lidar_workshop`에 받습니다. (이미지 빌드는 최초 1회만 합니다.)

```bash
cd ~
git clone --branch mac_practice --recurse-submodules https://github.com/churrosboy/lidar_workshop.git
cd lidar_workshop
docker build --platform linux/arm64 -t lidar-workshop:humble-arm64 .
```

기본 연결: 센서 `10.110.1.2:2000` → PC `10.110.1.3:3000`.
다른 GL5를 쓰면 `src/gl5_driver/config/gl5.yaml`의 `sensor_ip`를 수정합니다.
(같은 파일의 `lidar_type`을 `"GL3"`로 두면 GL3도 쓸 수 있습니다. macOS에서는 아직 검증하지 않았습니다.)

## 3. 유선 LAN 설정

라이다 전원을 켜고 USB 이더넷 어댑터로 Mac에 연결합니다.
**터미널 1**에서 어댑터 이름을 찾고 주소를 지정합니다.

- `$IFACE`라는 네트워크 어댑터에 `10.110.1.3/24`라는 IP 주소를 설정합니다.
- $\color{yellow}{\textsf{Mac 사용자 계정의 관리자 비밀번호를 입력}}$합니다.

```bash
cd ~/lidar_workshop
IFACE=$(bash mac/find_iface.sh) && echo "라이다 어댑터: $IFACE" &&
  sudo ifconfig "$IFACE" inet 10.110.1.3 netmask 255.255.255.0
```

어댑터를 찾지 못하면 목록이 출력됩니다. 케이블과 전원을 다시 확인하거나 `en8`처럼 이름을 직접 넣습니다.

## 4. 실행·수신 확인

1. **터미널 1** (위와 같은 창)에서 중계기를 띄웁니다. 실습하는 동안 $\color{red}{\textsf{이 창은 계속 열어 둡니다.}}$

   ```bash
   sudo python3 mac/gl5_relay.py --iface "$IFACE" --listen 0.0.0.0:2000
   ```

   &nbsp;

2. `command + T`로 터미널 창 2를 켭니다. **터미널 2**에서 드라이버·장애물 감지·RViz를 실행합니다. 실습하는 동안 $\color{red}{\textsf{이 창은 계속 열어 둡니다.}}$

   ```bash
   cd ~/lidar_workshop
   bash mac/start-lidar.sh
   ```

   터미널 2에 아래 줄이 5초마다 나오고 숫자가 계속 늘어나면 라이다 데이터를 잘 받고 있는 것입니다.

   ```
   [gl5_node-1] [INFO] [1789568238.811711542] [gl5_node]: Received 240 frames
   ```

   `Received ... frames` 대신 `GL5 stream command not acknowledged`가 나오면 터미널 1의 중계기가 켜져 있는지 확인합니다.
   `Invalid pageLength`가 반복되면 `gl5.yaml`의 `lidar_type`이 연결한 센서와 다른 것입니다.

   &nbsp;

3. `command + T`로 터미널 창 3을 켜고, 실제 스캔 수신 주기를 확인합니다. 약 40 Hz가 나오면 정상입니다.
   확인했으면 **Ctrl+C**로 빠져나옵니다.

   ```bash
   docker exec lidar-workshop /ros_entrypoint.sh ros2 topic hz /scan
   ```

   &nbsp;

4. **터미널 3**에서 아래 명령으로 RViz 화면에 접속합니다.
   화면 공유 앱이 열리고 $\color{yellow}{\textsf{암호를 물으면 gl5lab을 입력}}$합니다.

   ```bash
   open vnc://localhost:5901
   ```

   &nbsp;

5. RViz: **Draw Region → 꼭짓점 3개 이상 클릭 → Finish Region**.
   영역은 `mac/.local/gl5_region.json`에 자동 저장됩니다.

## 5. 패키지 구조

| 패키지 | 역할 |
|---|---|
| `gl5_driver` | 센서 수신, `/scan`·`/points` 발행 |
| `gl5_detection` | 감지 영역 편집, 장애물 감지·추적 |
| `gl5_localization` | ICP 스캔 매칭으로 센서 이동 궤적 추정 |
| `gl5_rviz_plugins` | RViz 영역 설정 패널 |
| `gl5_bringup` | 전체 노드 실행과 RViz 설정 |

## 6. 실습 (mac_practice 브랜치)

이 브랜치는 네 함수의 본문이 비어 있습니다. 각 단계에서 원리를 설명한 뒤 함수를 채우고, 테스트로 확인하고, 실행해 봅니다.
비어 있는 함수는 `raise NotImplementedError`로 표시되어 있고, 채우기 전에는 노드가 5초마다 `미구현` 로그를 내며 그 기능만 건너뜁니다. 그래서 1단계만 풀어도 실습이 돌아갑니다.
완성본은 `mac` 브랜치에 있습니다.

| 단계 | 주제 | 채우는 함수 | 채점 |
|---|---|---|---|
| 1 | 군집화 | `src/gl5_detection/gl5_detection/detection_core.py` → `cluster_scan` | `pytest test/test_clustering.py test/test_roi_geometry.py` |
| 2 | 배경 차분 | `src/gl5_detection/gl5_detection/background.py` → `BackgroundModel.foreground` | `pytest test/test_background.py` |
| 3 | 진입 예측 | `src/gl5_detection/gl5_detection/prediction.py` → `predict_entry` | `pytest test/test_prediction.py` |
| 4 | ICP 스캔 매칭 | `src/gl5_localization/gl5_localization/icp.py` → `icp` 반복 스텝 | `pytest test/test_icp.py` |

단계별 실행 (앞 단계 기능만 켠 설정으로 확인합니다):

```bash
# 1단계
GL5_OBSTACLE_PARAMS_FILE=/opt/lidar_workshop/src/gl5_detection/config/obstacles_step1.yaml \
  bash mac/start-lidar.sh scan_matcher:=false
# 2단계
GL5_OBSTACLE_PARAMS_FILE=/opt/lidar_workshop/src/gl5_detection/config/obstacles_step2.yaml \
  bash mac/start-lidar.sh scan_matcher:=false
# 3단계
bash mac/start-lidar.sh scan_matcher:=false
# 4단계
bash mac/start-lidar.sh rviz_config:=/opt/lidar_workshop/src/gl5_bringup/rviz/gl5_odom.rviz
```

테스트는 실습을 끈 뒤 `bash mac/start-lidar.sh shell`로 들어가 해당 패키지 폴더(`src/gl5_detection` 또는 `src/gl5_localization`)에서 실행합니다.
실습이 돌고 있는 중이라면 다른 터미널에서 이렇게도 됩니다.

```bash
docker exec -it lidar-workshop /ros_entrypoint.sh \
  bash -c "cd src/gl5_detection && python3 -m pytest test/test_clustering.py"
```

파이썬 파일은 **Mac에서 고치고 실습만 다시 실행하면 반영됩니다.** 다시 빌드할 필요 없습니다.

기대 결과: 1단계에서는 영역 안 물체에 빨간 박스, 밖은 초록. 2단계에서는 벽·고정물이 회색 배경이 되어 박스가 사라지고 새로 놓은 물체만 잡힘.
3단계에서는 영역 밖에서 걸어 들어올 때 초록 → 노랑(점선 예측, `in1.3s`) → 빨강. 4단계에서는 센서를 들고 움직이면 주황색 경로와 누적 스캔이 그려지고,
2단계 배경도 센서 회전을 따라가게 됩니다.

## 7. 종료

터미널 2에서 **Ctrl+C**로 드라이버·RViz를 종료합니다. 컨테이너도 함께 삭제됩니다.
터미널 1에서 **Ctrl+C**로 중계기를 종료합니다.

## 참고

- **중계기가 필요한 이유:** GL5는 데이터를 이더넷 브로드캐스트 프레임으로 보냅니다. $\color{blue}{\textsf{macOS 커널은 이 프레임을 버리므로}}$, 중계기가 `tcpdump`로 받아 컨테이너에 다시 보내 줍니다.
- **장애물 감지 설정:** `src/gl5_detection/config/obstacles.yaml`을 수정하고 실습을 다시 실행하면 적용됩니다. 이미지는 다시 빌드하지 않아도 됩니다.
- **파이썬 코드 수정:** `src/` 아래 `.py`를 Mac에서 고치고 실습을 다시 실행하면 바로 반영됩니다. C++(`gl5_driver`, `gl5_rviz_plugins`)을 고쳤을 때만 `bash mac/start-lidar.sh shell`로 들어가 `bash scripts/build.sh`를 실행합니다.
- **배경 학습:** RViz 패널의 **Learn Background**를 영역을 비운 상태에서 누르면 벽·고정물을 배경으로 학습해 감지에서 제외합니다. 센서를 크게 옮겼으면 다시 누릅니다.
- **컨테이너 셸:** `bash mac/start-lidar.sh shell`
- **RViz가 느릴 때:** mac은 RViz를 컨테이너 안에서 소프트웨어 렌더링하므로 CPU를 많이 씁니다.
  10코어 기준 기본 구성이 약 900%를 쓰니, 코어가 적은 기기에서는 다음을 차례로 시도합니다.
  `bash mac/start-lidar.sh scan_matcher:=false` (ICP 스캔 매칭 생략, 약 370% 절약) →
  `GL5_VNC_GEOMETRY=1100x700x24 bash mac/start-lidar.sh` (화면을 줄여 렌더링 부담 감소).
- **ICP 스캔 매칭 화면:** `bash mac/start-lidar.sh rviz_config:=/opt/lidar_workshop/src/gl5_bringup/rviz/gl5_odom.rviz`
  로 주황색 이동 경로와 누적 스캔을 봅니다.
- **이미 실행 중이라는 오류:** `docker stop lidar-workshop` 후 다시 실행합니다.
