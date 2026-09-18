# SOSLAB GL5 ROS 2 실습

Windows 11 + Docker Desktop(Linux 컨테이너) 기준.
ROS 2·SDK·드라이버는 Docker 이미지 안에서 설치·빌드됩니다.

## 1. 준비 (최초 1회)

- PowerShell에서 `git --version`을 입력해 git이 설치돼 있는지 확인합니다.
  버전이 나오면 그대로 넘어갑니다. 없으면 설치합니다.

  ```powershell
  winget install --id Git.Git --exact --source winget
  ```

- [Docker Desktop](https://www.docker.com/products/docker-desktop/)을 설치하고 실행합니다.
  준비됐는지 PowerShell에서 확인합니다. `linux`가 나오면 정상입니다.

  ```powershell
  docker info --format '{{.OSType}}'
  ```

  `docker`를 찾을 수 없다고 나오면 설치되지 않은 것이고, `Cannot connect`가 나오면 Docker Desktop이 꺼져 있는 것입니다. 앱을 켜고 잠시 기다린 뒤 다시 실행합니다.

Python(UDP 중계용)과 VcXsrv(RViz 화면용)는 다음 단계의 `setup.cmd`가 찾거나 설치합니다. 이미 설치된 Python이 있으면 그대로 씁니다.

## 2. 프로젝트 준비 및 이미지 빌드

**PowerShell에서 실행합니다.** 홈 폴더 아래 `lidar_workshop`에 받습니다. (이미지 빌드는 최초 1회만 합니다.)

```powershell
cd ~
git clone --branch windows_practice --recurse-submodules https://github.com/churrosboy/lidar_workshop.git
cd lidar_workshop
.\windows\setup.cmd
docker build --platform linux/amd64 -t lidar-workshop:humble-amd64 .
```

`setup.cmd`는 Python과 VcXsrv를 찾아(없으면 winget으로 설치) `windows/settings.json`을 만듭니다.
끝나면 찾은 경로가 출력됩니다. 설치가 새로 됐다면 PowerShell을 새로 열고 다시 실행합니다.

기본 연결: 센서 `10.110.1.2:2000` → PC `10.110.1.3:3000`.
다른 센서나 GL3를 쓰면 `windows/settings.json`의 `sensor_ip`, `lidar_type`(`"GL5"` 또는 `"GL3"`)을 수정합니다.

## 3. 유선 LAN 설정 (최초 1회)

라이다 전원을 켜고 유선 LAN(또는 USB 이더넷 어댑터)으로 PC에 연결합니다.
**PowerShell을 관리자 권한으로 열어** 실행합니다. 케이블이 꽂힌 유선 어댑터를 자동으로 찾아 `10.110.1.3/24`를 지정하고 방화벽을 엽니다. Wi-Fi는 그대로 유지됩니다.

```powershell
cd ~\lidar_workshop
powershell.exe -NoProfile -ExecutionPolicy Bypass -File windows\configure-network.ps1
ping 10.110.1.2
```

유선 어댑터가 여러 개 연결돼 있으면 후보 목록이 출력됩니다. 그때는 `-InterfaceIndex 16`처럼 번호를 붙여 다시 실행합니다.
`ping`은 IP 응답 확인용입니다. 실제 데이터 수신은 아래 4단계에서 확인합니다.

## 4. 실행·수신 확인

1. **일반 PowerShell**에서 드라이버·장애물 감지·RViz를 실행합니다. Docker Desktop이 켜져 있어야 합니다. 실습하는 동안 $\color{red}{\textsf{이 창은 계속 열어 둡니다.}}$

   ```powershell
   cd ~\lidar_workshop
   .\windows\start-lidar.cmd
   ```

   중계기와 VcXsrv가 자동으로 뜨고 RViz 창이 열립니다. 아래 줄이 5초마다 나오고 숫자가 계속 늘어나면 라이다 데이터를 잘 받고 있는 것입니다.

   ```
   [gl5_node-1] [INFO] [1789568238.811711542] [gl5_node]: Received 240 frames
   ```

   `Invalid pageLength`가 반복되면 `settings.json`의 `lidar_type`이 연결한 센서와 다른 것입니다.

   &nbsp;

2. **다른 PowerShell 창**에서 실제 스캔 수신 주기를 확인합니다. 약 40 Hz가 나오면 정상입니다.

   ```powershell
   docker exec lidar-workshop /ros_entrypoint.sh ros2 topic hz /scan
   ```

   &nbsp;

3. RViz: **Draw Region → 꼭짓점 3개 이상 클릭 → Finish Region**.
   영역은 `windows/.local/gl5_region.json`에 자동 저장됩니다.

## 5. 종료

실습 실행 창에서 **Ctrl+C**로 드라이버·RViz를 종료합니다.
컨테이너까지 정지하려면 PowerShell에서 실행합니다.

```powershell
docker stop lidar-workshop
```

## 참고

- **중계기가 필요한 이유:** 센서는 PC의 물리 LAN 주소로 UDP를 보내는데 Docker 컨테이너는 그 주소를 직접 받을 수 없습니다. `lidar_udp_relay.py`가 Windows에서 받아 컨테이너로 다시 보내 줍니다.
- **장애물 감지 설정:** `src/gl5_detection/config/obstacles.yaml`을 수정하고 실습을 다시 실행하면 적용됩니다. 이미지는 다시 빌드하지 않아도 됩니다.
- **경로를 직접 지정하고 싶을 때:** `windows/settings.json`의 `python_exe`, `vcxsrv_exe`에 `/` 구분자로 적습니다. 비워 두면 실행할 때 자동으로 찾습니다.
- **이미 실행 중이라는 오류:** `docker stop lidar-workshop` 후 다시 실행합니다.

## 패키지 구조

| 패키지 | 역할 |
|---|---|
| `gl5_driver` | 센서 수신, `/scan`·`/points` 발행 |
| `gl5_detection` | 감지 영역 편집, 장애물 감지·추적 |
| `gl5_localization` | ICP 스캔 매칭으로 센서 이동 궤적 추정 |
| `gl5_rviz_plugins` | RViz 영역 설정 패널 |
| `gl5_bringup` | 전체 노드 실행과 RViz 설정 |

## 실습 단계 (windows_practice 브랜치)

이 브랜치는 네 함수의 본문이 비어 있습니다. 각 단계에서 원리를 설명한 뒤 함수를 채우고, 테스트로 확인하고, 실행해 봅니다.
비어 있는 함수는 `raise NotImplementedError`로 표시되어 있고, 채우기 전에는 노드가 5초마다 `미구현` 로그를 내며 그 기능만 건너뜁니다.
완성본은 `windows` 브랜치에 있습니다.

| 단계 | 주제 | 채우는 함수 | 채점 | 실행 |
|---|---|---|---|---|
| 1 | 군집화 | `src/gl5_detection/gl5_detection/detection_core.py` → `cluster_scan` | `pytest test/test_clustering.py test/test_roi_geometry.py` | `GL5_OBSTACLE_PARAMS_FILE=.../config/obstacles_step1.yaml bash scripts/run.sh scan_matcher:=false` |
| 2 | 배경 차분 | `src/gl5_detection/gl5_detection/background.py` → `BackgroundModel.foreground` | `pytest test/test_background.py` | `GL5_OBSTACLE_PARAMS_FILE=.../config/obstacles_step2.yaml bash scripts/run.sh scan_matcher:=false` |
| 3 | 진입 예측 | `src/gl5_detection/gl5_detection/prediction.py` → `predict_entry` | `pytest test/test_prediction.py` | `bash scripts/run.sh scan_matcher:=false` |
| 4 | ICP 스캔 매칭 | `src/gl5_localization/gl5_localization/icp.py` → `icp` 반복 스텝 | `pytest test/test_icp.py` | `bash scripts/run.sh rviz_config:=.../rviz/gl5_odom.rviz` |

테스트는 해당 패키지 폴더(`src/gl5_detection` 또는 `src/gl5_localization`)에서 `source /opt/ros/humble/setup.bash` 후 실행합니다.
Docker 환경에서는 `docker exec -it lidar-workshop /ros_entrypoint.sh bash -c "cd src/gl5_detection && python3 -m pytest test/test_clustering.py"` 처럼 실행합니다.
파이썬 파일을 고친 뒤 노드에 반영하려면 `bash scripts/build.sh`를 다시 실행합니다.

기대 결과: 1단계에서는 영역 안 물체에 빨간 박스, 밖은 초록. 2단계에서는 벽·고정물이 회색 배경이 되어 박스가 사라지고 새로 놓은 물체만 잡힘.
3단계에서는 영역 밖에서 걸어 들어올 때 초록 → 노랑(점선 예측, `in1.3s`) → 빨강. 4단계에서는 센서를 들고 움직이면 주황색 경로와 누적 스캔이 그려지고,
2단계 배경도 센서 회전을 따라가게 됩니다.
