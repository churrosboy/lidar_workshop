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
git clone --branch mac_practice_1 --recurse-submodules https://github.com/churrosboy/lidar_workshop.git
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

## 6. 실습 (mac_practice_1 브랜치)

이 브랜치는 네 함수의 **핵심 부분만** 비어 있습니다. 좌표 변환이나 numpy 관용구처럼 짧은 시간에 유도하기 어려운 코드는
채워져 있고, 그 단계에서 배워야 할 판단 몇 줄만 `Insert Your Code` 블록으로 남겨 두었습니다. 블록 안 주석이 무엇을 쓸지 알려 줍니다.
완성본은 `mac` 브랜치에 있습니다.

채우기 전에는 `raise NotImplementedError`가 걸려 있어 노드가 5초마다 `미구현` 로그를 내며 **그 기능만** 건너뜁니다.
그래서 1단계만 풀어도 실습이 돌아가고, 순서대로 하나씩 채워 나가면 됩니다.
각 단계의 채점은 **그 단계까지만 풀면 전부 통과**하도록 나눠 두었습니다.

| 단계 | 주제 | 채우는 함수 | 채점 |
|---|---|---|---|
| 1 | 군집화 | `src/gl5_detection/gl5_detection/detection_core.py` → `cluster_scan` | `pytest test/test_clustering.py test/test_roi_geometry.py` |
| 2 | 배경 차분 | `src/gl5_detection/gl5_detection/background.py` → `BackgroundModel.foreground` | `pytest test/test_background.py` |
| 3 | 진입 예측 | `src/gl5_detection/gl5_detection/prediction.py` → `predict_entry` | `pytest test/test_prediction.py` |
| 4 | ICP 스캔 매칭 | `src/gl5_localization/gl5_localization/icp.py` → `icp` 반복 스텝 | `gl5_localization`에서 `pytest test/test_icp.py`<br>`gl5_detection`에서 `pytest test/test_background_alignment.py` |

### 공통 사항

**테스트 실행** — 실습을 끈 뒤 컨테이너 셸로 들어가 해당 패키지 폴더에서 돌립니다.

```bash
bash mac/start-lidar.sh shell
cd src/gl5_detection        # 또는 src/gl5_localization
python3 -m pytest test/test_clustering.py
```

실습이 돌고 있는 중이라면 다른 터미널에서 이렇게도 됩니다.

```bash
docker exec -it lidar-workshop /ros_entrypoint.sh \
  bash -c "cd src/gl5_detection && python3 -m pytest test/test_clustering.py"
```

`pytest test/`처럼 폴더를 통째로 지정해도 되지만, 그러면 아직 안 푼 단계의 실패까지 섞여 나옵니다. 단계별로 파일을 지정하는 편이 읽기 쉽습니다.

**수정 반영** — 파이썬 파일은 **Mac에서 고치고 실습만 다시 실행하면 됩니다.** 이미지를 다시 빌드할 필요 없습니다.

**각 단계 실행** — 앞 단계까지의 기능만 켠 설정으로 확인합니다. 명령은 저장소 최상위(`~/lidar_workshop`)에서 실행합니다.
영역이 없으면 아무것도 감지되지 않으니, 4장 5번의 **Draw Region**을 먼저 해 두세요.

---

### 1단계 — 군집화

**무엇을 하나**

라이다 한 프레임은 각도 순서대로 늘어선 거리값 배열입니다. GL5는 270°를 약 1500개 빔으로 훑습니다.
이 숫자들만으로는 물체가 몇 개인지 알 수 없습니다. 빔을 순서대로 지나가면서
**이웃한 반사점이 충분히 가까우면 같은 물체**로 묶는 것이 군집화입니다.

```
빔 순서 →   ● ● ● ● ●              ● ● ● ● ● ● ●
            \_______/      ↑       \___________/
              물체 A     이 간격이      물체 B
                        max_gap 초과
                        → 여기서 끊는다
```

끊는 경우가 두 가지입니다. 방금처럼 **이웃 점 사이가 `max_gap`보다 멀 때**, 그리고 **점이 감지 영역 밖일 때**입니다.
영역 밖 점은 버리면서 묶음도 함께 끊어야, 영역을 가로지르는 벽이 하나의 거대한 군집으로 뭉치지 않습니다.

**채울 곳** — `src/gl5_detection/gl5_detection/detection_core.py`의 `cluster_scan`, 95번째 줄 블록.

무효한 빔을 거르고 극좌표를 xy로 바꾸는 부분은 채워져 있습니다. 만들어진 `point`를 가지고 세 가지를 하면 됩니다.
영역 밖이면 `flush()` 후 버리기 / 앞점과 `max_gap`보다 멀면 `flush()` / `current`에 추가.
`flush()`는 위에 정의돼 있고, 모인 점이 `min_points` 이상일 때만 군집으로 확정합니다.

**채점**

```bash
cd src/gl5_detection
python3 -m pytest test/test_clustering.py test/test_roi_geometry.py
```

`16 passed`가 나오면 통과입니다.

**눈으로 확인**

```bash
GL5_OBSTACLE_PARAMS_FILE=/opt/lidar_workshop/src/gl5_detection/config/obstacles_step1.yaml \
  bash mac/start-lidar.sh scan_matcher:=false
```

영역 안팎에 사람이나 상자를 놓아 봅니다.

- 영역 **안** 물체 → **빨간** 박스, 패널에 `Obstacle detected in region`
- 영역 **밖** 물체 → **초록** 박스, 패널에 `No obstacles in region`
- 박스 위 라벨은 `#번호 / 가로x세로 / 속도m/s`

**막히면**

| 증상 | 원인 | 조치 |
|---|---|---|
| 한 물체가 여러 박스로 쪼개짐 | 이웃 점 간격이 `cluster_gap`보다 큼. 먼 물체일수록 빔 간격이 벌어집니다 | `obstacles_step1.yaml`의 `cluster_gap`을 키웁니다 (기본 0.30) |
| 두 물체가 한 박스로 합쳐짐 | `cluster_gap`이 너무 큼 | 줄입니다 |
| 작은 물체가 안 잡힘 | 점 개수가 `min_points`(기본 5)보다 적음 | 줄입니다 |
| 벽까지 통째로 박스가 됨 | 영역 밖 점에서 `flush()`를 안 함 | `test_outside_return_still_breaks_cluster` 실패로도 나타납니다 |
| 박스가 아예 없음 | 영역을 안 그림 | 패널이 `No region`이면 **Draw Region**부터 |

---

### 2단계 — 배경 차분

**무엇을 하나**

1단계만으로는 벽·기둥·책상 같은 고정물도 전부 장애물로 잡힙니다.
배경 차분은 **가만히 있는 것들을 미리 외워 두고 빼는** 방법입니다.

먼저 80프레임(약 2초) 동안 스캔을 모아 빔마다 **중앙값**을 취합니다.
중앙값을 쓰기 때문에 학습 중 사람이 잠깐 지나가도 배경에 섞이지 않습니다. 이렇게 만든 점들이 배경 지도입니다.
이후 매 스캔을 배경 지도와 대조해 **지도에서 충분히 떨어진 점만** 남깁니다.

```
배경 지도  ─────────────────────   (외워 둔 벽)
이번 스캔  ─────────● ● ●───────   (벽 + 새로 놓인 물체)
                    ↑
            지도에서 멀리 떨어짐 → 전경(장애물)
```

"충분히"의 기준이 두 부분입니다. 고정 여유 `margin`(0.15 m)과 거리에 비례하는 여유 `ratio`(2%)입니다.
먼 점일수록 빔 하나가 덮는 폭이 넓어 오차가 커지므로 비례 항이 필요합니다.

**채울 곳** — `background.py`의 `BackgroundModel.foreground`, 93번째 줄 블록.

가장 가까운 배경 점까지의 거리 `distance`는 이미 구해져 있습니다. `keep` 하나만 만들면 됩니다 —
각 점이 전경인지 담는 True/False 배열입니다. 시야 가장자리 처리와 결과 조립은 그 아래에 채워져 있습니다.

**채점**

```bash
cd src/gl5_detection
python3 -m pytest test/test_background.py
```

`6 passed`.

**눈으로 확인**

```bash
GL5_OBSTACLE_PARAMS_FILE=/opt/lidar_workshop/src/gl5_detection/config/obstacles_step2.yaml \
  bash mac/start-lidar.sh scan_matcher:=false
```

실행 직후 약 2초간 배경을 학습합니다. 이때 패널은 `Learning background - keep area clear`입니다 —
**영역을 비우고 센서를 고정**하세요.

- 학습이 끝나면 벽·고정물이 **회색 점**으로 바뀌고 박스가 사라집니다
- 새로 놓은 물체에만 박스가 생깁니다
- 원래 있던 물체를 치워도 그 자리가 잡힙니다. 배경과 달라진 것이 기준이기 때문입니다
- 센서를 옮겼으면 패널의 **Learn Background**를 다시 누릅니다

**막히면**

| 증상 | 원인 | 조치 |
|---|---|---|
| 회색 점이 안 보임 | 아직 학습 중 | 패널이 `Learning background`면 잠시 기다립니다 |
| 배경인데 계속 잡힘 | 학습할 때 그 자리에 사람이나 물건이 있었음 | 영역을 비우고 **Learn Background** |
| 먼 곳에서 오탐이 많음 | 거리에 비례하는 여유를 안 넣음 | `keep` 식에 `ratio * frame[index]`가 들어가야 합니다 |
| 물체가 통째로 안 잡힘 | 부등호 방향이 반대 | 지도에서 **먼** 점이 전경입니다 |
| `Scan does not align with the background map` | 센서가 크게 움직임 | **Learn Background**를 다시 누릅니다. 4단계 전에는 정렬이 없어 센서를 고정해야 합니다 |

---

### 3단계 — 진입 예측

**무엇을 하나**

영역에 들어온 뒤에 알려 주면 늦습니다. 3단계는 영역 **밖** 물체도 추적하면서,
지금 속도로 계속 간다면 **언제 영역에 들어올지** 미리 계산합니다.

속도는 추적기가 이미 구해 둡니다(최근 0.4초 위치 이력의 직선 회귀).
남은 일은 그 속도로 직진한다고 가정하고 0.1초 간격으로 3초 앞까지 찍어 보며
**처음 영역 안에 들어가는 순간**을 찾는 것입니다.

```
   현재 ●─ → ─ → ─ → ─ ⊗ ─ → ─ →
             0.1s 0.2s  │
                  ┌─────┼─────────┐
                  │           감지 영역
                  │     ⊗ = 처음 들어가는 지점 (이 시각과 좌표를 반환)
                  └───────────────┘
```

**채울 곳** — `prediction.py`의 `predict_entry`, 14번째 줄 블록.

너무 느리거나(`min_speed`) 이미 영역 안이면 예측하지 않는 조기 반환은 채워져 있습니다.
시간 루프만 쓰면 됩니다. 찾으면 `(t, point)`를, 끝까지 못 찾으면 `None`을 반환합니다.

**채점**

```bash
cd src/gl5_detection
python3 -m pytest test/test_prediction.py
```

`5 passed`.

**눈으로 확인**

```bash
bash mac/start-lidar.sh scan_matcher:=false
```

영역 밖에서 안쪽을 향해 걸어 들어갑니다.

- 멀리 있을 때 **초록**
- 진입 예상이 2초(`warning_time`) 이내가 되면 **노랑**으로 바뀌고, 예상 진입 지점까지 노란 **점선**이 그려집니다.
  라벨에 `in1.3s`가 붙고 패널은 `Obstacle about to enter region`
- 실제로 들어가면 **빨강**

**막히면**

| 증상 | 원인 | 조치 |
|---|---|---|
| 노란 박스·점선이 안 뜸 | 너무 느림 (`min_predict_speed` 0.1 m/s 미만) | 조금 빠르게 움직입니다 |
| 〃 | 진입 예상이 `warning_time`(2초)보다 멀다 | 계속 접근하면 뜹니다 |
| 〃 | 영역을 **향해** 가고 있지 않음 | 등속 직진 가정이라 비껴가면 예측되지 않습니다 |
| 점선이 엉뚱한 곳을 가리킴 | 속도 추정이 튐. 추적이 매 프레임 새로 잡히는 중 | 라벨의 `#번호`가 유지되는지 봅니다 |
| 언패킹 오류 | 반환이 `(시각, 지점)` 형태가 아님 | |

---

### 4단계 — ICP 스캔 매칭

**무엇을 하나**

여기까지는 센서가 고정돼 있다고 가정했습니다. 센서를 들고 움직이면 세상이 통째로 움직인 것처럼 보여서
2단계 배경 지도가 어긋납니다.

ICP(Iterative Closest Point)는 **두 스캔을 겹쳐 놓고 가장 잘 맞는 이동·회전을 찾는** 방법입니다.
이 실습은 점을 점에 맞추지 않고 **면(벽)에 붙이는** point-to-line 방식을 씁니다.
벽을 따라 미끄러지는 방향은 자유롭게 두고 벽에서 떨어진 정도만 줄이기 때문에, 평평한 실내에서 훨씬 안정적입니다.

```
   이번 스캔 점  ●
                 ╲   법선 방향(벽에 수직) 오차만 줄인다
                  ▼
   ──────────────────────────  키프레임의 벽
                 ←──→  벽을 따라가는 방향은 건드리지 않는다
```

한 번에 맞춰지지 않으므로 **최근접 대응 찾기 → 보정량 계산 → 적용**을 반복합니다.

**채울 곳** — `icp.py`의 `icp`, 91번째 줄 블록.

대응쌍(`p`, `q`, `n`)과 야코비안·잔차는 채워져 있습니다. 반복 한 회차의 나머지 네 가지를 쓰면 됩니다.
최소제곱으로 보정량 풀기 / 발산 검사 / 변환 누적과 점 갱신 / 수렴하면 종료.

**채점**

```bash
cd src/gl5_localization && python3 -m pytest test/test_icp.py                    # 8 passed
cd ../gl5_detection    && python3 -m pytest test/test_background_alignment.py    # 2 passed
```

두 번째가 4단계 채점에 있는 이유는, ICP가 생겨야 2단계 배경이 센서 움직임을 따라가기 때문입니다.

**눈으로 확인**

```bash
bash mac/start-lidar.sh rviz_config:=/opt/lidar_workshop/src/gl5_bringup/rviz/gl5_odom.rviz
```

센서를 들고 천천히 움직입니다.

- **주황색 경로**가 지나온 자리를 그립니다
- 스캔이 누적되어 방 전체 모양이 만들어집니다
- 기본 화면으로 돌아와서 센서를 돌려 보면, 회색 배경과 감지 영역이 제자리에 붙어 있습니다

**막히면**

| 증상 | 원인 | 조치 |
|---|---|---|
| 포즈가 전혀 안 움직임 | `current`를 갱신하지 않아 다음 회차가 같은 자리에서 시작 | 보정량을 점에도 적용해야 합니다 |
| 경로가 튀거나 발산 | 변환 합성 순서가 반대 | 새 보정량을 왼쪽에 곱합니다 |
| `Match rejected (fitness ... < 0.5)` 반복 | 너무 빨리 움직였거나 특징이 없는 공간 | 천천히 움직이거나 `scan_matcher.yaml`의 `max_correspondence`를 키웁니다 |
| `Match rejected (implausible motion ...)` | 연속 스캔 사이 이동이 0.15 m·15°를 넘음 | 천천히 움직입니다 |
| RViz가 버벅임 | ICP가 CPU를 많이 씀 | `scan_matcher.yaml`의 `process_every`를 2로 둡니다 |


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
