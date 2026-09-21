# SOSLAB GL5 ROS 2 실습

macOS(Apple Silicon) + Docker Desktop 기준.
ROS 2·SDK·드라이버는 Docker 이미지 안에서 설치·빌드됩니다.

## 1. 준비 (최초 1회)

아래 명령은 모두 **터미널**에서 입력합니다.

### a. git 설치

```bash
git --version
```

버전이 나오면 정상입니다.
설치되어 있지 않으면 "커맨드 라인 개발자 도구를 설치하시겠습니까?" 팝업이 뜨는데, **설치**를 눌러 진행합니다.
(팝업이 뜨지 않으면 터미널에 `xcode-select --install`을 입력합니다.) 설치는 몇 분 걸리며, 끝난 뒤 `git --version`으로 다시 확인합니다.

### b. Docker Desktop 설치

[Docker Desktop](https://www.docker.com/products/docker-desktop/)을 설치하고 실행한 뒤 확인합니다.

```bash
docker info --format '{{.OSType}} {{.Architecture}}'
```

`linux aarch64`가 나오면 정상입니다.
`command not found`가 나오면 설치되지 않은 것이고, `Cannot connect to the Docker daemon`이 나오면 Docker Desktop이 꺼져 있는 것입니다. 앱을 켜고 잠시 기다린 뒤 다시 실행합니다.

### c. VS Code 설치

[VS Code](https://code.visualstudio.com/)를 설치한 뒤 확인합니다.

```bash
open -a "Visual Studio Code"
```

VS Code가 열리면 정상입니다.

### d. python3

```bash
python3 --version
```

버전이 나오면 정상입니다.
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

코드 파일은 VS Code로 열어 확인 및 수정이 가능합니다. VS Code에서 `lidar_workshop` 폴더를 엽니다.

```bash
open -a "Visual Studio Code" ~/lidar_workshop
```

## 3. 라이다 연결·실행

라이다를 뽑으면 IP 설정·중계기·실습이 모두 꺼집니다. 라이다를 **꽂을 때마다 3-1부터** 다시 합니다.

### 3-1. 라이다 꽂기

라이다 전원을 켜고 USB 이더넷 어댑터로 Mac에 연결합니다.

### 3-2. 어댑터 IP 지정 (터미널 1)

어댑터 이름(`$IFACE`)을 찾아 `10.110.1.3/24` 주소를 지정합니다.
$\color{yellow}{\textsf{Mac 사용자 계정의 관리자 비밀번호를 입력}}$합니다.

```bash
cd ~/lidar_workshop
IFACE=$(bash mac/find_iface.sh) && echo "라이다 어댑터: $IFACE" &&
  sudo ifconfig "$IFACE" inet 10.110.1.3 netmask 255.255.255.0
```

어댑터를 찾지 못하면 목록이 출력됩니다. 케이블과 전원을 다시 확인하거나 `en8`처럼 이름을 직접 넣습니다.

### 3-3. 중계기 켜기 (터미널 1)

같은 창에서 중계기를 띄웁니다. 실습하는 동안 $\color{red}{\textsf{이 창은 계속 열어 둡니다.}}$

```bash
sudo python3 mac/gl5_relay.py --iface "$IFACE" --listen 0.0.0.0:2000
```

### 3-4. 실습 실행 (터미널 2)

`command + T`로 터미널 2를 켜고 드라이버·장애물 감지·RViz를 실행합니다. 실습하는 동안 $\color{red}{\textsf{이 창은 계속 열어 둡니다.}}$

```bash
cd ~/lidar_workshop
bash mac/start-lidar.sh
```

**실습 중에는 이 명령 대신 5장 각 단계의 실행 명령**을 씁니다.

아래 줄이 5초마다 나오고 숫자가 계속 늘어나면 라이다 데이터를 잘 받고 있는 것입니다.

```
[gl5_node-1] [INFO] [1789568238.811711542] [gl5_node]: Received 240 frames
```

`Received ... frames` 대신 `GL5 stream command not acknowledged`가 나오면 터미널 1의 중계기가 켜져 있는지 확인합니다.
`Invalid pageLength`가 반복되면 `gl5.yaml`의 `lidar_type`이 연결한 센서와 다른 것입니다.

### 3-5. RViz 화면 열기 (터미널 3)

`command + T`로 터미널 3을 켜고 RViz 화면에 접속합니다.
화면 공유 앱이 열리고 $\color{yellow}{\textsf{암호를 물으면 gl5lab을 입력}}$합니다.

```bash
open vnc://localhost:5901
```

### 3-6. 감지 영역 그리기 (처음 한 번)

RViz에서 **Draw Region → 꼭짓점 3개 이상 클릭 → Finish Region**.
영역은 `mac/.local/gl5_region.json`에 저장되어 다음부터는 다시 그리지 않아도 됩니다.

### 3-7. 종료·반납

1. **터미널 2**에서 Ctrl+C, **터미널 1**에서 Ctrl+C. 컨테이너도 함께 삭제됩니다.
2. 케이블을 뽑아 다음 사람에게 넘깁니다.

다음에 실행할 때 `이미 실행 중`이라는 오류가 나면 `docker stop lidar-workshop` 후 다시 실행합니다.

## 4. 패키지 구조

| 패키지 | 역할 |
|---|---|
| `gl5_driver` | 센서 수신, `/scan`·`/points` 발행 |
| $`\color{red}{\texttt{gl5\_detection}}`$ | 감지 영역 편집, 장애물 감지·추적 |
| $`\color{red}{\texttt{gl5\_localization}}`$ | ICP 스캔 매칭으로 센서 이동 궤적 추정 |
| `gl5_rviz_plugins` | RViz 영역 설정 패널 |
| `gl5_bringup` | 전체 노드 실행과 RViz 설정 |

## 5. 실습

네 함수의 본문이 비어 있습니다. 한 단계씩 **코드 채우기 → 라이다 연결·실행 → 화면 확인 → 종료·반납** 순서로 진행합니다.
라이다는 여럿이 나눠 쓰므로, **코드는 라이다 없이 먼저 채워 두고** 라이다를 받았을 때 실행만 합니다.

| 단계 | 주제 | 고칠 파일 → 함수 | 정답 |
|---|---|---|---|
| 1 | Clustering | `src/gl5_detection/gl5_detection/detection_core.py` → `cluster_scan` | `src_answer/1_Clustering.py` |
| 2 | Background | `src/gl5_detection/gl5_detection/background.py` → `foreground` | `src_answer/2_Background.py` |
| 3 | Obstacle Estimation | `src/gl5_detection/gl5_detection/prediction.py` → `predict_entry` | `src_answer/3_Obstacle_Estimation.py` |
| 4 | Scan Matching(ICP) | `src/gl5_localization/gl5_localization/icp.py` → `icp` | `src_answer/4_ICP.py` |

- 채울 곳은 함수 안의 `Insert Your Code` 두 줄 사이입니다. `raise NotImplementedError(...)` 줄은 지우고 그 자리에 씁니다.
- 고친 뒤 $\color{yellow}{\textsf{command + S로 저장}}$하고 다시 실행하면 반영됩니다. (다시 빌드할 필요 없습니다.)
- 아직 안 채운 함수가 있으면 터미널 2에 `미구현` 로그가 5초마다 나오고 그 기능만 건너뜁니다. 오류가 아닙니다.

라이다를 받으면 **3. 라이다 연결·실행**을 3-1부터 하고, 3-4에서 아래 단계의 **실행** 명령을 씁니다. 끝나면 **3-7. 종료·반납**으로 넘깁니다.

### 1단계 · 군집화

1. **고칠 곳:** `detection_core.py` → `cluster_scan`
2. **실행 (터미널 2):**

   ```bash
   GL5_OBSTACLE_PARAMS_FILE=/opt/lidar_workshop/src/gl5_detection/config/obstacles_step1.yaml \
     bash mac/start-lidar.sh scan_matcher:=false
   ```

3. **화면에서 확인:**
   - [ ] 영역 안에 물체(손, 상자)를 두면 **빨간 박스**
   - [ ] 영역 밖 물체는 **초록 박스**
   - [ ] 벽도 박스로 잡힙니다. 정상이고, 2단계에서 없앱니다.

### 2단계 · 배경 차분

1. **고칠 곳:** `background.py` → `BackgroundModel.foreground`
2. **실행 (터미널 2):**

   ```bash
   GL5_OBSTACLE_PARAMS_FILE=/opt/lidar_workshop/src/gl5_detection/config/obstacles_step2.yaml \
     bash mac/start-lidar.sh scan_matcher:=false
   ```

   켜진 직후 약 2초 동안 배경을 학습합니다. 그동안 **영역을 비우고 라이다를 움직이지 않습니다.**
3. **화면에서 확인:**
   - [ ] 벽·고정물이 **회색 점**(배경)으로 바뀌고 그 박스가 사라짐
   - [ ] 새로 놓은 물체만 박스로 잡힘
   - [ ] 라이다를 건드려 배경이 어긋나면 RViz 패널의 **Learn Background**를 다시 누릅니다.

### 3단계 · 진입 예측

1. **고칠 곳:** `prediction.py` → `predict_entry`
2. **실행 (터미널 2):**

   ```bash
   bash mac/start-lidar.sh scan_matcher:=false
   ```

3. **화면에서 확인:** 영역 밖에서 영역 쪽으로 걸어 들어옵니다.
   - [ ] 멀리 있을 때 **초록**
   - [ ] 다가오면 **노랑** + 점선 예측선 + `in1.3s` 같은 진입 예상 시간
   - [ ] 영역에 들어가면 **빨강**
   - [ ] 영역 밖에 가만히 서 있으면 예측선이 나오지 않습니다(정상).

### 4단계 · ICP 스캔 매칭

1. **고칠 곳:** `icp.py` → `icp`
2. **실행 (터미널 2):**

   ```bash
   bash mac/start-lidar.sh
   ```

3. **화면에서 확인:** 라이다를 들고 **천천히** 움직이거나 돌립니다.
   - [ ] 지나온 자리에 **주황색 경로**가 꼬리처럼 그려짐
   - [ ] 회색 배경이 라이다 회전을 따라감
   - [ ] 경로가 멈추고 터미널 2에 `Match rejected`가 반복되면 너무 빨리 움직인 것입니다. Ctrl+C 후 다시 실행합니다.

## 6. 종료

실습을 모두 마치면 **3-7. 종료·반납**과 같이 터미널 2, 터미널 1 순서로 Ctrl+C 합니다.

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
  로 주황색 이동 경로에 30초치 누적 스캔까지 겹쳐 봅니다. 점이 많아 CPU를 훨씬 많이 쓰니 실습 4단계에는 쓰지 않습니다.
- **이미 실행 중이라는 오류:** `docker stop lidar-workshop` 후 다시 실행합니다.
- **라이다를 뺐다 꽂았을 때:** **3. 라이다 연결·실행**을 3-1부터 다시 합니다.
