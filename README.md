# SOSLAB GL5 ROS 2 실습 · 라이다 없이 (녹화 재생판)

**Mac 한 대와 Docker Desktop만 있으면 됩니다.**
라이다 실물, USB 이더넷 어댑터, 관리자 비밀번호, 중계기 모두 필요 없습니다.
저장소에 들어 있는 GL5 녹화 파일을 재생해서 실습 4단계를 전부 해 봅니다.

> 실물 라이다를 꽂아서 하려면 이 브랜치가 아니라 `mac_practice` 브랜치를 쓰세요.

---

## 0. 이 실습에서 하는 일

라이다는 주변으로 빛을 쏘아 **"몇 도 방향에 몇 미터"** 를 초당 40번 측정해 보냅니다.
그 측정값 덩어리 하나가 **스캔 1장**이고, ROS 2에서는 `/scan` 이라는 이름으로 흘러다닙니다.

이 실습에서는 라이다 대신, 팀에서 미리 **실제 GL5로 찍어 둔 녹화 파일**을 재생합니다.
재생기는 녹화된 스캔을 원래 찍힌 속도(40Hz)로 하나씩 다시 내보내므로,
프로그램 입장에서는 라이다가 꽂혀 있는 것과 **똑같습니다.** 코드는 한 줄도 다르지 않습니다.

그 `/scan` 을 받아 "사람이 지정한 구역에 들어왔는가"를 판단하는 프로그램을 네 조각으로 나눠 채웁니다.

| 단계 | 주제 | 하는 일 |
|---|---|---|
| 1 | 군집화 | 흩어진 점들을 묶어 **물체 하나**로 만든다 |
| 2 | 배경 차분 | 벽·기둥처럼 **원래 있던 것**을 빼고 새로 나타난 것만 본다 |
| 3 | 진입 예측 | 물체가 움직이는 방향으로 **몇 초 뒤 구역에 들어올지** 내다본다 |
| 4 | ICP 스캔 매칭 | 스캔끼리 겹쳐 맞춰 **센서가 얼마나 움직였는지** 알아낸다 |

걸리는 시간: **준비 약 10분**(최초 1회) + **실습 약 60분**.

---

## 1. 준비 (최초 1회)

### a. 터미널 열기

이 문서의 명령은 모두 **터미널** 앱에 입력합니다.
`command + space` → `터미널` 입력 → `Return`. 검은(또는 흰) 창이 열리고 커서가 깜빡입니다.

명령은 **한 줄씩 복사해서 붙여넣고 `Return`** 을 누르면 됩니다.
명령이 끝나면 커서가 다시 깜빡입니다. 그때 다음 명령을 넣습니다.

### b. git 확인

```bash
git --version
```

`git version 2.x.x` 처럼 버전이 나오면 정상입니다.
설치되어 있지 않으면 "커맨드 라인 개발자 도구를 설치하시겠습니까?" 팝업이 뜨는데 **설치**를 누릅니다.
(팝업이 안 뜨면 `xcode-select --install` 을 입력합니다.) 몇 분 걸리고, 끝난 뒤 위 명령으로 다시 확인합니다.

### c. Docker Desktop 설치

[Docker Desktop](https://www.docker.com/products/docker-desktop/)을 설치하고 **앱을 실행한 뒤** 확인합니다.

```bash
docker info --format '{{.OSType}} {{.Architecture}}'
```

`linux aarch64` 가 나오면 정상입니다.

- `command not found` → Docker Desktop이 설치되지 않았습니다.
- `Cannot connect to the Docker daemon` → 앱이 꺼져 있습니다. 실행하고 상단 메뉴바의 고래 아이콘이
  움직임을 멈출 때까지(30초 정도) 기다린 뒤 다시 입력합니다.

### d. VS Code 설치

코드를 고칠 편집기입니다. [VS Code](https://code.visualstudio.com/)를 설치한 뒤 확인합니다.

```bash
open -a "Visual Studio Code"
```

VS Code가 열리면 정상입니다.

> RViz 화면을 볼 **화면 공유(VNC)** 는 macOS에 기본으로 들어 있습니다. 따로 설치할 것이 없습니다.

---

## 2. 프로젝트 받기와 이미지 빌드 (최초 1회)

홈 폴더 아래 `~/lidar_workshop` 에 받습니다.

```bash
cd ~
git clone --branch mac_practice_anywhere https://github.com/churrosboy/lidar_workshop.git
cd lidar_workshop
docker build --platform linux/arm64 -t lidar-workshop:humble-arm64 .
```

`clone` 에는 녹화 파일 약 98MB가 포함돼 있어 1~3분 걸립니다.
`docker build` 는 ROS 2와 RViz를 담은 실습 환경을 만드는 과정으로, 처음 한 번만 하고 **5~15분** 걸립니다.
(네트워크 속도에 좌우됩니다. 중간에 글자가 빠르게 지나가는 것은 정상입니다.)

마지막에 아래처럼 `naming to ... lidar-workshop:humble-arm64 done` 이 나오면 성공입니다.

```
 => => naming to docker.io/library/lidar-workshop:humble-arm64    done
```

코드를 열어 두려면:

```bash
open -a "Visual Studio Code" ~/lidar_workshop
```

---

## 3. 녹화를 재생해서 실행하기

### 3-1. 들어 있는 녹화

저장소의 `bags/` 폴더에 실제 GL5로 찍은 녹화 두 개가 있습니다.

| 이름 | 내용 | 길이 | 쓰는 단계 |
|---|---|---|---|
| `gl5_sopcom_stationary` | 센서를 고정해 두고 사람이 드나듦 | 108초 | 실습 **1~3단계** |
| `gl5_sopcom_moving` | 센서를 손에 들고 이동 | 93초 | 실습 **4단계** |

이름이 기억나지 않으면 인자 없이 실행하면 목록이 나옵니다.

```bash
cd ~/lidar_workshop
bash mac/start-bag.sh
```

### 3-2. 실행 (터미널 1)

녹화 이름을 붙여 실행합니다. 실습하는 동안 $\color{red}{\textsf{이 창은 계속 열어 둡니다.}}$

```bash
cd ~/lidar_workshop
bash mac/start-bag.sh gl5_sopcom_stationary
```

**실습 중에는 이 명령 대신 6장 각 단계의 실행 명령**을 씁니다.

정상이면 접속 안내가 먼저 나오고, 이어서 재생 로그가 나옵니다.

```
녹화 재생 :: gl5_sopcom_stationary (끝까지 가면 처음부터 다시 재생됩니다)

RViz 가 뜨면 화면 공유로 접속하세요.

    open vnc://localhost:5901
  암호: gl5lab

가상 화면 :99 (1400x900x24), VNC 5901 번 대기 중

[INFO] [gl5_bag-1]: process started with pid [42]
[gl5_bag-1] [INFO] [...] [rosbag2_storage]: Opened database '/opt/lidar_workshop/bags/gl5_sopcom_stationary/gl5_sopcom_stationary_0.db3' for READ_ONLY.
[gl5_bag-1] [INFO] [...] [rosbag2_player]: Set rate to 1
```

`Opened database ... for READ_ONLY` 가 나오면 녹화를 제대로 읽은 것입니다.
녹화는 **끝까지 가면 자동으로 처음부터 다시 재생**됩니다. 끌 때까지 무한히 돕니다.

아직 채우지 않은 단계가 있으면 5초마다 `미구현` 줄이 나옵니다. **오류가 아니고 정상입니다.**

```
[gl5_obstacle_node-2] [ERROR] ... 배경 차분 미구현: background.py foreground() 를 채우세요. 2단계 ...
```

### 3-3. RViz 화면 열기 (터미널 2)

`command + T` 로 새 터미널 탭을 열고 접속합니다.
화면 공유 앱이 열리고 $\color{yellow}{\textsf{암호를 물으면 gl5lab을 입력}}$합니다.

```bash
open vnc://localhost:5901
```

검은 배경에 초록색 점들이 보이면 스캔이 들어오고 있는 것입니다.

### 3-4. 감지 영역 그리기 (처음 한 번)

RViz의 **GL5 Region Controls** 패널에서 **Draw Region** → 화면을 **꼭짓점 3개 이상 클릭** → **Finish Region**.

영역은 `mac/.local/gl5_region.json` 에 저장되어 다음 실행부터는 다시 그리지 않아도 됩니다.
마우스 휠로 확대하려면 먼저 상단 도구에서 **Move Camera** 를 고릅니다.

### 3-5. 데이터가 잘 들어오는지 확인 (선택)

`command + T` 로 또 다른 탭을 열어 초당 몇 장 들어오는지 봅니다. 약 40이면 정상입니다.

```bash
docker exec lidar-workshop /ros_entrypoint.sh ros2 topic hz /scan
```

확인했으면 `Ctrl+C` 로 이 명령만 끕니다.

### 3-6. 끄기

**터미널 1** 에서 `Ctrl+C`. RViz와 재생기가 함께 꺼지고 컨테이너도 삭제됩니다.

### 3-7. 잘 안 될 때

| 화면에 나오는 것 | 원인 | 조치 |
|---|---|---|
| `이미지 lidar-workshop:humble-arm64 가 없습니다` | 2장의 `docker build` 를 아직 안 했습니다 | 2장을 실행합니다 |
| `컨테이너 lidar-workshop 가 이미 있습니다` | 다른 창에서 실습이 돌고 있습니다 | 그 창에서 `Ctrl+C`, 또는 `docker stop lidar-workshop` |
| `녹화를 찾지 못했습니다: ...` | 녹화 이름을 잘못 적었습니다 | `bash mac/start-bag.sh` 로 목록을 확인합니다 |
| `Cannot connect to the Docker daemon` | Docker Desktop이 꺼져 있습니다 | 앱을 켜고 30초 기다린 뒤 다시 실행 |
| 화면 공유가 암호를 계속 물음 | 암호는 `gl5lab` 입니다 | 소문자로 정확히 입력합니다 |
| 화면 공유가 연결되지 않음 | 터미널 1이 아직 준비 중입니다 | `VNC 5901 번 대기 중` 줄이 나온 뒤에 접속합니다 |
| RViz는 뜨는데 점이 하나도 없음 | Fixed Frame이 어긋났습니다 | RViz Displays의 **Global Options → Fixed Frame** 이 `laser` 인지 확인 |
| 화면이 끊기고 마우스가 늦게 듣는다 | CPU가 부족합니다 | 아래 **참고**의 "RViz가 느릴 때" |

---

## 4. 녹화 재생이 실물과 다른 점

읽어 두면 "고장난 줄 알았는데 정상"인 상황을 대부분 피할 수 있습니다.

- **끝까지 가면 처음으로 되돌아갑니다.** 화면이 갑자기 앞 장면으로 튀는 것은 정상입니다.
  그 순간 추적 중이던 박스와 4단계의 주황색 경로는 **일부러 초기화**됩니다
  (녹화 끝의 자세에 녹화 처음 장면을 이어 붙이면 경로가 엉키기 때문입니다).
  4단계에서는 터미널 1에 이 줄이 한 번 나옵니다. 오류가 아닙니다.

  ```
  [gl5_scan_matcher-3] [INFO] ... Odometry reset: scan time moved backwards (bag replay restarted)
  ```

- **내가 직접 손을 흔들 수 없습니다.** 확인은 녹화 안에서 움직이는 사람으로 합니다.
  사람이 나올 때까지 몇 초 기다려야 할 수 있습니다.

- **영역은 녹화 장면에 맞춰 그립니다.** 초록 점이 벽처럼 둘러싸고 있고 그 안쪽 빈 공간으로
  사람이 지나갑니다. 사람이 지나다니는 길목을 감싸도록 영역을 그리면 확인이 쉽습니다.

- **배경 학습은 재생이 시작된 직후 약 2초 구간**을 씁니다. 그 구간에 사람이 이미 서 있었다면
  사람까지 배경으로 외워 버립니다. 화면에서 사람이 없는 순간에 RViz 패널의
  **Learn Background** 를 눌러 다시 학습시키면 됩니다.

- **시각은 녹화의 시각**입니다(`/clock`). 모든 노드가 녹화 시계를 따라가므로,
  터미널에 찍히는 시각이 오늘 날짜가 아니어도 정상입니다.

- **점구름(`/points`) 화면은 없습니다.** 녹화에는 `/scan` 만 담겨 있어서 RViz 설정에서 아예 뺐습니다.

---

## 5. 패키지 구조

| 패키지 | 역할 |
|---|---|
| $`\color{red}{\texttt{gl5\_detection}}`$ | 감지 영역 편집, 장애물 감지·추적 |
| $`\color{red}{\texttt{gl5\_localization}}`$ | ICP 스캔 매칭으로 센서 이동 궤적 추정 |
| `gl5_rviz_plugins` | RViz 영역 설정 패널 |
| `gl5_bringup` | 녹화 재생과 전체 노드 실행, RViz 설정 |

빨간 두 개가 이번에 고치는 패키지입니다.
(실물 센서를 읽는 `gl5_driver` 는 이 브랜치에 없습니다. 녹화 재생에는 쓰이지 않기 때문입니다.)

---

## 6. 실습

네 함수의 본문이 비어 있습니다. 한 단계씩 **코드 채우기 → 실행 → 화면 확인** 순서로 진행합니다.

| 단계 | 주제 | 고칠 파일 → 함수 | 정답 |
|---|---|---|---|
| 1 | Clustering | `src/gl5_detection/gl5_detection/detection_core.py` → `cluster_scan` | `src_answer/1_Clustering.py` |
| 2 | Background | `src/gl5_detection/gl5_detection/background.py` → `foreground` | `src_answer/2_Background.py` |
| 3 | Obstacle Estimation | `src/gl5_detection/gl5_detection/prediction.py` → `predict_entry` | `src_answer/3_Obstacle_Estimation.py` |
| 4 | Scan Matching(ICP) | `src/gl5_localization/gl5_localization/icp.py` → `icp` | `src_answer/4_ICP.py` |

공통 규칙:

- 채울 곳은 함수 안의 `Insert Your Code` 두 줄 **사이**입니다.
  `raise NotImplementedError(...)` 줄은 지우고 그 자리에 씁니다.
- 고친 뒤 $\color{yellow}{\textsf{command + S로 저장}}$하고 다시 실행하면 반영됩니다. (다시 빌드할 필요 없습니다.)
- 아직 안 채운 함수가 있으면 터미널 1에 `미구현` 로그가 5초마다 나오고 **그 기능만 건너뜁니다.**
  프로그램이 죽지 않으니, 1단계만 채운 상태로 1단계를 확인해도 됩니다.
- 단계를 바꿀 때는 터미널 1에서 `Ctrl+C` 로 끄고 새 명령으로 다시 켭니다.
- 영역은 한 번 그려 두면 계속 남습니다.

### 1단계 · 군집화

1. **고칠 곳:** `detection_core.py` → `cluster_scan`
2. **실행 (터미널 1):**

   ```bash
   GL5_OBSTACLE_PARAMS_FILE=/opt/lidar_workshop/src/gl5_detection/config/obstacles_step1.yaml \
     bash mac/start-bag.sh gl5_sopcom_stationary scan_matcher:=false
   ```

3. **화면에서 확인:**
   - [ ] 녹화 속 사람이 영역 **안**에 있으면 **빨간 박스**
   - [ ] 영역 **밖**에 있으면 **초록 박스**
   - [ ] 벽도 박스로 잡힙니다. 정상이고, 2단계에서 없앱니다.
   - [ ] 박스가 하나도 안 보이면: `미구현` 로그가 계속 나오는지 확인하세요. 아직 안 채운 것입니다.

### 2단계 · 배경 차분

1. **고칠 곳:** `background.py` → `BackgroundModel.foreground`
2. **실행 (터미널 1):**

   ```bash
   GL5_OBSTACLE_PARAMS_FILE=/opt/lidar_workshop/src/gl5_detection/config/obstacles_step2.yaml \
     bash mac/start-bag.sh gl5_sopcom_stationary scan_matcher:=false
   ```

   켜진 직후 약 2초 동안 녹화 첫 장면으로 배경을 학습합니다.
   터미널에 `Background map learned` 가 나오면 끝난 것입니다.
3. **화면에서 확인:**
   - [ ] 벽·고정물이 **회색 점**(배경)으로 바뀌고 그 박스가 사라짐
   - [ ] 녹화 속에서 움직이는 사람만 박스로 잡힘
   - [ ] 사람까지 회색이 됐으면: 학습 구간에 사람이 서 있던 것입니다.
         사람이 안 보이는 순간에 RViz 패널의 **Learn Background** 를 누릅니다.

### 3단계 · 진입 예측

1. **고칠 곳:** `prediction.py` → `predict_entry`
2. **실행 (터미널 1):**

   ```bash
   bash mac/start-bag.sh gl5_sopcom_stationary scan_matcher:=false
   ```

3. **화면에서 확인:** 녹화 속 사람이 영역 쪽으로 걸어오는 구간을 기다립니다.
   - [ ] 멀리 있을 때 **초록**
   - [ ] 다가오면 **노랑** + 점선 예측선 + `in1.3s` 같은 진입 예상 시간
   - [ ] 영역에 들어가면 **빨강**
   - [ ] 영역 밖에 가만히 서 있으면 예측선이 나오지 않습니다(정상).

### 4단계 · ICP 스캔 매칭

1. **고칠 곳:** `icp.py` → `icp`
2. **실행 (터미널 1):** 이 단계만 **센서를 들고 이동한 녹화**를 씁니다.

   ```bash
   bash mac/start-bag.sh gl5_sopcom_moving
   ```

3. **화면에서 확인:**
   - [ ] 센서가 지나온 자리에 **주황색 경로**가 꼬리처럼 그려짐
   - [ ] 회색 배경이 센서 회전을 따라감
   - [ ] 93초마다 경로가 초기화되고 `Odometry reset: ... bag replay restarted` 가 나옴 (정상)
   - [ ] 경로가 아예 안 그려지면: `ICP 미구현` 로그가 계속 나오는지 확인하세요.
   - [ ] `Match rejected` 가 반복되면 `bag_rate` 를 1보다 크게 준 것입니다. 1배속으로 돌리세요.

---

## 7. 내 녹화를 추가하기 (선택)

`bags/` 아래에 폴더로 넣으면 이름만으로 바로 쓸 수 있습니다. **이미지를 다시 빌드할 필요 없습니다.**

```
bags/
└── 내녹화이름/          ← 공백 없는 영문 이름
    ├── metadata.yaml
    └── 내녹화이름_0.db3
```

```bash
bash mac/start-bag.sh 내녹화이름
```

`/scan`(`sensor_msgs/msg/LaserScan`)이 담겨 있어야 합니다.
실물 센서로 새로 녹화하려면 `mac_practice` 브랜치에서 실습을 띄워 둔 채
`bash mac/start-lidar.sh shell` 로 컨테이너에 들어가 실행합니다.

```bash
ros2 bag record -o /mac-runtime/내녹화이름 /scan
```

녹화가 Mac의 `mac/.local/내녹화이름/` 에 생기므로, 그 폴더를 이 브랜치의 `bags/` 로 옮기면 됩니다.

---

## 참고

- **파이썬 코드 수정:** `src/` 아래 `.py` 를 Mac에서 고치고 실습을 다시 실행하면 바로 반영됩니다.
  C++(`gl5_rviz_plugins`)을 고쳤을 때만 `bash mac/start-bag.sh shell` 로 들어가
  `bash scripts/build.sh` 를 실행합니다.
- **장애물 감지 설정:** `src/gl5_detection/config/obstacles.yaml` 을 수정하고 다시 실행하면 적용됩니다.
- **배경 학습:** RViz 패널의 **Learn Background** 를 영역이 비어 있는 순간에 누르면 벽·고정물을
  배경으로 학습해 감지에서 제외합니다.
- **천천히 보고 싶을 때:** `bag_rate:=0.5` 를 붙이면 절반 속도로 재생합니다.
  반대로 1보다 크게 주면 스캔 사이 움직임이 커져 **4단계 ICP가 실패합니다**(`Match rejected`).
  1~3단계에서만 쓰세요.

  ```bash
  bash mac/start-bag.sh gl5_sopcom_stationary scan_matcher:=false bag_rate:=0.5
  ```

- **RViz가 느릴 때:** mac은 RViz를 컨테이너 안에서 소프트웨어 렌더링하므로 CPU를 많이 씁니다.
  코어가 적은 기기에서는 다음을 차례로 시도합니다.
  `scan_matcher:=false` (ICP 생략, 1~3단계에서는 어차피 필요 없음) →
  `GL5_VNC_GEOMETRY=1100x700x24 bash mac/start-bag.sh ...` (화면을 줄여 렌더링 부담 감소).
- **ICP 경로를 크게 보기:** 주황색 경로에 30초치 누적 스캔까지 겹쳐 봅니다.
  점이 많아 CPU를 훨씬 많이 쓰니 4단계 확인용으로만 잠깐 씁니다.

  ```bash
  bash mac/start-bag.sh gl5_sopcom_moving \
    rviz_config:=/opt/lidar_workshop/src/gl5_bringup/rviz/gl5_odom.rviz
  ```

- **컨테이너 셸:** `bash mac/start-bag.sh shell`
- **이미 실행 중이라는 오류:** `docker stop lidar-workshop` 후 다시 실행합니다.
- **실물 라이다로 하려면:** `mac_practice` 브랜치를 쓰세요. 유선 어댑터 IP 설정과 UDP 중계기가
  필요하며, 이 브랜치에는 그 도구들이 들어 있지 않습니다.
