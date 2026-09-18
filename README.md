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
git clone --branch mac --recurse-submodules https://github.com/churrosboy/lidar_workshop.git
cd lidar_workshop
docker build --platform linux/arm64 -t lidar-workshop:humble-arm64 .
```

기본 연결: 센서 `10.110.1.2:2000` → PC `10.110.1.3:3000`.
다른 GL5를 쓰면 `src/gl5_driver/config/gl5.yaml`을 수정합니다.

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

   &nbsp;

3. `command + T`로 터미널 창 3을 켜고, 아래 명령으로 RViz 화면에 접속합니다.
   화면 공유 앱이 열리고 $\color{yellow}{\textsf{암호를 물으면 gl5lab을 입력}}$합니다.

   ```bash
   open vnc://localhost:5901
   ```

   &nbsp;

4. RViz: **Draw Region → 꼭짓점 3개 이상 클릭 → Finish Region**.
   영역은 `mac/.local/gl5_region.json`에 자동 저장됩니다.

## 5. 종료

터미널 2에서 **Ctrl+C**로 드라이버·RViz를 종료합니다. 컨테이너도 함께 삭제됩니다.
터미널 1에서 **Ctrl+C**로 중계기를 종료합니다.

## 참고

- **중계기가 필요한 이유:** GL5는 데이터를 이더넷 브로드캐스트 프레임으로 보냅니다. $\color{blue}{\textsf{macOS 커널은 이 프레임을 버리므로}}$, 중계기가 `tcpdump`로 받아 컨테이너에 다시 보내 줍니다.
- **장애물 감지 설정:** `src/gl5_detection/config/obstacles.yaml`을 수정하고 실습을 다시 실행하면 적용됩니다. 이미지는 다시 빌드하지 않아도 됩니다.
- **컨테이너 셸:** `bash mac/start-lidar.sh shell`
- **이미 실행 중이라는 오류:** `docker stop lidar-workshop` 후 다시 실행합니다.

## 패키지 구조

| 패키지 | 역할 |
|---|---|
| `gl5_driver` | 센서 수신, `/scan`·`/points` 발행 |
| `gl5_detection` | 감지 영역 편집, 장애물 감지·추적 |
| `gl5_rviz_plugins` | RViz 영역 설정 패널 |
| `gl5_bringup` | 전체 노드 실행과 RViz 설정 |
