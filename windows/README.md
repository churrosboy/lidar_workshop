# Windows 실습 실행

Windows 11 Pro에서 **Docker Desktop(Hyper-V) + VcXsrv + Windows UDP 중계**로 GL5를 실행하는 구성이다. WSL은 사용하지 않았다. Docker 내부는 Ubuntu 22.04 / ROS 2 Humble이다.

## 준비

1. [Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/)을 설치한다. Hyper-V 방식은 all-users 설치를 사용한다. Hyper-V 활성화 후 재부팅한다.
2. Docker 그룹 권한 오류가 있으면 현재 계정의 `docker-users` 등록을 확인하고 다시 로그인한다.
3. Windows용 Python 3와 [VcXsrv](https://github.com/marchaesen/vcxsrv/releases)를 설치한다. 설치 파일은 이 저장소에 포함하지 않는다.
4. SDK 서브모듈을 포함해 저장소를 받는다.

```powershell
git clone --branch windows --recurse-submodules https://github.com/churrosboy/lidar_workshop.git
cd lidar_workshop
docker build --platform linux/amd64 -t lidar-workshop:humble-amd64 .
Copy-Item windows/settings.example.json windows/settings.json
```

ZIP으로 받으면 SDK 서브모듈 내용이 빠지므로 별도 준비가 필요하다. `settings.json`에서 센서 IP·포트와 `python_exe`, `vcxsrv_exe`를 설치 환경에 맞게 지정한다. JSON의 Windows 경로는 `C:/.../python.exe`처럼 슬래시를 쓰거나 역슬래시를 두 번 쓴다. 빈 실행 경로는 PATH의 `python.exe`와 기본 VcXsrv 설치 경로를 사용한다.

## USB 유선 LAN 설정

연결: PC USB-C → USB LAN 어댑터 → Ethernet → GL5.

검증한 센서는 `10.110.1.2:2000`에서 PC `10.110.1.3:3000`으로 UDP를 전송했다. **다른 센서는 장치 설정을 먼저 확인한다.**

관리자 PowerShell에서 USB 유선 어댑터의 `ifIndex`를 확인한다.

```powershell
Get-NetAdapter | Format-Table ifIndex, Name, InterfaceDescription
powershell.exe -NoProfile -ExecutionPolicy Bypass -File windows/configure-network.ps1 -InterfaceIndex <USB-LAN-ifIndex>
```

이 명령은 선택한 어댑터의 IPv4를 고정 IP로 변경하고 센서에서 오는 UDP 수신만 허용한다. 기본 게이트웨이와 DNS를 추가하지 않으며 Wi-Fi는 변경하지 않는다. `network.sh`는 Linux NetworkManager용이므로 Docker Desktop 컨테이너 안에서 실행하지 않는다. 이 어댑터를 일반 유선 인터넷에 다시 쓰려면 IP 자동 할당을 복원해야 한다.

## 실행

Docker Desktop을 켜고 `windows/start-lidar.cmd`를 실행한다. PowerShell 실행 정책은 해당 프로세스에만 적용하며 전역 정책을 바꾸지 않는다.

실행기는 다음을 준비한다.

- PC별 X 인증 쿠키와 화면 서버: `windows/.local/`
- Windows 루프백 UDP 중계: `127.0.0.1:13000`
- Docker 컨테이너 하나: `lidar-workshop`
- Docker 호스트 주소 자동 조회 및 드라이버 설정 생성
- `run.sh`로 드라이버·장애물 감지·RViz 실행

동일 이름의 다른 컨테이너가 있으면 자동 삭제하지 않고 중단한다. 기존 작업을 보존한 뒤 수동으로 이전하거나 `settings.json`의 컨테이너 이름을 조정한다. 이미 구성한 이 PC의 기존 실행기는 그대로 사용할 수 있으며, 이 배포용 실행기는 별도 체크아웃의 런타임 경로를 사용한다.

중계는 Windows가 센서의 고정 목적지 포트에서 직접 수신하고, Docker가 연결한 루프백 상대에게 반환한다. **컨테이너에 센서 UDP 포트를 게시할 필요가 없다.** 기본 SDK를 그대로 사용한다. 센서 쪽 소켓은 설정한 IP·포트의 패킷만 전달한다. 루프백 중계는 신뢰할 수 있는 로컬 실습 환경을 전제로 하며, 같은 PC의 다른 프로그램도 접근할 수 있다.

X 서버는 인증 쿠키를 사용하고 소프트웨어 OpenGL로 RViz를 표시한다. `settings.json`, `.local/`, 인증 파일, 설치 프로그램, 로그는 Git에 포함하지 않는다.

## 토픽 및 종료

다른 터미널에서:

```powershell
docker exec -it lidar-workshop /ros_entrypoint.sh bash
```

```bash
ros2 topic list
ros2 topic info /scan
ros2 topic info /points
ros2 topic hz /scan
ros2 topic echo /scan --once --qos-reliability best_effort
```

`/scan`은 LaserScan, `/points`는 PointCloud2다. 구독자만 있어도 토픽 이름은 나타날 수 있으므로 발행자와 실제 메시지 주기를 함께 확인한다.

실습 실행 터미널의 Ctrl+C로 드라이버·감지·RViz를 종료한다. 화면 서버와 중계는 재실행을 위해 남는다. 중계를 종료하려면 `.local/relay.pid`의 프로세스가 `lidar_udp_relay.py`인지 확인하고 작업 관리자에서 해당 프로세스를 종료한다. 센서 SDK 수신 프로그램을 드라이버와 동시에 실행하지 않는다.

## 검증 범위

2026-09-12 이 구성의 원형을 Windows 11 Pro / Intel i5 / RAM 16GB에서 검증했다.

- SDK·ROS 드라이버·RViz 패널 빌드 성공
- 영역 판정 8개, 추적 5개, 변환 5개 테스트 통과
- 가상 스캔 ROS 통합 테스트 통과
- 실제 센서 약 40Hz, 프레임당 1,500개 샘플
- `/scan`, `/points` 발행 및 RViz 영역 지정·저장 확인
- Wi-Fi 인터넷 연결 유지

배포용 스크립트는 PC별 경로를 설정으로 분리하고 호스트 IP를 자동 조회하도록 정리했다. 기존 실습 세션을 보존하기 위해 **새 PC에서의 최초 설치부터 재부팅까지 전체 과정을 재검증한 것은 아니다.**

중계 자체의 회귀 테스트는 센서 없이 수행할 수 있다.

```powershell
python windows/test_udp_relay.py
```
