# SOSLAB GL5 ROS 2 실습

Windows 11 Pro + Docker Desktop(Hyper-V, Linux 컨테이너) 기준.
ROS 2·SDK·드라이버는 Docker 이미지 안에서 설치·빌드됩니다.

## 1. Python·VcXsrv 설치 (없는 경우, 최초 1회)

**Windows PowerShell에서 실행합니다. 현재 폴더는 어디든 괜찮습니다.**
Python은 Windows UDP 중계용, VcXsrv는 RViz 화면 출력용입니다.

```powershell
winget install --id Python.Python.3.12 --exact --source winget
winget install --id marha.VcXsrv --exact --source winget
```

설치 후 PowerShell을 새로 열고 실행 경로를 확인합니다.

```powershell
py -3.12 -c "import sys; print(sys.executable)"
Test-Path "C:/Program Files/VcXsrv/vcxsrv.exe"
```

첫 명령의 출력이 Python 경로입니다. 두 번째가 `True`이면 해당 VcXsrv 경로를 사용합니다.
`False`이면 실제 설치 폴더에서 `vcxsrv.exe` 경로를 확인합니다.

## 2. 프로젝트·이미지 준비

저장소가 없다면 PowerShell에서 클론합니다.

```powershell
git clone --branch windows --recurse-submodules https://github.com/churrosboy/lidar_workshop.git
```

Docker Desktop을 켜고 실행합니다. `<Windows 브랜치 폴더>`를 실제 경로로 바꾸세요.
아래 명령으로 실습용 이미지를 최초 1회 빌드합니다.

```powershell
cd "<Windows 브랜치 폴더>"  # Dockerfile이 있는 폴더, 예: C:/work/lidar_workshop
docker build --platform linux/amd64 -t lidar-workshop:humble-amd64 .
Copy-Item windows/settings.example.json windows/settings.json  # 최초 1회만

notepad windows/settings.json # 수정 필요할 시
```

센서 IP·포트와 위에서 확인한 실행 경로를 입력합니다. 경로는 `/`를 사용합니다.
아래 두 항목은 기존 JSON에서 수정합니다(사용자 이름은 실제 값으로 변경).

```json
"python_exe": "C:/Users/사용자이름/AppData/Local/Programs/Python/Python312/python.exe",
"vcxsrv_exe": "C:/Program Files/VcXsrv/vcxsrv.exe"
```

기본 연결: 센서 `10.110.1.2:2000` → PC `10.110.1.3:3000`.

## 3. 유선 LAN 설정 (최초 1회)

센서와 PC를 유선 연결합니다. **PowerShell을 관리자 권한으로 열어** 실행합니다.
`16`은 센서가 연결된 유선 LAN의 실제 `ifIndex`로 바꾸세요.
해당 어댑터의 고정 IP·방화벽을 설정하며 Wi-Fi는 유지합니다.

```powershell
cd "<Windows 브랜치 폴더>"
Get-NetAdapter
powershell.exe -NoProfile -ExecutionPolicy Bypass -File windows/configure-network.ps1 -InterfaceIndex 16 # Get-NetAdapter로 얻은 숫자로 실행
ping 10.110.1.2
```

`ping`은 IP 응답 확인용입니다. 실제 데이터 수신은 아래 `/scan` 주기로 확인합니다.

## 4. 실행·수신 확인

**일반 PowerShell**, 프로젝트 폴더에서 실행합니다. Docker Desktop이 켜져 있어야 합니다.

```powershell
.\windows\start-lidar.cmd
```

RViz: **Draw Region → 꼭짓점 3개 이상 클릭 → Finish Region**.
영역은 `windows/.local/gl5_region.json`에 자동 저장됩니다.

**다른 PowerShell 창**에서 실제 스캔 수신 주기를 확인합니다.

```powershell
docker exec lidar-workshop /ros_entrypoint.sh ros2 topic hz /scan
```

## 5. 종료

실습 실행 터미널에서 **Ctrl+C**로 드라이버·RViz를 종료합니다.
컨테이너까지 정지하려면 PowerShell에서 실행합니다.

```powershell
docker stop lidar-workshop
```

## 패키지 구조

| 패키지 | 역할 |
|---|---|
| `gl5_driver` | 센서 수신, `/scan`·`/points` 발행 |
| `gl5_detection` | 감지 영역 편집, 장애물 감지·추적 |
| `gl5_rviz_plugins` | RViz 영역 설정 패널 |
| `gl5_bringup` | 전체 노드 실행과 RViz 설정 |
