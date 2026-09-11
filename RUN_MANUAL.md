# GL5 장애물 감지 실행 매뉴얼

현재 PC: Ubuntu 22.04 / ROS 2 Humble. 아래 명령은 이 PC의 터미널에서 실행합니다. 사진 저장 없이 실시간 화면으로 실습합니다.

## 1. 유선 IP 확인

```bash
cd /home/dhkim/soslab/ros2_ws
bash scripts/network.sh show
```

`enp0s31f6`에 `10.110.1.3/24`가 있으면 다음 단계로 진행합니다. 이 장치에서 수신이 확인된 설정은 다음과 같습니다.

| 항목 | 주소 |
|---|---|
| 라이다 | `10.110.1.2:2000` |
| PC | `10.110.1.3:3000` |

PC 주소가 없을 때만 아래 명령을 실행합니다.

```bash
bash scripts/network.sh up
```

프로필이 이미 있다는 메시지가 나오면 기존 프로필을 활성화합니다.

```bash
nmcli connection up gl5-lab-temp
```

## 2. 빌드

최초 실행 또는 소스 코드를 수정한 경우 실행합니다. 이미 빌드했다면 생략합니다.

```bash
cd /home/dhkim/soslab/ros2_ws
bash scripts/build.sh
```

## 3. 라이다·감지 노드·RViz 실행

기존 실행 터미널이 있다면 먼저 `Ctrl+C`로 종료합니다. 같은 라이다에 연결하는 SDK 프로그램과 중복 실행하지 않습니다.

```bash
cd /home/dhkim/soslab/ros2_ws
export ROS_DOMAIN_ID=42
bash scripts/run.sh params_file:=/home/dhkim/soslab/ros2_ws/src/gl5_driver/config/gl5.yaml
```

실행 터미널을 열어 둡니다. RViz에서 스캔 점이 보이면 수신 중입니다. 저장된 영역이 있으면 자동으로 불러와 감지합니다.

## 4. 감지 영역 지정

1. 왼쪽 `GL5 Region Controls` 패널에서 **영역 지정**을 누릅니다.
2. 화면에서 원하는 영역의 둘레를 따라 꼭짓점을 **3개 이상** 클릭합니다. 처음에는 사각형의 네 모서리를 순서대로 선택하세요.
3. **영역 확정**을 누릅니다. 첫 꼭짓점 근처를 다시 클릭해도 확정됩니다.
4. 확정된 영역 안에 물체가 들어오면 빨간 박스와 라벨이 표시됩니다.

변이 서로 교차하면 확정되지 않습니다. 편집 중에는 장애물 판정이 중지됩니다.

| 버튼 | 동작 |
|---|---|
| 마지막 점 취소 | 가장 최근에 찍은 꼭짓점 제거 |
| 편집 취소 | 새 영역 편집을 취소하고 이전 적용 영역으로 복귀 |
| 영역 전체 삭제 | 현재 영역·편집 점·저장된 영역을 한 번에 삭제 |
| 저장 영역 불러오기 | 저장 파일의 영역 불러오기 |
| 카메라 조작 (확대·축소) | 화면 조작 도구로 전환. 이후 마우스 휠로 확대·축소 |

패널이 없다면 `Panels → Add New Panel → gl5_driver/RegionPanel`을 추가합니다. 영역은 확정할 때 `config/gl5_region.json`에 저장됩니다.

## 5. 박스와 상태 읽기

라벨 예시:

```text
#12
0.36x0.10
0.25m/s
```

- `#12`: 추적 ID. 장애물 개수가 아니며 새로운 물체로 판단하면 새 번호를 부여합니다.
- `0.36x0.10`: 관측된 점들의 X·Y 방향 크기(m). 물체 전체 크기나 높이는 아닙니다.
- `0.25m/s`: 박스 중심 이동으로 추정한 센서 기준 속도. 초기에는 `--m/s`로 표시될 수 있습니다.

센서를 고정해 사용하세요. 군집 분리·합체나 가림에 따라 ID와 속도가 흔들릴 수 있습니다.

| 상태 | 의미 |
|---|---|
| NO_REGION | 적용된 영역 없음 |
| EDITING | 영역 편집 중 |
| NO_DATA | 유효한 스캔이 없거나 수신이 끊김 |
| CLEAR | 영역 안에서 조건을 만족하는 군집 없음 |
| OCCUPIED | 영역 안에서 장애물 감지가 설정 시간 이상 지속됨 |

## 6. 클러스터링 파라미터 조절

수정할 파일: `src/gl5_driver/config/obstacles.yaml`

아래 값은 매뉴얼 작성 시 파일 기준입니다. **수정 후 실행 터미널에서 `Ctrl+C`를 누르고 3단계 명령으로 다시 실행**해야 적용됩니다. 이 YAML 변경에는 재빌드가 필요 없습니다.

| 파라미터 | 현재 값 | 조절 효과 |
|---|---:|---|
| cluster_gap | 0.30 | 인접 점 연결 거리(m). 키우면 덜 쪼개지지만 가까운 물체가 합쳐질 수 있음 |
| min_points | 5 | 군집의 최소 점 수. 키우면 작은 잡음과 작은 물체가 제외됨 |
| track_match_distance | 0.4 | 이전 추적과 연결할 최대 거리(m) |
| track_max_age | 0.5 | 사라진 추적을 유지하는 시간(s) |
| speed_window | 0.4 | 속도 계산 시간창(s). 키우면 부드러워지지만 반응이 느려짐 |
| enter_delay | 0.2 | OCCUPIED 전환에 필요한 감지 지속 시간(s) |
| exit_delay | 0.5 | CLEAR 전환에 필요한 미감지 지속 시간(s) |
| data_timeout | 1.0 | 수신 중단 판단 시간(s) |
| label_height | 0.14 | 라벨 글자 높이(m) |
| close_radius | 0.15 | 첫 꼭짓점 클릭으로 영역을 닫는 허용 반경(m) |

한 물체가 여러 박스로 갈라지면 `cluster_gap`부터 조절합니다. **중간에 무효 측정점이나 영역 밖 점이 있으면 현재 구현은 군집을 끊습니다.** 이 경우 거리 값만 키워도 연결되지 않을 수 있습니다. `min_points`는 군집을 합치는 기능이 아닙니다.

## 7. 수신 확인과 문제 해결

별도 터미널에서 실행합니다. 실행 터미널과 같은 ROS 도메인을 사용해야 합니다.

```bash
cd /home/dhkim/soslab/ros2_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic hz /scan
```

기존 실기 검증에서는 약 40 Hz가 확인됐습니다. 확인을 마치면 `Ctrl+C`로 이 명령만 종료합니다.

감지 상태 확인:

```bash
bash scripts/region.sh status
```

RViz 대신 터미널에서 영역을 전체 삭제하려면:

```bash
bash scripts/region.sh clear
```

| 증상 | 확인할 항목 |
|---|---|
| 스캔이 안 보임 | PC IP, 실행 터미널의 연결 오류, 다른 SDK/드라이버 중복 실행 여부 |
| 점은 보이는데 박스가 없음 | 영역 확정 여부, 영역 안의 점 존재 여부, `min_points` |
| 영역·박스 모두 안 보임 | RViz `Detection region and obstacles` 표시 활성화, Fixed Frame=`laser` |
| 확대·축소가 안 됨 | 패널의 카메라 조작 버튼을 누른 뒤 휠 사용. 또는 `Views → Scale` 조절 |
| YAML 변경이 적용 안 됨 | 감지 노드 재시작 여부, 다른 YAML을 실행 인자로 지정했는지 확인 |

## 8. 종료

3단계 실행 터미널에서 `Ctrl+C`를 누릅니다. 함께 실행한 드라이버·감지 노드·RViz가 종료됩니다. 확정한 영역은 다음 실행에도 유지됩니다.

유선 임시 프로필까지 삭제하려는 경우에만 다음을 실행합니다.

```bash
bash scripts/network.sh down
```
