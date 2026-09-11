# Validation record

실행일: 2026-09-11, Ubuntu 22.04 / ROS 2 Humble

## 현재 결과: 실제 수신 및 영역 장애물 검증 통과

- 박스 옆 빨간 크기·속도 라벨 추가. 센서 좌표의 X/Y 관측 폭과 박스 중심 이동 속도를 표시하며 0.4초 회귀 및 단기 ID 연결을 사용함. 추적 단위 테스트 5개(정지·기지 속도 2 m/s·입력 순서 변화·소멸·가림 후 초기화) 통과. ROS 통합 테스트에서 박스별 라벨 개수·빨간색·작은 글자 크기·크기와 속도 단위·박스 삭제 시 라벨 제거 검증. `artifacts/roi_tracking_integration.log`.

- 조작 UI를 `gl5_driver/RegionPanel` RViz 고정 패널로 변경. 플러그인 빌드와 RViz 로딩 성공, `/gl5/obstacle_state` 수신용 `gl5_region_panel` 노드 생성 확인. 화면의 메뉴 마커와 상태 문구 표시를 제거하고 지정·확정·전체 삭제·Undo·Cancel·Load·카메라 전환 버튼 및 상태를 패널로 이동. 기존 메뉴 서버는 호환성을 위해 남아 있으나 기본 RViz 화면에는 표시하지 않음.

- 실제 연결: GL5 `10.110.1.2:2000`, PC `10.110.1.3:3000`. `2020`은 UDP 연결 거부, `2000`은 스트리밍 시작·종료 응답 정상.
- SDK: 약 40 Hz, 프레임당 1,500개 샘플 수신.
- ROS 2 60초 검증: `/scan` 2,399개, `/points` 2,398개, 좌표·intensity 일치 2,398쌍, 검사 오류 0개. `artifacts/hardware_20260911_191024/ros.json`.
- rosbag: 각 토픽 472개, 총 944개 메시지 저장. `artifacts/hardware_20260911_191024/bag/`.
- 영역 기하·시간 판정 단위 테스트 8개 통과, 기존 스캔 변환 테스트 5개 통과.
- 독립 ROS 도메인 91 통합 테스트 통과: 클릭으로 영역 닫기·저장·불러오기, 잘못된 영역 거부, 영역 밖 물체 제외, 진입·해제 지연, 수신 중단 및 전부 무효인 프레임에서 NO_DATA 전환과 박스 제거. `artifacts/roi_integration.log`.
- 실제 GL5 스캔을 이용한 영역 판정 통과: 전체 관측점을 포함하는 진단 영역에서 `OCCUPIED`와 박스 43개, 관측 범위 밖 진단 영역에서 `CLEAR`와 박스 0개. 이 진단은 사용자 영역 파일을 변경하지 않았으며, 특정 실물 상자를 이동시키는 테스트와는 별개임. `artifacts/roi_live_verification.json`.
- RViz 연결 확인: `Publish Point`의 `/clicked_point`를 장애물 노드가 구독하고, RViz가 `/gl5/obstacle_markers`를 구독함. ROS 2 MarkerArray의 토픽 설정 키를 `Topic`으로 수정. 사용자 요청에 따라 추가 화면 캡처는 수행하지 않음.
- RViz Interactive Marker 메뉴 추가: 전체 삭제·영역 확정·편집·점 취소·편집 취소. 도메인 91 통합 테스트에서 실제 메뉴 정보 조회 및 MENU_SELECT 피드백으로 삭제 후 `NO_REGION`, 박스 0개, 저장 파일 빈 영역을 검증함. `artifacts/roi_menu_test.log`.
- 사용자 지정 영역 편집 중에는 `EDITING`, 지정 전에는 `NO_REGION`이므로 장애물 박스가 없음. 첫 점 근처 클릭 또는 `bash scripts/region.sh finish`로 확정.

실물의 전방·좌우 방향 및 상자를 넣고 빼는 비교는 사용자가 선택한 영역에서 확인할 항목이다. 현재 기능은 관측 점 기반 구역 점유 판정이며 사람/상자 분류나 물체 추적을 수행하지 않는다.

## 통과

- SDK Release 빌드: `SOSLAB_SDK/_archive_/lib/libLidar_x64_release.so` 생성.
- 독립 실행 파일 빌드: `build/gl5_receive` 생성.
- ROS 2 패키지 빌드: `gl5_driver` 설치 완료.
- 변환 단위 테스트: 5개 테스트, 0 errors, 0 failures (`artifacts/test_summary.txt`).
- ROS 2 launch/RViz 설정 로딩: RViz 창이 열리고 `Grid (1 m)`, `Sensor axes`, `GL5 scan`, `GL5 points`가 표시됨 (`artifacts/rviz_no_data.png`).
- 무수신 검증 도구: 실제 토픽이 없을 때 실패로 판정하고 원인을 기록함 (`artifacts/no_data_verification.json`).

## 초기 진단 기록 (주소 확인으로 해결됨)

센서 IP 후보 `192.168.1.10`, PC `192.168.1.15`를 임시 유선 프로필로 설정했다. UDP 2000과 2020을 각각 시험했으나 SDK 스트리밍 응답과 프레임이 없었다.

- 2000: `start_ack=false`, `frames=0` (`artifacts/probe_2000.json`)
- 2020: `start_ack=false`, `frames=0` (`artifacts/probe_2020.json`)
- `192.168.1.0/24`의 제한된 후보 탐색에서도 ARP 응답을 확인하지 못함.
- 유선 인터페이스는 `enp0s31f6`이며 링크는 UP이지만 장치가 NetworkManager에 연결된 상태는 아니었다.

위 초기 후보 주소에서는 수신되지 않았다. 이후 실제 IP를 확인해 현재 결과처럼 수신·발행 검증이 통과했다. 올바른 설정은 YAML에 반영되어 있으며 재검증은 다음 명령으로 수행한다.

```bash
bash scripts/network.sh show
sudo timeout 15 tcpdump -i enp0s31f6 -nn -c 20 'arp or udp'
bash scripts/test_hardware.sh /home/dhkim/soslab/ros2_ws/src/gl5_driver/config/gl5.yaml
```

`tcpdump`에는 관리자 권한이 필요하다. 패킷이 보이면 YAML의 `sensor_ip`, `sensor_port`, `pc_ip`, `pc_port`를 실제 값으로 바꾼다. 장치가 고정 목적지 포트를 사용하면 `pc_port: 0` 대신 그 포트를 지정한다.
