# PiCar-Pro 미션 코드 (MISSION 1~5)

Adeept PiCar-Pro 의 `Server/` 코드(Move.py, RPIservo.py, Ultra.py)를 재사용해서
미션 안내 PDF 의 5개 미션을 수행합니다. 미션별로 클래스가 나뉘어 있어서 한 미션만 골라
돌리거나, 팔 동작을 한 단계씩 확인할 수 있습니다.

```
mission/
├── config.py        모든 튜닝 상수 (속도, 거리, 서보 포즈, 동작 순서)  ← 여기만 고치면 됨
├── hardware.py      하드웨어 래퍼 (Drive / Servos / Arm / Ultrasonic / Gyro / Camera / QR)
├── missions.py      Mission1 ~ Mission5 클래스, PrecisionStopper(정밀 정지), PickPlaceTask(집기/놓기)
├── run_mission.py   실행 진입점 + 점검 도구 (--calibrate, --test-* ...)
├── debug_tool.py    디버그 웹 화면: 카메라/QR 인식 확인 + 서보 각도 조절 + 수동 주행(초음파 거리 표시)
└── logs/            실행 로그(run_*.log), 결과(result_*.json), 카메라 스냅샷(qr_last.jpg)
```

## 1. 설치

```bash
# 파이에서, 원하는 폴더 아무 데나 clone (예: 홈)
cd ~
git clone https://github.com/tjrud/phc26_2.git
cd phc26_2

sudo systemctl stop Adeept_Robot.service   # 부팅 시 뜨는 웹서버가 카메라/GPIO 를 잡고 있어서 먼저 정지
python3 run_mission.py --home              # 모터 초기화(바퀴 정지 + 서보 기본값)만 하고 종료 → 설치/동작 확인

# 코드를 고쳐서 다시 올렸으면 파이에서
git pull
```

Adeept 의 `Move.py`, `RPIservo.py`, `Ultra.py` 는 자동으로 찾습니다
(이 폴더의 상위 폴더 → `~/Adeept_PiCar-Pro/Server` → `/home/*/Adeept_PiCar-Pro/Server` 순, sudo 로 실행해도 동작).
Adeept 폴더가 다른 위치에 있으면 `config.py` 의 `SERVER_DIR` 에 경로를 적어 주세요.
어떤 폴더를 잡았는지 보려면 `python3 run_mission.py --home --log-level DEBUG` (로그에 `Adeept Server 폴더: ...`).

* 부팅 때마다 웹서버가 다시 뜨므로 매번 `stop` 하거나, `run_mission.py --stop-service` 로 자동 정지하세요.
  경기 끝나고 웹 UI 를 쓰려면 `sudo systemctl start Adeept_Robot.service`.
* 권한 오류(I2C/GPIO/카메라)가 나면 `sudo python3 run_mission.py ...` 로 실행.
* QR 인식률을 높이고 싶으면(선택): `sudo apt install libzbar0 && sudo pip3 install pyzbar --break-system-packages`
  → 있으면 자동으로 pyzbar 를 쓰고, 없으면 OpenCV 내장 QR 인식기를 씁니다.

## 2. 실행

```bash
python3 run_mission.py                         # MISSION 1 → 5 전체 (경기용)
python3 run_mission.py --only 2                # MISSION 2 만 (정밀 정지 튜닝)
python3 run_mission.py --only 1                # QR 인식만
python3 run_mission.py --from 3 --direction LEFT          # 정지선에 세워 두고 3,4,5 만 (QR/주행 생략)
python3 run_mission.py --only 3 --direction RIGHT --pause-steps   # 팔 동작을 Enter 로 한 단계씩
python3 run_mission.py --continue-on-error     # 한 미션이 실패해도 끝까지
```

실행하면 `logs/run_날짜.log` 에 전체 로그, `logs/result_날짜.json` 에 미션별 결과(정지 거리, 방향, 소요 시간)가 남습니다.

**실행하면 항상 모터 초기화부터.** 어떤 옵션으로 실행하든 가장 먼저 바퀴 모터를 정지하고 조향/팔 서보를 기본값으로 보냅니다.
기본값은 `Server/RPIservo.py` 의 `init_pwm0~7`(웹서버가 켜질 때 서보가 가는 위치)이고, 홈 포즈(`--home`, `--calibrate` 의 `h`)와
조향 중립도 같은 값을 씁니다. RPIservo.py 를 못 읽으면 `config.py` 의 `ARM_POSES["home"]` / 조향 90.
기본값 대신 config 값을 쓰려면 `USE_ADEEPT_INIT_AS_HOME = False`.

### 디버그 화면 (debug_tool.py)

QR 이 잘 안 읽히거나 팔 각도를 찾을 때 씁니다. 파이에서 실행하고, 같은 와이파이의 노트북/폰 브라우저로 접속합니다.
Adeept 웹 화면처럼 주행도 되므로 이 화면 하나로 주행·QR 확인·팔 각도 찾기를 다 할 수 있습니다
(카메라와 모터를 같이 쓰므로 `Adeept_Robot.service` 와 동시에는 못 켭니다. 먼저 정지하세요).

```bash
python3 debug_tool.py          # 터미널에 나오는 주소(http://파이IP:8000)를 브라우저로 열기. 끝내려면 Ctrl+C
```

* **카메라 + QR 인식**: 실시간 화면에 QR 테두리를 그립니다 (초록 = 읽힘, 노랑 = QR 은 보이는데 해독 실패).
  밝기·선명도·QR 크기(px)·연속 인식 횟수와 안 읽히는 이유 힌트가 나옵니다.
  **[분석]** 을 누르면 지금 화면을 여러 방법(확대, 대비 향상, 이진화, 좌우 반전 ...)으로 읽어 보고 어떤 방법이 되는지 표로 보여 줍니다.
* **카메라 조절**: ExposureValue(밝기 보정) / Contrast / Sharpness. 잘 읽히는 값이 나오면 화면의 `CAMERA_CONTROLS = {...}` 줄을 `config.py` 에.
* **주행**: ▲▼ 전진/후진, ◀▶ 좌우 조향 버튼은 누르고 있는 동안만 움직이고 떼면 멈춥니다(조향은 가운데로). 키보드 ↑↓←→, 스페이스 = 정지.
  속도 슬라이더 기본값은 `DRIVE_SPEED_FAST`. 와이파이가 끊기거나 탭을 닫아서 명령이 0.5초 동안 안 오면 자동으로 멈춥니다.
  주행은 한 화면에서만 하세요. 다른 화면에서 버튼을 누르고 있으면 여기서 정지해도 바로 다시 움직입니다.
* **초음파 거리**: 지금 거리와 정지선 기준 위치(빨간 선 / 노란 선 / 넘어감). **[펄스 ▲▼]** 는 정밀 정지에서 쓰는
  펄스 한 번(`CREEP_SPEED`, `CREEP_PULSE_S`)만큼 움직이고 이동 거리를 보여 줍니다.
* **서보**: steer / base / shoulder / wrist / grip 의 현재 각도, ± 스텝이나 숫자로 이동 (키보드 a/d, w/s, i/k, [ ], , . 도 가능).
  config 포즈로 이동, 집게 열기/닫기, base 좌/우, 모터 초기화.
* **config 에 붙여넣기**: 지금 각도를 `{CH_SHOULDER: 138, CH_WRIST: 105}` 같은 형식으로 보여 줍니다. 스펀지를 잡을 수 있는 각도를 찾으면
  이름을 적고 **[기록]** → `logs/debug_poses.txt` 에 쌓입니다.

## 3. 미션별 동작 (missions.py)

| 미션 | 클래스 | 동작 |
|---|---|---|
| 1 | `Mission1QRStart` | `scan_qr` 포즈 → 카메라로 QR 대기 → `Left`/`Right` 가 연속 2프레임 읽히면 방향 확정 → `drive` 포즈로 팔 정리 |
| 2 | `Mission2DriveAndStop` → `PrecisionStopper` | 31cm 까지 펄스 없이 연속 주행(FAST → 마지막 10cm 는 SLOW) → 정지 → 펄스로 앞/뒤 움직여 30±0.5cm 맞춤 → RED/YELLOW/OVER/SHORT 판정 |
| 3 | `Mission3FirstTransfer` → `PickPlaceTask` | QR 방향 상자의 물체 집기 → 반대편 상자에 넣기 (`config.PICK_PLACE_SEQUENCE` 순서대로) |
| 4 | `Mission4SecondTransfer` → `PickPlaceTask` | 반대편 물체 집기 → 처음 방향 상자에 넣기 |
| 5 | `Mission5Finish` | 팔 `finish` 포즈, 모터 정지, 결과 요약/저장 → 조교 확인 후 팀원이 종료 버튼 |

QR 실제 내용: 출발 QR 은 `Left` / `Right`, 상자 QR 은 `PICK` / `PLACE` 입니다. 코드는 대소문자를 무시하고 비교합니다.

## 4. 튜닝 순서 (처음 세팅할 때 이 순서대로)

1. **모터 초기화 확인** `python3 run_mission.py --home` — 바퀴가 멈춰 있고 조향/팔이 기본값 자세로 가는지. 조향이 한쪽으로 치우치면 `STEER_CENTER_TRIM`.
2. **포즈 각도 찾기** `python3 debug_tool.py` (브라우저에서 카메라를 보면서) 또는 `python3 run_mission.py --calibrate`
   키보드로 서보를 움직이며(a/d base, w/s shoulder, i/k wrist, [/] grip) 각 자세를 만들고 `p` 를 누르면
   `{CH_BASE: 95, CH_SHOULDER: 85, ...}` 형식으로 출력됩니다 → `config.ARM_POSES` 에 붙여넣기.
   찾아야 할 것: `scan_qr`, `drive`, `carry`, `pick_approach`, `pick_grab`, `place_drop`, `finish`,
   `BASE_LEFT` / `BASE_RIGHT`, `GRIP_OPEN` / `GRIP_CLOSED`. 좌우가 대칭이 아니면 `ARM_POSE_SIDE_OVERRIDE`.
3. **포즈 확인** `python3 run_mission.py --pose pick_grab --side LEFT`
4. **QR 인식 확인** `python3 debug_tool.py` — 출발선에서 모니터 QR 이 초록 테두리로 읽히는지. 노랑이면 [분석] 과 카메라 조절,
   화면에 QR 이 없으면 `scan_qr` 포즈(팔을 앞으로 뻗어 카메라를 모니터에 가깝게)나 `CAMERA_SIZE` 조정.
   (터미널만 쓸 때는 `python3 run_mission.py --test-qr` → `logs/qr_last.jpg`)
5. **초음파 보정 (가장 중요)** `python3 run_mission.py --measure-stopline`
   로봇의 초음파 센서를 빨간 정지선 위에 정확히 놓고 실행 → 출력되는 `ULTRA_OFFSET_CM` 값을 config 에 입력.
   (노트북의 어느 면에 초음파가 반사되느냐에 따라 측정값이 30 이 아닐 수 있어서 반드시 필요)
6. **속도/펄스 확인** `--test-drive 25 1` (SLOW 속도에서 cm/s), `--test-pulse 5` (펄스당 이동 거리 0.5~1cm 가 되도록 `CREEP_SPEED`, `CREEP_PULSE_S`).
   `debug_tool.py` 의 [펄스 ▲▼] 로도 한 번씩 움직여 보며 확인할 수 있습니다.
7. **정밀 정지 반복** `--only 2` 를 여러 번 → 로그의 `[APPROACH] 정지 명령 ...cm → 멈춘 뒤 ...cm (밀림 x.xcm)` 의 밀림 값을
   `APPROACH_LEAD_CM` 에 넣으면 연속 주행이 31cm 근처에서 멈추고, 펄스는 1~2번이면 끝납니다.
   31cm 보다 더 들어가서 멈추면 `APPROACH_LEAD_CM` 을 키우세요 (정지선을 넘으면 실격).
8. **집기/놓기** `--from 3 --direction LEFT --pause-steps` 와 `--direction RIGHT` 양쪽 모두 단계별로 확인.
9. **전체 리허설** `python3 run_mission.py`

## 5. 경기 당일 절차

1. `sudo systemctl stop Adeept_Robot.service`
2. 로봇을 출발선에: **초음파 센서가 빨간 출발선 위**, 뒤쪽 검은 가이드라인으로 방향 맞추기
3. `python3 run_mission.py` → 로그에 `READY ▶ 출발선에서 QR 대기 중` 이 뜨면 준비 완료
4. 다른 팀원이 시작 버튼 → 5초 후 QR 표시 → 로봇이 인식하고 자동 진행
5. 끝나면 `모든 로봇 동작 완료` 로그 → 조교 확인 → 종료 버튼
6. 문제가 생기면 **Ctrl+C** (모터 즉시 정지, 팔은 그 자리 유지)

## 6. 규칙과 코드의 관계 (PDF 주의사항)

* 정지 후 팔이 움직인 뒤 바퀴가 움직이면 실격 → MISSION 3 부터는 모터 명령이 전혀 없습니다. 위치 보정(후진 포함)은 전부 MISSION 2 안에서 끝납니다.
* 정지선을 넘으면 실격 → `APPROACH_LEAD_CM` 은 넉넉히, `ALLOW_REVERSE_FIX` 로 살짝 지나친 건 되돌립니다(`REVERSE_FIX_LIMIT_CM` 이내만).
* 팔이 모니터에 닿으면 실격 → `drive` / `scan_qr` 포즈 높이를 확인하세요.
* 잘못된 상자에 넣으면 실격 → `BASE_LEFT` 가 실제로 로봇 기준 **왼쪽** 상자를 향하는지 `--pose` 로 꼭 확인.
* QR 인식 후에 움직여야 함 → `scan_qr` 포즈는 시작 버튼 전에 잡고, 인식 후에 팔 정리·주행을 시작합니다.

## 7. 자주 겪는 문제

| 증상 | 확인할 것 |
|---|---|
| 카메라 열기 실패 / GPIO busy | `Adeept_Robot.service` 정지했는지 (`--stop-service`) |
| 전진하라는데 후진 | `Server/Move.py` 의 `M1_Direction`, `M2_Direction` (현재 둘 다 1 로 수정돼 있음) |
| 조향 좌우 반대 | `STEER_RIGHT_SIGN = -1` |
| 팔이 반대쪽 상자로 감 | `BASE_LEFT` / `BASE_RIGHT` 값 교환 |
| creep 에서 로봇이 안 움직임 (로그: "안 움직임") | `CREEP_SPEED` 올리기 (모터 데드존), `CREEP_PULSE_S` 늘리기 |
| 정지선을 지나침 | `APPROACH_LEAD_CM` ↑ (로그의 밀림 값), `DRIVE_SPEED_SLOW` ↓, `ULTRA_OFFSET_CM` 재측정 |
| 팔이 물체/상자까지 안 닿음 | `pick_approach` / `pick_grab` / `place_drop` 의 shoulder ↑(팔을 내리며 앞으로), wrist ↑(손을 들어 앞으로). `--calibrate` 의 s / i 키로 맞추고 p 로 붙여넣기 |
| 초음파 값이 튐 (None / 150 이상) | 노트북 면 각도, `ULTRA_MAX_VALID_CM`, `ULTRA_SAMPLES` ↑ |
| QR 이 안 읽힘 | `debug_tool.py` 의 힌트와 [분석], `scan_qr` 포즈, 조명 반사, pyzbar 설치, `CAMERA_CONTROLS`, `CAMERA_SIZE` |
| 물체를 넘어뜨림 | `SERVO_STEP_DELAY_S` ↑ (천천히), `pick_approach` → `pick_grab` 사이 경로 조정 |
| 프로그램이 죽었는데 바퀴가 계속 돎 | 자동 정지는 `debug_tool.py` 가 살아 있어야 동작합니다. 전원을 끄거나 `python3 run_mission.py --home` (바퀴 정지. 조향과 팔도 기본값으로 움직이니 주의) |
| 자이로 보정 켰더니 더 휨 | `--test-gyro` 로 부호 확인 (`GYRO_YAW_SIGN`), 아니면 `USE_GYRO_HEADING_HOLD = False` |
