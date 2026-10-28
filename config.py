# -*- coding: utf-8 -*-
"""
config.py - 미션 튜닝 상수 모음
=================================
로봇/경기장에 맞춰 바꿀 값은 전부 이 파일에만 있습니다.
(hardware.py / missions.py / run_mission.py 는 건드리지 않아도 됩니다)

서보 각도 방향 기준 (Adeept WebServer.py 의 버튼 동작과 동일, 기본 위치 90도):
  ch0 steer    : 조향.  WebServer 기준 left = 90 - 30, right = 90 + 30  (STEER_RIGHT_SIGN 으로 반전)
  ch1 base     : 각도 ↑ = 팔 왼쪽 회전(lookleft)   / 각도 ↓ = 오른쪽(lookright)
  ch2 shoulder : 각도 ↓ = 팔 올리기(armup)         / 각도 ↑ = 내리기(armdown)
  ch3 wrist    : 각도 ↑ = 손 올리기(handup)        / 각도 ↓ = 내리기(handdown)
  ch4 grip     : 각도 ↓ = 잡기(grab)               / 각도 ↑ = 벌리기(loose)
  ※ 조립 상태에 따라 반대일 수 있으니  python3 run_mission.py --calibrate  로 직접 확인하세요.

경기장 치수 (PDF):
  총길이 109cm, 폭 39.4cm / 출발선: 뒤쪽 끝에서 22cm / 정지선: 모니터 앞 30cm
  정지선 = 빨간 2cm (만점) + 양옆 노란 2cm (감점) / 정지선을 넘어가면 실격
  상자: 정지선 기준 앞뒤 10cm, 주행로 좌우 바깥 15cm
  출발 QR: 'Left' / 'Right' (15x15cm, 모니터)   상자 QR: 'PICK' / 'PLACE' (7x7cm)
"""
import os

# =====================================================================
# 1. 경로 / 로그
# =====================================================================
# Adeept Server 폴더 (Move.py, RPIservo.py, Ultra.py 가 있는 곳). 이 저장소는 어디에 clone 해도 됩니다.
# None 이면 자동 탐색: 이 폴더의 상위 폴더 → 이 폴더 → ~/Adeept_PiCar-Pro/Server (sudo 면 원래 사용자 홈도)
#                    → /home/*/Adeept_PiCar-Pro/Server
# Adeept 폴더가 다른 곳에 있을 때만 지정.  예) SERVER_DIR = "/home/pi/robot/Adeept_PiCar-Pro/Server"
SERVER_DIR = None

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
ROBOT_SERVICE_NAME = "Adeept_Robot.service"   # 부팅 시 자동 실행되는 웹서버 (카메라/GPIO 점유 → 실행 전 정지 필요)

# =====================================================================
# 2. QR 코드
# =====================================================================
# 디코딩 결과는 strip().upper() 해서 비교합니다.
# 실제 QR 파일 디코딩 결과: 출발 'Left' / 'Right',  상자 'PICK' / 'PLACE'
QR_START_MAP = {            # 읽힌 문자열(대문자) → 방향
    "LEFT": "LEFT",
    "RIGHT": "RIGHT",
}
QR_PICK_TEXT = "PICK"
QR_PLACE_TEXT = "PLACE"

QR_CONFIRM_COUNT = 2        # 같은 방향이 연속으로 n프레임 읽혀야 확정 (오인식 방지)
QR_WAIT_TIMEOUT_S = 0       # 출발 QR 대기 최대 시간(초). 0 = 무제한 (시작 버튼 5초 후 표시되므로 보통 무제한)
QR_POLL_INTERVAL_S = 0.05   # 프레임 사이 대기
QR_STATUS_LOG_EVERY_S = 5.0 # 대기 중 상태 로그 주기
QR_SAVE_DEBUG_FRAME_S = 3.0 # 대기 중 logs/qr_last.jpg 저장 주기 (0 = 저장 안 함). 카메라가 뭘 보는지 확인용
QR_TRY_UPSCALE = True       # 인식 실패 시 1.5배 확대해서 재시도 (멀리 있는 작은 QR용)

# 상자 PICK / PLACE QR 확인:  "off" = 안 함, "log" = 보이는지 기록만, "require" = 안 보이면 미션 실패
QR_BOX_CHECK_MODE = "log"
QR_BOX_CHECK_TIMEOUT_S = 1.0

# =====================================================================
# 3. 카메라
# =====================================================================
CAMERA_BACKEND = "auto"     # "auto" | "picamera2" (파이 카메라) | "usb" (USB 웹캠, cv2.VideoCapture)
CAMERA_SIZE = (1280, 720)   # QR 이 멀리(출발선→모니터 약 87cm) 있으므로 640x480 보다 크게
CAMERA_HFLIP = 0
CAMERA_VFLIP = 0
USB_CAMERA_INDEX = 0
CAMERA_WARMUP_S = 0.8
CAMERA_CONTROLS = {}        # picamera2 컨트롤. debug_tool.py 의 "카메라 조절"에서 QR 이 잘 읽히는 값을 찾아 넣으세요
                            # 예) {"ExposureValue": -1.0, "Contrast": 1.5, "Sharpness": 2.0}

# =====================================================================
# 4. 주행 / 정밀 정지  (MISSION 2)
# =====================================================================
STOP_TARGET_CM = 30.0       # 모니터 앞 정지 목표 (초음파 센서 기준)
STOP_TOLERANCE_CM = 0.5     # |측정 - 목표| 가 이 값 이하면 정지 완료
RED_HALF_WIDTH_CM = 1.0     # 빨간선 반폭 (폭 2cm)  → 29~31cm = 만점
YELLOW_HALF_WIDTH_CM = 3.0  # 노란선 바깥 경계      → 27~33cm = 감점, 27 미만 = 실격(넘어감)

# 접근: APPROACH_CM 까지는 펄스 없이 연속 주행으로 빠르게 → 정지 → 펄스(creep)로 앞/뒤 움직여 STOP_TARGET_CM 에 맞춤
APPROACH_CM = 31.0          # 연속 주행으로 다가갈 거리
APPROACH_LEAD_CM = 4.5      # 정지 명령 뒤 더 밀리는 거리 (초음파 지연 + 관성). APPROACH_CM + 이 값을 읽으면 모터 정지.
                            # 로그의 "[APPROACH] ... 밀림 x.xcm" 값을 넣으면 APPROACH_CM 에 맞춰 섭니다 (더 들어가면 키우세요)
DRIVE_SPEED_FAST = 40       # 0~100
DRIVE_SPEED_SLOW = 25       # 정지 직전 SLOW_ZONE_CM 구간 속도 (연속 주행. 느릴수록 밀림이 작고 일정)
SLOW_ZONE_CM = 10.0         # 정지 명령 지점(APPROACH_CM + APPROACH_LEAD_CM) 10cm 앞부터 SLOW. 0 = 끝까지 FAST (밀림↑)
DRIVE_LOOP_S = 0.03         # 주행 루프 주기
DRIVE_TIMEOUT_S = 20.0      # 각 주행 단계 최대 시간 (초과 시 비상 정지)
STOP_SETTLE_S = 0.6         # 정지 후 센서값 안정 대기

# 펄스(creep): 아주 짧게 전진/후진 → 정지 → 측정 반복
CREEP_SPEED = 30            # 모터가 실제로 움직이기 시작하는 최소값보다 커야 함
CREEP_PULSE_S = 0.08        # 펄스 길이 (한 번에 약 0.5~1cm 움직이도록 --test-pulse 로 조절)
CREEP_PULSE_MIN_S = 0.04    # 목표를 지나쳐 방향이 바뀔 때마다 펄스를 줄임 (하한)
CREEP_PULSE_MAX_S = 0.25    # 안 움직일 때 자동으로 늘어나는 상한
CREEP_SHRINK = 0.6          # 방향 반전 시 펄스 길이 배율
CREEP_STALL_CM = 0.2        # 펄스 후 이동이 이보다 작으면 "안 움직임" (2회 연속이면 펄스 x1.5)
CREEP_FAR_MULTIPLIER = 2.0  # 목표까지 3cm 이상 남았으면 펄스 길이 x2
CREEP_SETTLE_S = 0.7        # 펄스 후 측정까지 대기 (gpiozero 가 9샘플 중앙값을 쓰므로 0.6s 이상 권장)
MAX_CREEP_PULSES = 40
ALLOW_REVERSE_FIX = True    # 목표보다 더 들어갔을 때 살짝 후진 허용 (팔이 움직이기 전까지만 가능)
REVERSE_FIX_LIMIT_CM = 2.5  # 이보다 더 넘어갔으면(실격 영역) 후진 시도 안 함

# 초음파
ULTRA_MODE = "adeept"       # "adeept" = Server/Ultra.py 재사용 (9샘플 중앙값, 반응 느림)
                            # "fast"   = 같은 핀으로 직접 생성 (queue 짧게, 반응 빠름)
ULTRA_TRIG_PIN = 23
ULTRA_ECHO_PIN = 24
ULTRA_FAST_QUEUE_LEN = 3
ULTRA_OFFSET_CM = 0.0       # 보정값: (자로 잰 실제 거리) - (센서 측정값) 을 넣으세요
ULTRA_SAMPLES = 3           # read() 시 중앙값 샘플 수
ULTRA_SAMPLE_GAP_S = 0.07
ULTRA_MIN_VALID_CM = 2.0
ULTRA_MAX_VALID_CM = 150.0  # 이 범위 밖 값은 무시 (에코 못 받으면 200cm 가 나옴)

# 조향 서보
STEER_CENTER_TRIM = 0       # 직진인데 한쪽으로 쏠리면 +/- 몇 도 보정
STEER_RIGHT_SIGN = 1        # 서보각 = 90 + trim + sign*deg  (deg>0 = 우회전). 좌우 반대면 -1
STEER_MAX_DEG = 30

# 자이로 직진 보정 (MPU6050). 처음엔 False 로 두고,  --test-gyro 로 축/부호 확인 후 켜세요.
USE_GYRO_HEADING_HOLD = False
GYRO_ADDR = 0x68
GYRO_YAW_AXIS = "z"
GYRO_YAW_SIGN = 1           # 로봇을 손으로 왼쪽(반시계)으로 돌렸을 때 yaw 가 + 가 되도록
GYRO_KP = 0.8               # yaw 1도당 조향 보정 각도
GYRO_MAX_STEER = 8          # 보정 조향 상한 (도)
GYRO_BIAS_SAMPLES = 100     # 정지 상태 바이어스 측정 샘플 수 (약 1초)

# =====================================================================
# 5. 서보 채널 / 로봇팔 포즈  (MISSION 1, 3, 4)
# =====================================================================
CH_STEER, CH_BASE, CH_SHOULDER, CH_WRIST, CH_GRIP = 0, 1, 2, 3, 4
CH_NAMES = {CH_STEER: "steer", CH_BASE: "base", CH_SHOULDER: "shoulder", CH_WRIST: "wrist", CH_GRIP: "grip"}

# ---- 서보 기본값 = 파이카 기본값 (Server/RPIservo.py 의 init_pwm0~7, 웹서버가 켜질 때 서보가 가는 위치) ----
# run_mission.py 는 실행하자마자 모터 초기화(바퀴 정지 → 조향/팔 서보를 이 기본값으로 즉시 이동)부터 합니다.
# 홈 포즈(--home, --calibrate 의 h)와 조향 중립도 같은 값.
USE_ADEEPT_INIT_AS_HOME = True   # True: RPIservo.py 의 init_pwm 값 사용 (못 읽으면 ARM_POSES["home"] / 조향 90)

SERVO_FAST_DRIVER = True    # True: PCA9685 객체를 한 번만 만들어 재사용 (부드럽고 빠름). 문제 있으면 False
SERVO_LIMITS = {            # 기구 보호용 각도 제한 (절대각). 캘리브레이션 후 실제 가동범위로 줄이세요
    CH_STEER: (50, 130),
    CH_BASE: (0, 180),
    CH_SHOULDER: (0, 180),
    CH_WRIST: (0, 180),
    CH_GRIP: (0, 180),
}
SERVO_STEP_DEG = 2          # 보간 이동 시 한 스텝 각도
SERVO_STEP_DELAY_S = 0.02   # 스텝 간 대기 (작을수록 빠름; 물체를 넘어뜨리면 키우세요)
SERVO_SETTLE_S = 0.4        # 포즈 도착 후 대기
GRIP_STEP_DELAY_S = 0.01
GRIP_SETTLE_S = 0.6         # 집게 잡기/놓기 후 대기

# ---- 집게 ----
GRIP_OPEN = 130
GRIP_CLOSED = 60

# ---- 팔 좌우 회전(base) 각도 ----
BASE_CENTER = 90
BASE_LEFT = 170             # 왼쪽 상자를 향하는 각도
BASE_RIGHT = 10             # 오른쪽 상자를 향하는 각도

# ---- 포즈 (절대각). 아래 값은 전부 "자리표시자" 입니다. 반드시 --calibrate 로 실제 값을 찾아 넣으세요 ----
ARM_POSES = {
    # 홈 포즈. USE_ADEEPT_INIT_AS_HOME=True 면 RPIservo.py 의 init_pwm 값이 대신 쓰이고, 이 값은 못 읽을 때의 대체값
    "home":          {CH_BASE: 90, CH_SHOULDER: 90, CH_WRIST: 90, CH_GRIP: 90},
    # MISSION 1: 출발선에서 카메라가 모니터 QR 을 보는 자세 (필요하면 팔을 앞으로 뻗어 카메라를 모니터에 가깝게)
    "scan_qr":       {CH_BASE: 90, CH_SHOULDER: 75, CH_WRIST: 90, CH_GRIP: 90},
    # MISSION 2: 주행 중 팔 정리 자세 (모니터/상자에 안 닿게 위로)
    "drive":         {CH_BASE: 90, CH_SHOULDER: 60, CH_WRIST: 110, CH_GRIP: 90},
    # 물체를 든 채로 이동하는 높이 (base 회전 전후)
    "carry":         {CH_SHOULDER: 60, CH_WRIST: 100},
    # ↓ 집기/놓기는 팔을 멀리 뻗는 자세: shoulder ↑ = 팔을 내리며 앞으로, wrist ↑ = 손을 들어 앞으로.
    #   상자까지 모자라면 둘 다 더 키우고, 상자에 부딪히면 shoulder 를 줄이세요.
    # 상자 위 모서리에 세워진 물체 바로 앞 (집게 벌린 상태)
    "pick_approach": {CH_SHOULDER: 125, CH_WRIST: 110},
    # 집는 위치
    "pick_grab":     {CH_SHOULDER: 138, CH_WRIST: 105},
    # 반대편 상자 안쪽 위 (여기서 집게를 벌려 떨어뜨림)
    "place_drop":    {CH_SHOULDER: 130, CH_WRIST: 115},
    # MISSION 5: 종료 자세
    "finish":        {CH_BASE: 90, CH_SHOULDER: 60, CH_WRIST: 110, CH_GRIP: 90},
}

# 좌/우가 완전히 대칭이 아니면 쪽별로 포즈 일부만 덮어쓰기.  예) "LEFT": {"pick_grab": {CH_WRIST: 80}}
ARM_POSE_SIDE_OVERRIDE = {
    "LEFT": {},
    "RIGHT": {},
}

# 집기/놓기 동작 순서 (MISSION 3, 4 공용).  "PICK"/"PLACE" 는 실행 시 실제 방향(LEFT/RIGHT)으로 바뀝니다.
#   ("pose", 이름)       : ARM_POSES 의 포즈로 이동
#   ("base", 쪽)         : 팔 좌우 회전  ("PICK" | "PLACE" | "CENTER")
#   ("grip", "open"/"close")
#   ("qr", "PICK"/"PLACE"): 상자 QR 확인 (QR_BOX_CHECK_MODE 에 따라)
#   ("wait", 초)
PICK_PLACE_SEQUENCE = [
    ("pose", "carry"),
    ("grip", "open"),
    ("base", "PICK"),
    ("qr",   "PICK"),
    ("pose", "pick_approach"),
    ("pose", "pick_grab"),
    ("grip", "close"),
    ("pose", "carry"),          # 들어올리기
    ("base", "PLACE"),
    ("qr",   "PLACE"),
    ("pose", "place_drop"),
    ("grip", "open"),
    ("pose", "carry"),
    ("base", "CENTER"),
]

TUCK_ARM_BEFORE_DRIVE = True   # QR 인식 후 주행 전에 "drive" 포즈로 정리 (PDF TIP)
FINISH_POSE = "finish"         # MISSION 5 에서 취할 포즈


# =====================================================================
# 헬퍼
# =====================================================================
def get_pose(name, side=None):
    """포즈 이름 → {채널: 각도}. side('LEFT'/'RIGHT') 가 있으면 쪽별 덮어쓰기 적용."""
    if name not in ARM_POSES:
        raise KeyError("ARM_POSES 에 '%s' 포즈가 없습니다. 있는 것: %s" % (name, ", ".join(ARM_POSES)))
    pose = dict(ARM_POSES[name])
    if side:
        pose.update(ARM_POSE_SIDE_OVERRIDE.get(side, {}).get(name, {}))
    return pose


def base_angle(side):
    """'LEFT' / 'RIGHT' / 'CENTER' → base 서보 각도"""
    return {"LEFT": BASE_LEFT, "RIGHT": BASE_RIGHT, "CENTER": BASE_CENTER}[side]


def opposite(side):
    return {"LEFT": "RIGHT", "RIGHT": "LEFT"}[side]
