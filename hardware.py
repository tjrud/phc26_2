# -*- coding: utf-8 -*-
"""
hardware.py - 하드웨어 래퍼
===========================
Adeept PiCar-Pro 의 Server 코드를 재사용해서 실제 로봇을 다룹니다.
missions.py 는 이 파일의 Robot 객체만 사용합니다.

  Robot
   ├─ drive  : 전진/후진/정지/펄스        (Server/Move.py 재사용)
   ├─ servos : 서보 절대각 쓰기/보간 이동  (Server/RPIservo.py 재사용)
   ├─ steer  : 조향 (ch0)
   ├─ arm    : 포즈 이동, 집게, base 회전  (ch1~4)
   ├─ ultra  : 초음파 거리(cm)            (Server/Ultra.py 재사용)
   ├─ gyro   : MPU6050 yaw (선택)
   ├─ camera : 프레임 캡처 (Picamera2 / USB)
   └─ qr     : QR 디코딩 (pyzbar 있으면 사용, 없으면 OpenCV)

바퀴 모터와 서보는 실행 직후 Robot.init_motors() 에서 초기화되고, 나머지 부품은 처음 사용할 때
초기화됩니다(lazy). 그래서 --only 3 처럼 일부 미션만 돌릴 때 필요 없는 장치(예: 초음파)를 건드리지 않습니다.
"""
import glob
import logging
import math
import os
import statistics
import sys
import time
from typing import Callable, Dict, List, Optional

import config as C

log = logging.getLogger("hw")


class HardwareError(RuntimeError):
    pass


# =====================================================================
# Adeept Server 폴더를 import 경로에 추가
# =====================================================================
def server_dir_candidates() -> List[str]:
    """Adeept Server 폴더(Move.py 가 있는 곳) 후보를 우선순위 순으로. 이 폴더는 어디에 clone 해도 됩니다."""
    here = os.path.dirname(os.path.abspath(__file__))
    cands = []
    if C.SERVER_DIR:                                  # 1. config 에서 직접 지정
        cands.append(os.path.expanduser(C.SERVER_DIR))
    cands += [os.path.dirname(here), here]            # 2. 이 폴더의 상위 / 이 폴더 (Server/ 안에 clone 한 경우)
    homes = [os.path.expanduser("~")]                 # 3. 홈 아래 기본 위치
    sudo_user = os.environ.get("SUDO_USER")
    if sudo_user:                                     #    sudo 로 실행하면 ~ 가 /root 가 되므로 원래 사용자 홈도 확인
        homes.append(os.path.expanduser("~" + sudo_user))
    for h in homes:
        cands.append(os.path.join(h, "Adeept_PiCar-Pro", "Server"))
    cands += sorted(glob.glob("/home/*/Adeept_PiCar-Pro/Server"))   # 4. 다른 사용자 홈
    out = []
    for d in cands:
        d = os.path.normpath(d)
        if d not in out:
            out.append(d)
    return out


def add_server_path() -> str:
    candidates = server_dir_candidates()
    for d in candidates:
        if os.path.isfile(os.path.join(d, "Move.py")):
            if d not in sys.path:
                sys.path.insert(0, d)
            log.debug("Adeept Server 폴더: %s", d)
            return d
    raise HardwareError(
        "Adeept Server 폴더(Move.py)를 찾지 못했습니다. config.py 의 SERVER_DIR 에 "
        "Adeept_PiCar-Pro/Server 경로를 지정하세요. 찾아본 곳: %s" % candidates)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# =====================================================================
# 파이카 기본 서보 초기값 (Server/RPIservo.py 의 init_pwm0~7)
# =====================================================================
def read_adeept_init_positions() -> Optional[Dict[int, int]]:
    """RPIservo.py 의 init_pwm 값 {채널: 각도}. Adeept 웹서버가 켜질 때 서보가 가는 위치와 같음.
    Server 폴더나 Adeept 라이브러리가 없으면(PC 등) None."""
    try:
        add_server_path()
        import RPIservo
        sc = RPIservo.ServoCtrl()            # 스레드는 시작하지 않음, initPos 만 읽음
        return {ch: int(a) for ch, a in enumerate(sc.initPos)}
    except Exception as e:
        log.debug("RPIservo init_pwm 읽기 실패: %s", e)
        return None


# =====================================================================
# 1. 주행 (DC 모터)
# =====================================================================
class Drive:
    """backend: move(speed, direction, turn) / motorStop() 을 가진 객체 (Server/Move.py)"""

    def __init__(self, backend, sleep: Callable[[float], None] = time.sleep):
        self._b = backend
        self._sleep = sleep
        self.last_cmd = 0          # +speed / -speed / 0  (로그용)
        self.stop()

    def forward(self, speed: float):
        speed = int(clamp(speed, 0, 100))
        self._b.move(speed, 1, "no")
        self.last_cmd = speed

    def backward(self, speed: float):
        speed = int(clamp(speed, 0, 100))
        self._b.move(speed, -1, "no")
        self.last_cmd = -speed

    def stop(self):
        self._b.motorStop()
        self.last_cmd = 0

    def pulse(self, direction: int, speed: float, duration: float):
        """direction +1 전진 / -1 후진 으로 duration 초만 움직이고 정지"""
        try:
            (self.forward if direction > 0 else self.backward)(speed)
            self._sleep(duration)
        finally:
            self.stop()


# =====================================================================
# 2. 서보 (조향 + 로봇팔)
# =====================================================================
class Servos:
    """backend.set_angle(ch, angle) 만 있으면 됨. 현재 각도를 기억해서 보간 이동을 만듭니다."""

    def __init__(self, backend, sleep: Callable[[float], None] = time.sleep):
        self._b = backend
        self._sleep = sleep
        self.angles: Dict[int, float] = {}       # 마지막으로 쓴 각도 (전원 직후엔 모름)

    def _limit(self, ch: int, angle: float) -> float:
        lo, hi = C.SERVO_LIMITS.get(ch, (0, 180))
        a = clamp(float(angle), lo, hi)
        if a != angle:
            log.warning("ch%d(%s) 각도 %.1f → 제한 %.1f (SERVO_LIMITS)", ch, C.CH_NAMES.get(ch, "?"), angle, a)
        return a

    def write(self, ch: int, angle: float):
        """즉시 이동 (보간 없음)"""
        a = self._limit(ch, angle)
        self._b.set_angle(ch, int(round(a)))
        self.angles[ch] = a

    def get(self, ch: int) -> Optional[float]:
        return self.angles.get(ch)

    def move_to(self, targets: Dict[int, float], step_deg: float = None, step_delay: float = None):
        """여러 채널을 동시에 보간 이동. 현재 각도를 모르는 채널은 즉시 이동."""
        step_deg = step_deg or C.SERVO_STEP_DEG
        step_delay = C.SERVO_STEP_DELAY_S if step_delay is None else step_delay
        starts, goals = {}, {}
        for ch, a in targets.items():
            goal = self._limit(ch, a)
            cur = self.angles.get(ch)
            if cur is None:
                self.write(ch, goal)
            elif abs(cur - goal) > 1e-6:
                starts[ch], goals[ch] = cur, goal
        if not goals:
            return
        n = max(1, int(math.ceil(max(abs(goals[c] - starts[c]) for c in goals) / step_deg)))
        for i in range(1, n + 1):
            for ch in goals:
                a = starts[ch] + (goals[ch] - starts[ch]) * i / n
                self._b.set_angle(ch, int(round(a)))
                self.angles[ch] = a
            self._sleep(step_delay)
        log.debug("servo move_to %s (%d steps)", _fmt_pose(goals), n)


class Steering:
    def __init__(self, servos: Servos, center: float = 90.0):
        self._s = servos
        self.center_angle = float(center)        # 직진 서보각 (RPIservo init_pwm0 또는 90)
        self.current_deg = 0.0

    def set(self, deg: float):
        """deg > 0 우회전, deg < 0 좌회전 (STEER_RIGHT_SIGN 으로 반전 가능)"""
        deg = clamp(deg, -C.STEER_MAX_DEG, C.STEER_MAX_DEG)
        self._s.write(C.CH_STEER, self.center_angle + C.STEER_CENTER_TRIM + C.STEER_RIGHT_SIGN * deg)
        self.current_deg = deg

    def center(self):
        self.set(0)


class Arm:
    """로봇팔 (ch1 base, ch2 shoulder, ch3 wrist, ch4 grip)"""

    def __init__(self, servos: Servos, sleep: Callable[[float], None] = time.sleep,
                 home_pose: Optional[Dict[int, float]] = None):
        self.servos = servos
        self._sleep = sleep
        self.home_pose = dict(home_pose) if home_pose else C.get_pose("home")

    def init_home(self):
        """시작 시 홈 포즈로 '즉시' 이동 (Adeept moveInit 과 동일한 방식, 보간 없음)"""
        log.info("arm home → %s", _fmt_pose(self.home_pose))
        for ch, a in self.home_pose.items():
            self.servos.write(ch, a)
        self._sleep(0.5)

    def pose(self, name: str, side: Optional[str] = None, settle: float = None, speed: float = 1.0):
        """ARM_POSES 의 포즈로 보간 이동. speed>1 이면 빠르게, <1 이면 느리게. 'home' 은 RPIservo 초기값 기준"""
        targets = self.home_pose if name == "home" else C.get_pose(name, side)
        log.info("arm pose '%s'%s → %s", name, " (%s)" % side if side else "", _fmt_pose(targets))
        self.servos.move_to(targets, step_delay=C.SERVO_STEP_DELAY_S / max(speed, 1e-3))
        self._sleep(C.SERVO_SETTLE_S if settle is None else settle)

    def base_to(self, side_or_angle, settle: float = None):
        angle = C.base_angle(side_or_angle) if isinstance(side_or_angle, str) else float(side_or_angle)
        log.info("arm base → %s (%.0f°)", side_or_angle, angle)
        self.servos.move_to({C.CH_BASE: angle})
        self._sleep(C.SERVO_SETTLE_S if settle is None else settle)

    def grip(self, angle: float, settle: float = None):
        self.servos.move_to({C.CH_GRIP: angle}, step_delay=C.GRIP_STEP_DELAY_S)
        self._sleep(C.GRIP_SETTLE_S if settle is None else settle)

    def grip_open(self):
        log.info("grip open (%d°)", C.GRIP_OPEN)
        self.grip(C.GRIP_OPEN)

    def grip_close(self):
        log.info("grip close (%d°)", C.GRIP_CLOSED)
        self.grip(C.GRIP_CLOSED)

    def current_pose(self) -> Dict[int, float]:
        return {ch: self.servos.get(ch) for ch in (C.CH_BASE, C.CH_SHOULDER, C.CH_WRIST, C.CH_GRIP)}


def _fmt_pose(p: Dict[int, float]) -> str:
    return "{" + ", ".join("%s:%s" % (C.CH_NAMES.get(ch, ch), "?" if a is None else "%.0f" % a)
                           for ch, a in sorted(p.items())) + "}"


# =====================================================================
# 3. 초음파
# =====================================================================
class Ultrasonic:
    def __init__(self, read_fn: Callable[[], float], sleep: Callable[[float], None] = time.sleep):
        self._read = read_fn
        self._sleep = sleep
        self.last: Optional[float] = None

    def read_raw(self) -> Optional[float]:
        """한 번 측정 (cm). 유효 범위 밖이면 None"""
        try:
            d = float(self._read()) + C.ULTRA_OFFSET_CM
        except Exception as e:                      # 센서 순간 오류는 무시
            log.debug("ultra read error: %s", e)
            return None
        if not (C.ULTRA_MIN_VALID_CM <= d <= C.ULTRA_MAX_VALID_CM):
            return None
        self.last = d
        return d

    def read(self, samples: int = None) -> Optional[float]:
        """samples 번 측정해서 중앙값 (cm). 전부 무효면 None"""
        n = samples or C.ULTRA_SAMPLES
        vals = []
        for i in range(n):
            d = self.read_raw()
            if d is not None:
                vals.append(d)
            if i < n - 1:
                self._sleep(C.ULTRA_SAMPLE_GAP_S)
        return statistics.median(vals) if vals else None


# =====================================================================
# 4. 자이로 (MPU6050, 선택)
# =====================================================================
class Gyro:
    def __init__(self, rate_fn: Callable[[], float], sleep: Callable[[float], None] = time.sleep,
                 now: Callable[[], float] = time.monotonic):
        self._rate = rate_fn           # yaw 축 각속도 (deg/s), 부호 적용 전
        self._sleep = sleep
        self._now = now
        self.bias = 0.0
        self.yaw = 0.0
        self._t = None

    def calibrate(self, samples: int = None):
        n = samples or C.GYRO_BIAS_SAMPLES
        vals = []
        for _ in range(n):
            vals.append(self._rate())
            self._sleep(0.01)
        self.bias = sum(vals) / len(vals)
        log.info("gyro bias = %.3f deg/s (%d samples)", self.bias, n)
        self.reset()

    def reset(self):
        self.yaw = 0.0
        self._t = self._now()

    def update(self) -> float:
        """적분해서 yaw(deg) 갱신. +: 왼쪽(반시계) 회전 (GYRO_YAW_SIGN 으로 맞춤)"""
        t = self._now()
        if self._t is None:
            self._t = t
        dt = t - self._t
        self._t = t
        self.yaw += (self._rate() - self.bias) * C.GYRO_YAW_SIGN * dt
        return self.yaw

    def rate(self) -> float:
        return (self._rate() - self.bias) * C.GYRO_YAW_SIGN


# =====================================================================
# 5. 카메라 / QR
# =====================================================================
class Camera:
    def __init__(self, backend: str = None):
        backend = backend or C.CAMERA_BACKEND
        self._picam = None
        self._cap = None
        self.kind = None
        errors = []
        if backend in ("auto", "picamera2"):
            try:
                self._open_picamera2()
            except Exception as e:
                errors.append("picamera2: %s" % e)
                if backend == "picamera2":
                    raise HardwareError("Picamera2 열기 실패: %s" % e)
        if self.kind is None and backend in ("auto", "usb"):
            try:
                self._open_usb()
            except Exception as e:
                errors.append("usb: %s" % e)
        if self.kind is None:
            raise HardwareError("카메라를 열 수 없습니다 (%s). Adeept_Robot.service 가 카메라를 잡고 있지 않은지 확인하세요."
                                % "; ".join(errors))
        log.info("camera: %s %sx%s", self.kind, *C.CAMERA_SIZE)
        time.sleep(C.CAMERA_WARMUP_S)

    def _open_picamera2(self):
        from picamera2 import Picamera2          # noqa
        import libcamera                          # noqa
        cam = Picamera2()
        cfg = cam.create_preview_configuration(
            main={"size": tuple(C.CAMERA_SIZE), "format": "RGB888"},
            transform=libcamera.Transform(hflip=C.CAMERA_HFLIP, vflip=C.CAMERA_VFLIP))
        cam.configure(cfg)
        cam.start()
        self._picam = cam
        self.kind = "picamera2"

    def _open_usb(self):
        import cv2
        cap = cv2.VideoCapture(C.USB_CAMERA_INDEX)
        if not cap.isOpened():
            raise HardwareError("VideoCapture(%d) 열기 실패" % C.USB_CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.CAMERA_SIZE[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.CAMERA_SIZE[1])
        self._cap = cap
        self.kind = "usb"

    def read(self):
        """BGR numpy 프레임 또는 None"""
        import cv2
        if self._picam is not None:
            return self._picam.capture_array()      # flip 은 configure 의 transform 에서 처리됨
        ok, frame = self._cap.read()
        if not ok:
            return None
        if C.CAMERA_HFLIP and C.CAMERA_VFLIP:
            frame = cv2.flip(frame, -1)
        elif C.CAMERA_HFLIP:
            frame = cv2.flip(frame, 1)
        elif C.CAMERA_VFLIP:
            frame = cv2.flip(frame, 0)
        return frame

    def close(self):
        try:
            if self._picam is not None:
                self._picam.stop()
                self._picam.close()
            if self._cap is not None:
                self._cap.release()
        except Exception as e:
            log.debug("camera close: %s", e)
        self._picam = self._cap = None


class QRReader:
    """frame → 디코딩된 문자열 목록 (strip().upper(), 중복 제거)"""

    def __init__(self):
        import cv2
        self._cv2 = cv2
        self._det = cv2.QRCodeDetector()
        self._pyzbar = None
        try:
            from pyzbar import pyzbar as _pz      # 설치돼 있으면 더 잘 읽힘 (sudo apt install libzbar0; pip3 install pyzbar)
            self._pyzbar = _pz
            log.info("QR: pyzbar + OpenCV")
        except Exception:
            log.info("QR: OpenCV QRCodeDetector")
        self.last_texts: List[str] = []

    def decode(self, frame) -> List[str]:
        if frame is None:
            return []
        cv2 = self._cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        texts = self._decode_gray(gray)
        if not texts and C.QR_TRY_UPSCALE:
            big = cv2.resize(gray, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
            texts = self._decode_gray(big)
        self.last_texts = texts
        return texts

    def _decode_gray(self, gray) -> List[str]:
        found = []
        if self._pyzbar is not None:
            try:
                for obj in self._pyzbar.decode(gray):
                    found.append(obj.data.decode("utf-8", "ignore"))
            except Exception as e:
                log.debug("pyzbar error: %s", e)
        if not found:
            try:
                ok, datas, _, _ = self._det.detectAndDecodeMulti(gray)
                if ok:
                    found.extend(d for d in datas if d)
            except Exception as e:
                log.debug("cv2 multi error: %s", e)
            if not found:
                try:
                    data, _, _ = self._det.detectAndDecode(gray)
                    if data:
                        found.append(data)
                except Exception as e:
                    log.debug("cv2 single error: %s", e)
        out = []
        for t in found:
            t = t.strip().upper()
            if t and t not in out:
                out.append(t)
        return out

    def save_debug(self, frame, path: str):
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self._cv2.imwrite(path, frame)
        except Exception as e:
            log.debug("save_debug: %s", e)


# =====================================================================
# 6. 실제 로봇
# =====================================================================
class Robot:
    """실제 하드웨어. 부품은 처음 접근할 때 초기화됩니다."""

    def __init__(self):
        self.sleep = time.sleep
        self.now = time.monotonic
        self._drive = self._servos = self._steer = self._arm = None
        self._ultra = self._gyro = self._camera = self._qr = None
        self._server_dir = None
        self._adeept_init = None

    # ---- 내부 ----
    def _server(self):
        if self._server_dir is None:
            self._server_dir = add_server_path()
        return self._server_dir

    # ---- 파이카 기본 초기값 ----
    def adeept_init(self) -> Optional[Dict[int, int]]:
        """RPIservo.py 의 init_pwm 값 (한 번만 읽음). 없으면 None"""
        if self._adeept_init is None:
            init = read_adeept_init_positions() if C.USE_ADEEPT_INIT_AS_HOME else None
            self._adeept_init = init if init else False
            if init:
                log.info("서보 초기값: RPIservo.py init_pwm %s", _fmt_pose({ch: init[ch] for ch in range(5) if ch in init}))
            elif C.USE_ADEEPT_INIT_AS_HOME:
                log.info("서보 초기값: RPIservo.py 를 못 읽어 config ARM_POSES['home'] / 90 사용")
        return self._adeept_init or None

    def home_pose(self) -> Dict[int, float]:
        """홈 포즈 (팔 채널). RPIservo init_pwm 이 있으면 그 값, 없으면 ARM_POSES['home']"""
        pose = C.get_pose("home")
        init = self.adeept_init()
        if init:
            pose = {ch: float(init.get(ch, a)) for ch, a in pose.items()}
        return pose

    def steer_center(self) -> float:
        init = self.adeept_init()
        return float(init.get(C.CH_STEER, 90)) if init else 90.0

    # ---- 모터 초기화 ----
    def init_motors(self):
        """run_mission.py 가 실행되자마자 호출: 바퀴 정지 → 조향/팔 서보를 기본값으로 즉시 이동 (Adeept moveInit 과 같은 방식)"""
        self.drive.stop()
        self.steer.center()
        log.info("모터 초기화: 바퀴 정지, steer → %.0f", self.servos.get(C.CH_STEER))
        self.arm.init_home()

    # ---- 부품 (lazy) ----
    @property
    def drive(self) -> Drive:
        if self._drive is None:
            self._server()
            import Move                       # Server/Move.py (M1/M2_Direction 수정 그대로 반영)
            Move.setup()
            self._drive = Drive(Move, self.sleep)
            log.info("drive: Server/Move.py (M1_Direction=%s, M2_Direction=%s)", Move.M1_Direction, Move.M2_Direction)
        return self._drive

    @property
    def servos(self) -> Servos:
        if self._servos is None:
            self._server()
            self._servos = Servos(_make_servo_backend(C.SERVO_FAST_DRIVER), self.sleep)
            log.info("servos: RPIservo.ServoCtrl (%s)", "fast" if C.SERVO_FAST_DRIVER else "stock")
        return self._servos

    @property
    def steer(self) -> Steering:
        if self._steer is None:
            self._steer = Steering(self.servos, center=self.steer_center())
        return self._steer

    @property
    def arm(self) -> Arm:
        if self._arm is None:
            self._arm = Arm(self.servos, self.sleep, home_pose=self.home_pose())
        return self._arm

    @property
    def ultra(self) -> Ultrasonic:
        if self._ultra is None:
            self._server()
            if C.ULTRA_MODE == "adeept":
                import Ultra                  # import 시점에 DistanceSensor(echo=24, trigger=23) 생성됨
                self._ultra = Ultrasonic(Ultra.checkdist, self.sleep)
                log.info("ultra: Server/Ultra.py")
            else:
                from gpiozero import DistanceSensor
                sensor = DistanceSensor(echo=C.ULTRA_ECHO_PIN, trigger=C.ULTRA_TRIG_PIN, max_distance=2,
                                        queue_len=C.ULTRA_FAST_QUEUE_LEN, partial=True)
                self._ultra = Ultrasonic(lambda: sensor.distance * 100, self.sleep)
                log.info("ultra: gpiozero fast (queue_len=%d)", C.ULTRA_FAST_QUEUE_LEN)
        return self._ultra

    @property
    def gyro(self) -> Optional[Gyro]:
        """MPU6050 이 없거나 실패하면 None (경고만)"""
        if self._gyro is None:
            try:
                from mpu6050 import mpu6050
                sensor = mpu6050(C.GYRO_ADDR)
                axis = C.GYRO_YAW_AXIS
                self._gyro = Gyro(lambda: sensor.get_gyro_data()[axis], self.sleep, self.now)
                log.info("gyro: MPU6050 @0x%02x axis=%s", C.GYRO_ADDR, axis)
            except Exception as e:
                log.warning("gyro 사용 불가 (%s) → 직진 보정 없이 진행", e)
                self._gyro = False
        return self._gyro or None

    @property
    def camera(self) -> Camera:
        if self._camera is None:
            self._camera = Camera()
        return self._camera

    @property
    def qr(self) -> QRReader:
        if self._qr is None:
            self._qr = QRReader()
        return self._qr

    # ---- 복합 동작 ----
    def read_qr(self, save_path: Optional[str] = None) -> List[str]:
        frame = self.camera.read()
        texts = self.qr.decode(frame)
        if save_path and frame is not None:
            self.qr.save_debug(frame, save_path)
        return texts

    def stop_all(self):
        """비상/종료 시: 모터만 즉시 정지 (팔은 그 자리 유지)"""
        try:
            if self._drive is not None:
                self._drive.stop()
        except Exception as e:
            log.error("stop_all: %s", e)

    def shutdown(self):
        self.stop_all()
        if self._camera is not None:
            self._camera.close()


def _make_servo_backend(fast: bool):
    import RPIservo
    if not fast:
        return RPIservo.ServoCtrl()             # 스톡: set_angle 마다 I2C/PCA9685 객체 재생성

    from board import SCL, SDA
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo as af_servo

    class FastServoCtrl(RPIservo.ServoCtrl):
        """RPIservo.ServoCtrl 과 같은 파라미터(0x5f, 50Hz, 500~2400us)로 객체를 한 번만 만들어 재사용"""

        def __init__(self):
            super().__init__()
            self._pwm = PCA9685(busio.I2C(SCL, SDA), address=0x5f)
            self._pwm.frequency = 50
            self._servos = {}

        def set_angle(self, ID, angle):
            s = self._servos.get(ID)
            if s is None:
                s = af_servo.Servo(self._pwm.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
                self._servos[ID] = s
            s.angle = clamp(angle, 0, 180)

    return FastServoCtrl()
