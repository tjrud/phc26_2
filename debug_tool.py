#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
debug_tool.py - 카메라(QR 인식) + 서보 각도 + 수동 주행 디버깅 도구 (웹 화면)
==========================================================================
파이에서 실행하고, 같은 와이파이에 연결된 노트북/폰 브라우저로 접속합니다.
Adeept 웹 화면처럼 주행도 되므로 이 화면 하나로 주행 · QR 확인 · 팔 각도 찾기를 할 수 있습니다.

  sudo systemctl stop Adeept_Robot.service     # 웹서버가 카메라/GPIO 를 잡고 있으므로 먼저 정지 (둘은 동시에 못 켬)
  python3 debug_tool.py                        # 터미널에 나오는 주소(http://파이IP:8000)로 접속
  python3 debug_tool.py --port 8080            # 포트 변경 (8000 이 사용 중일 때)

화면에서 할 수 있는 것
  * 카메라 실시간 화면 + QR 인식: 초록 테두리 = 읽힘, 노랑 = QR 은 보이는데 해독 실패
    밝기 / 선명도 / QR 크기(px) / 연속 인식 횟수, 안 읽히는 이유 힌트
  * [분석] 지금 화면을 여러 방법(확대, 대비 향상, 이진화, 좌우 반전 ...)으로 읽어 보고 어떤 방법이 되는지 표시
  * 주행: 전진/후진/좌우 조향 버튼은 누르고 있는 동안만 움직이고 떼면 멈춤(조향은 가운데로), 속도 조절, 정지
    키보드 ↑ ↓ ← →, 스페이스 = 정지. 브라우저 연결이 끊기거나 탭을 닫아 명령이 0.5초 동안 안 오면 자동 정지
  * 초음파 거리 + 정지선 기준 위치(빨간 선 / 노란 선 / 넘어감), [펄스] = 정밀 정지 펄스 한 번과 이동 거리
  * 서보 5개(steer, base, shoulder, wrist, grip) 현재 각도 확인, ± 스텝 또는 숫자로 이동 (키보드도 가능)
  * config 포즈로 이동, 집게 열기/닫기, base 좌/우, 모터 초기화
  * 현재 각도를 config.py 에 붙여넣을 형식으로 표시, [기록] 하면 logs/debug_poses.txt 에 저장
  * 카메라 밝기 보정(ExposureValue) / 대비 / 선명도 조절 → 잘 읽히는 값을 config.CAMERA_CONTROLS 에

run_mission.py 처럼 실행하면 모터 초기화(바퀴 정지 + 서보 기본값)부터 하고, Ctrl+C 로 끝내면 바퀴는 멈추고 서보는 그 자리에 둡니다.
"""
import argparse
import json
import logging
import logging.handlers
import math
import os
import queue
import signal
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as C                                      # noqa: E402
from hardware import Robot, HardwareError, clamp        # noqa: E402
from run_mission import setup_logging, check_service    # noqa: E402

log = logging.getLogger("debug")

SERVO_CHANNELS = [C.CH_STEER, C.CH_BASE, C.CH_SHOULDER, C.CH_WRIST, C.CH_GRIP]
CAMERA_CONTROL_SPEC = [            # picamera2 컨트롤: 이름, 최소, 최대, 스텝, 기본값
    ("ExposureValue", -4.0, 4.0, 0.5, 0.0),
    ("Contrast", 0.0, 4.0, 0.1, 1.0),
    ("Sharpness", 0.0, 8.0, 0.5, 1.0),
]
DRIVE_HOLD_S = 0.5                 # 주행 버튼을 누르는 동안 브라우저가 0.15초마다 명령을 다시 보냄. 이 시간 동안 안 오면 자동 정지
# Ctrl+C / kill / 터미널 닫힘 / Ctrl+\ / Ctrl+Z(멈추면 감시 스레드도 멈춰서 바퀴가 계속 돎): 모두 바퀴를 세우고 종료
EXIT_SIGNALS = [getattr(signal, n) for n in ("SIGINT", "SIGTERM", "SIGHUP", "SIGQUIT", "SIGTSTP") if hasattr(signal, n)]


# =====================================================================
# 작은 도우미
# =====================================================================
class _Rate:
    """초당 횟수 (지수 평균)"""

    def __init__(self):
        self._t = None
        self.value = 0.0

    def tick(self):
        now = time.monotonic()
        if self._t is not None and now > self._t:
            r = 1.0 / (now - self._t)
            self.value = r if self.value == 0 else self.value * 0.8 + r * 0.2
        self._t = now


def _side_px(quad) -> float:
    """QR 테두리 한 변의 평균 길이 (px)"""
    n = len(quad)
    return sum(math.dist(quad[i], quad[(i + 1) % n]) for i in range(n)) / n


def _center(quad):
    return sum(p[0] for p in quad) / len(quad), sum(p[1] for p in quad) / len(quad)


def _put_text(img, text, org, color, scale=0.5):
    """검은 배경 위에 글자 (cv2 글꼴은 영문만 지원)"""
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = int(org[0]), int(org[1])
    cv2.rectangle(img, (x - 3, y - th - 4), (x + tw + 3, y + base + 1), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def _pyzbar_texts(pyzbar, img):
    if pyzbar is None:
        return None                     # 설치 안 됨
    try:
        return [o.data.decode("utf-8", "ignore") for o in pyzbar.decode(img)]
    except Exception as e:
        return ["오류: %s" % e]


def _opencv_texts(det, img):
    try:
        ok, datas, _, _ = det.detectAndDecodeMulti(img)
        found = [d for d in datas if d] if ok else []
        if not found:
            data, _, _ = det.detectAndDecode(img)
            found = [data] if data else []
        return found
    except Exception as e:
        return ["오류: %s" % e]


def _hints(res) -> list:
    """QR 이 안 읽히는 이유 추정 (화면 아래 표시)"""
    if res["start"]:
        return []
    hints = []
    texts, boxes = res["texts"], res["boxes"]
    if texts:
        hints.append("읽힌 값 %s 은(는) 출발 QR(QR_START_MAP: %s)이 아님" % (texts, ", ".join(C.QR_START_MAP)))
    elif not boxes:
        hints.append("화면에서 QR 을 못 찾음 → 카메라가 QR 을 향하는지(scan_qr 자세), QR 전체가 화면에 들어오는지 확인")
    else:
        hints.append("QR 은 보이는데 해독 실패(노란 테두리) → 흐림 / 빛 반사 / 너무 작음. [분석] 으로 어떤 방법이 읽히는지 확인")
    sizes = [_side_px(q) for q, _ in boxes]
    if sizes and min(sizes) < 80:
        hints.append("QR 이 작음 (한 변 %dpx) → 팔을 앞으로 뻗어 카메라를 가깝게, 또는 CAMERA_SIZE ↑ (100px 이상 권장)"
                     % min(sizes))
    if res["brightness"] > 200:
        hints.append("화면이 너무 밝음 (평균 %d) → 카메라 조절에서 ExposureValue 를 낮춰 보세요" % res["brightness"])
    elif res["brightness"] < 50:
        hints.append("화면이 너무 어두움 (평균 %d) → ExposureValue ↑" % res["brightness"])
    if res["sharpness"] < 30:
        hints.append("화면이 흐림 (선명도 %d) → 렌즈 초점 / 흔들림 확인, 카메라 조절에서 Sharpness ↑" % res["sharpness"])
    return hints


# =====================================================================
# 카메라: 화면 스트리밍 스레드 + QR 인식 스레드
# =====================================================================
class Vision:
    """화면은 부드럽게 보내고, QR 인식은 할 수 있는 만큼 자주 돌려서 그 결과를 화면 위에 그립니다."""

    def __init__(self, robot: Robot, stream_width: int, stream_fps: float):
        self.robot = robot
        self.stream_width = stream_width
        self.stream_dt = 1.0 / max(stream_fps, 1.0)
        self.cond = threading.Condition()
        self.qr_lock = threading.Lock()          # robot.qr 는 한 번에 한 스레드만 사용
        self.frame, self.frame_seq = None, 0     # 마지막 원본 프레임 (BGR)
        self.jpeg, self.jpeg_seq = None, 0       # 화면 전송용 JPEG
        self.result = {"texts": [], "start": None, "streak": 0, "boxes": [], "brightness": None,
                       "sharpness": None, "decode_ms": None, "hints": []}
        self.crosshair = True
        self.running = True
        self.error = None
        self.cam_rate, self.qr_rate = _Rate(), _Rate()
        self._det = cv2.QRCodeDetector()         # 화면 표시용 (해독 실패한 QR 위치까지 찾음)
        self._pyzbar = robot.qr._pyzbar
        self._streak_dir, self._streak = None, 0
        self._threads = []

    def start(self):
        for fn in (self._capture_loop, self._detect_loop):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self):
        self.running = False
        with self.cond:
            self.cond.notify_all()
        for t in self._threads:
            t.join(timeout=2.0)

    def latest_frame(self):
        with self.cond:
            return self.frame

    def _capture_loop(self):
        while self.running:
            t0 = time.monotonic()
            try:
                frame = self.robot.camera.read()
                self.error = None
            except Exception as e:
                self.error = "카메라 읽기 실패: %s" % e
                time.sleep(0.5)
                continue
            if frame is None:
                time.sleep(0.05)
                continue
            self.cam_rate.tick()
            with self.cond:
                self.frame, self.frame_seq = frame, self.frame_seq + 1
                res = self.result
                self.cond.notify_all()
            ok, buf = cv2.imencode(".jpg", self._draw(frame, res), [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                with self.cond:
                    self.jpeg, self.jpeg_seq = buf.tobytes(), self.jpeg_seq + 1
                    self.cond.notify_all()
            time.sleep(max(0.0, self.stream_dt - (time.monotonic() - t0)))

    def _detect_loop(self):
        last = 0
        while self.running:
            with self.cond:
                self.cond.wait_for(lambda: self.frame_seq != last or not self.running, timeout=1.0)
                frame, last = self.frame, self.frame_seq
            if frame is None or not self.running:
                continue
            t0 = time.monotonic()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            boxes = self._boxes(gray)
            decoded = [t.strip().upper() for _, t in boxes if t and t.strip()]
            if decoded:                                      # 미션의 첫 단계(pyzbar / OpenCV)와 같은 결과
                texts = list(dict.fromkeys(decoded))
            else:                                            # 못 읽었으면 미션 방식 전체(확대 재시도 포함)로 확인
                with self.qr_lock:
                    texts = self.robot.qr.decode(frame)
            ms = (time.monotonic() - t0) * 1000
            self.qr_rate.tick()
            start = next((C.QR_START_MAP[t] for t in texts if t in C.QR_START_MAP), None)
            if start and start == self._streak_dir:
                self._streak += 1
            else:
                self._streak_dir, self._streak = start, (1 if start else 0)
            res = {"texts": texts, "start": start, "streak": self._streak, "boxes": boxes,
                   "brightness": round(float(gray.mean())),
                   "sharpness": round(float(cv2.Laplacian(gray, cv2.CV_64F).var())),
                   "decode_ms": round(ms)}
            res["hints"] = _hints(res)
            with self.cond:
                self.result = res

    def _boxes(self, gray):
        """화면에 그릴 QR 위치 [(꼭짓점들, 글자 또는 None)].  None = QR 모양은 찾았지만 해독 실패"""
        boxes = []
        if self._pyzbar is not None:
            try:
                for o in self._pyzbar.decode(gray):
                    boxes.append(([(p.x, p.y) for p in o.polygon], o.data.decode("utf-8", "ignore")))
            except Exception as e:
                log.debug("pyzbar: %s", e)
        try:
            ok, datas, points, _ = self._det.detectAndDecodeMulti(gray)
            if ok and points is not None:
                for text, quad in zip(datas, points):
                    quad = [(float(x), float(y)) for x, y in quad]
                    if all(math.dist(_center(quad), _center(q)) > 25 for q, _ in boxes):
                        boxes.append((quad, text or None))
        except Exception as e:
            log.debug("cv2 QR: %s", e)
        return boxes

    def _draw(self, frame, res):
        h, w = frame.shape[:2]
        s = min(1.0, self.stream_width / float(w)) if self.stream_width else 1.0
        img = cv2.resize(frame, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA) if s < 1.0 else frame.copy()
        ih, iw = img.shape[:2]
        if self.crosshair:
            cv2.line(img, (iw // 2, 0), (iw // 2, ih), (120, 120, 120), 1)
            cv2.line(img, (0, ih // 2), (iw, ih // 2), (120, 120, 120), 1)
        for quad, text in res["boxes"]:
            pts = np.array([[int(x * s), int(y * s)] for x, y in quad], dtype=np.int32)
            color = (90, 220, 90) if text else (0, 215, 255)          # BGR: 초록 = 읽힘, 노랑 = 해독 실패
            cv2.polylines(img, [pts], True, color, 2)
            _put_text(img, "%s %dpx" % (text or "?", _side_px(quad)),
                      (pts[:, 0].min(), max(16, pts[:, 1].min() - 6)), color)
        _put_text(img, "QR %s  streak %d/%d" % (",".join(res["texts"]) or "-", res["streak"], C.QR_CONFIRM_COUNT),
                  (8, 22), (255, 255, 255), 0.6)
        _put_text(img, "cam %.0f fps  qr %.1f/s %s ms  bright %s  sharp %s"
                  % (self.cam_rate.value, self.qr_rate.value, res["decode_ms"], res["brightness"], res["sharpness"]),
                  (8, 46), (200, 200, 200))
        return img


# =====================================================================
# 주행: 버튼을 누르는 동안만 움직이고, 명령이 끊기면 자동 정지 (+ 초음파 거리)
# =====================================================================
def _zone(cm):
    """정지선 기준 위치. missions.PrecisionStopper.classify 와 같은 기준"""
    if cm is None:
        return None
    err = cm - C.STOP_TARGET_CM
    if abs(err) <= C.RED_HALF_WIDTH_CM:
        return "RED"
    if abs(err) <= C.YELLOW_HALF_WIDTH_CM:
        return "YELLOW"
    return "OVER" if err < 0 else "SHORT"


def _drive_label(d: int, speed: int) -> str:
    return {1: "전진 %d" % speed, -1: "후진 %d" % speed}.get(d, "정지")


class Driving:
    """주행 버튼. 누르고 있는 동안 브라우저가 같은 명령을 0.15초마다 다시 보내고, DRIVE_HOLD_S 동안 명령이 없으면
    (떼는 신호를 놓침, 와이파이 끊김, 탭 닫힘) 감시 스레드가 바퀴를 세웁니다. 초음파 거리도 여기서 계속 읽습니다."""

    def __init__(self, robot: Robot):
        self.robot = robot
        self.lock = threading.Lock()             # 바퀴/조향 명령은 한 번에 하나씩. 서보 lock 과 따로라서 팔이 움직이는 중에도 바로 멈춤
        self.dir, self.speed = 0, 0              # 바퀴: +1 전진 / -1 후진 / 0 정지
        self.steer_dir = 0                       # 조향: +1 오른쪽 / -1 왼쪽 / 0 가운데
        self.deadline = 0.0                      # 이 시각까지 다음 주행 명령이 안 오면 자동 정지
        self.auto_stops = 0
        self.closed = False                      # 종료 중이면 바퀴를 다시 돌리지 않음
        self.dist = {"cm": None, "t": None, "error": None}
        self._seq = {}                           # (브라우저 id, 종류) → 마지막 명령 순번
        self._halt = threading.Event()
        self._threads = []

    def start(self):
        for fn in (self._watchdog_loop, self._ultra_loop):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self._threads.append(t)

    def close(self):
        """종료: 바퀴를 세우고, 이후 들어오는 주행 명령은 무시"""
        with self.lock:
            self.closed = True
            for _ in range(20):                  # I2C 순간 오류면 약 2초 동안 다시 시도 (이 뒤로는 감시 스레드가 없음)
                try:
                    self._apply(0, 0)
                    break
                except Exception as e:
                    log.error("바퀴 정지 실패: %s", e)
                    time.sleep(0.1)
            else:
                log.error("바퀴가 아직 돌고 있을 수 있습니다! 전원을 끄세요")
        self._halt.set()
        for t in self._threads:
            t.join(timeout=2.0)

    def _fresh(self, kind: str, q: dict) -> bool:
        """그 브라우저가 이미 더 나중 명령을 보냈으면 늦게 도착한 옛 명령(예: 뗀 뒤에 도착한 주행 명령) → False"""
        sid, seq = q.get("sid"), q.get("seq")
        if not sid or seq is None:
            return True
        key, seq = (sid, kind), int(seq)
        if seq <= self._seq.get(key, -1):
            return False
        self._seq[key] = seq
        return True

    def _apply(self, d: int, speed: int):
        """바퀴 명령 (lock 안에서). 누르는 동안 계속 다시 보내므로 모터 상태가 어긋나도 곧 맞춰짐.
        바퀴가 돌 수 있으면 dir 이 0 이 아니어야 감시 스레드가 세울 수 있으므로, 돌리는 명령은 보내기 전에,
        정지는 성공한 뒤에 상태를 바꿉니다. (I2C 오류로 명령이 실패해도 감시 스레드가 정지를 다시 시도)"""
        drive = self.robot.drive
        if d:
            self.dir, self.speed = d, speed
            (drive.forward if d > 0 else drive.backward)(speed)
            return
        try:
            drive.stop()
        except Exception:
            self.deadline = 0.0
            raise
        self.dir, self.speed = 0, 0

    def drive(self, q: dict) -> dict:
        """누르는 동안 반복해서 오는 주행 명령: dir=1 전진 / -1 후진 / 0 정지(뗌), speed=0~100"""
        d = int(q.get("dir", 0))
        if d not in (-1, 0, 1):
            raise ValueError("dir 은 -1, 0, 1 중 하나")
        speed = int(clamp(float(q.get("speed", C.DRIVE_SPEED_FAST)), 0, 100)) if d else 0
        with self.lock:
            if self._fresh("drive", q) and not (d and self.closed):
                changed = (d, speed) != (self.dir, self.speed)
                self.deadline = time.monotonic() + DRIVE_HOLD_S
                self._apply(d, speed)
                if changed:
                    log.info("주행 %s", _drive_label(d, speed))
        return self.info()

    def stop(self, q: dict = None, why: str = "정지 버튼") -> dict:
        """무조건 정지. 순번도 갱신해서, 이보다 먼저 보낸 주행 명령이 늦게 도착해도 무시"""
        with self.lock:
            if q:
                self._fresh("drive", q)
            moving = self.dir
            self._apply(0, 0)
            if moving:
                log.info("주행 정지 (%s)", why)
        return self.info()

    def steer(self, q: dict) -> dict:
        """조향 버튼: dir=-1 왼쪽 / 1 오른쪽 (STEER_MAX_DEG 만큼), 0 가운데(뗌)"""
        d = int(q.get("dir", 0))
        if d not in (-1, 0, 1):
            raise ValueError("dir 은 -1, 0, 1 중 하나")
        with self.lock:
            if self._fresh("steer", q):
                self.robot.steer.set(d * C.STEER_MAX_DEG)
                self.steer_dir = d
        return self.info()

    def pulse(self, q: dict) -> dict:
        """정밀 정지(MISSION 2)의 펄스 한 번: CREEP_SPEED 로 CREEP_PULSE_S 초 움직이고 정지"""
        d = 1 if int(q.get("dir", 1)) > 0 else -1
        with self.lock:
            if self.closed or not self._fresh("drive", q):
                return self.info()
            self.dir, self.speed, self.deadline = d, C.CREEP_SPEED, 0.0      # 펄스 끝의 정지가 실패하면 감시 스레드가 세움
            self.robot.drive.pulse(d, C.CREEP_SPEED, C.CREEP_PULSE_S)
            self.dir, self.speed = 0, 0
            log.info("펄스 %s (CREEP_SPEED %d, %.2f초)", "전진" if d > 0 else "후진", C.CREEP_SPEED, C.CREEP_PULSE_S)
        return self.info()

    def info(self) -> dict:
        dist = self.dist
        cm = dist["cm"]
        return {"dir": self.dir, "speed": self.speed, "steer": self.steer_dir, "auto_stops": self.auto_stops,
                "cm": None if cm is None else round(cm, 1), "zone": _zone(cm), "error": dist["error"],
                "age": None if dist["t"] is None else round(time.monotonic() - dist["t"], 1)}

    def _watchdog_loop(self):
        while not self._halt.wait(0.05):
            with self.lock:
                if not self.dir or time.monotonic() <= self.deadline:
                    continue
                try:
                    self._apply(0, 0)
                except Exception as e:
                    self.deadline = time.monotonic() + 0.1
                    log.error("자동 정지 실패 (0.1초 뒤 다시 시도): %s", e)
                else:
                    self.auto_stops += 1
                    log.warning("바퀴 자동 정지: 주행 명령이 끊김 (버튼 뗀 신호를 놓침 / 연결 끊김) 또는 모터 명령 실패")

    def _ultra_loop(self):
        while not self._halt.is_set():
            try:
                cm = self.robot.ultra.read()
            except Exception as e:                   # 센서를 못 염 (배선 / GPIO 사용 중) → 잠시 뒤 다시 시도
                if self.dist["error"] is None:
                    log.warning("초음파 사용 불가: %s", e)
                self.dist = {"cm": None, "t": time.monotonic(), "error": "초음파 사용 불가: %s" % e}
                self._halt.wait(3.0)
                continue
            self.dist = {"cm": cm, "t": time.monotonic(), "error": None}
            self._halt.wait(0.1)


# =====================================================================
# 화면의 요청 처리
# =====================================================================
class App:
    def __init__(self, robot: Robot, vision: Vision, driving: Driving):
        self.robot = robot
        self.vision = vision
        self.driving = driving
        self.lock = threading.Lock()             # 서보 명령은 한 번에 하나씩
        self.records = []
        self.picam = getattr(robot.camera, "_picam", None)
        self.controls = ({name: float(C.CAMERA_CONTROLS.get(name, default)) for name, _, _, _, default in CAMERA_CONTROL_SPEC}
                         if self.picam is not None else None)

    def status(self) -> dict:
        v = self.vision
        with v.cond:
            res = v.result
        out = {
            "servos": {C.CH_NAMES[ch]: self.robot.servos.get(ch) for ch in SERVO_CHANNELS},
            "config": self.config_text(),
            "qr": {k: res[k] for k in ("texts", "start", "streak", "brightness", "sharpness", "decode_ms", "hints")},
            "confirm": C.QR_CONFIRM_COUNT,
            "boxes": [{"text": t, "size": round(_side_px(q))} for q, t in res["boxes"]],
            "camera": {"kind": self.robot.camera.kind, "size": list(C.CAMERA_SIZE), "fps": round(v.cam_rate.value, 1),
                       "error": v.error},
            "controls": self.controls,
            "control_spec": CAMERA_CONTROL_SPEC if self.controls is not None else [],
            "poses": list(C.ARM_POSES),
            "records": self.records[-12:],
            "drive": self.driving.info(),
            "drive_cfg": {"speed": C.DRIVE_SPEED_FAST, "target": C.STOP_TARGET_CM, "creep_speed": C.CREEP_SPEED,
                          "creep_s": C.CREEP_PULSE_S, "settle_s": C.CREEP_SETTLE_S},
        }
        if self.controls is not None:
            out["controls_text"] = "CAMERA_CONTROLS = {%s}" % ", ".join('"%s": %.1f' % kv for kv in self.controls.items())
        return out

    def config_text(self) -> str:
        get = self.robot.servos.get

        def deg(ch):
            a = get(ch)
            return "?" if a is None else "%d" % round(a)

        steer, center = get(C.CH_STEER), self.robot.steer.center_angle
        trim = "?" if steer is None else "%d" % round(steer - center)
        if self.driving.steer_dir:               # 조향 버튼을 누르는 중에는 ±30 이 trim 처럼 보이지 않게
            trim = "(조향 버튼 누르는 중)"
        return "\n".join([
            "포즈 전체:   {CH_BASE: %s, CH_SHOULDER: %s, CH_WRIST: %s, CH_GRIP: %s}"
            % (deg(C.CH_BASE), deg(C.CH_SHOULDER), deg(C.CH_WRIST), deg(C.CH_GRIP)),
            "집기/놓기:   {CH_SHOULDER: %s, CH_WRIST: %s}" % (deg(C.CH_SHOULDER), deg(C.CH_WRIST)),
            "BASE_LEFT / BASE_RIGHT = %s     GRIP_OPEN / GRIP_CLOSED = %s" % (deg(C.CH_BASE), deg(C.CH_GRIP)),
            "STEER_CENTER_TRIM = %s  (중립 %.0f 기준)" % (trim, center),
        ])

    def action(self, path: str, q: dict) -> dict:
        robot = self.robot
        if self.driving.closed:                  # 종료 중: 재사용 중인 연결로 요청이 더 와도 아무것도 움직이지 않음
            raise HardwareError("종료 중입니다")
        if path == "/api/drive":                 # 주행은 서보 lock 을 기다리지 않음 (팔이 움직이는 중에도 바로 멈춤)
            return {"drive": self.driving.drive(q)}
        if path == "/api/steer":
            return {"drive": self.driving.steer(q)}
        if path == "/api/stop":
            return {"drive": self.driving.stop(q)}
        if path == "/api/pulse":
            return {"drive": self.driving.pulse(q)}
        if path == "/api/servo":
            ch = int(q["ch"])
            if ch not in SERVO_CHANNELS:
                raise ValueError("알 수 없는 채널: %d" % ch)
            with self.lock:
                cur = robot.servos.get(ch)
                target = float(q["angle"]) if "angle" in q else (90.0 if cur is None else cur) + float(q.get("delta", 0))
                robot.servos.move_to({ch: target})
            log.info("servo %s → %.0f", C.CH_NAMES[ch], robot.servos.get(ch))
        elif path == "/api/pose":
            with self.lock:
                robot.arm.pose(q["name"], side=q.get("side") or None)
        elif path == "/api/grip":
            with self.lock:
                if q.get("state") == "open":
                    robot.arm.grip_open()
                else:
                    robot.arm.grip_close()
        elif path == "/api/base":
            with self.lock:
                robot.arm.base_to(q.get("side", "CENTER"))
        elif path == "/api/init":
            self.driving.stop(why="모터 초기화")
            with self.lock:
                robot.init_motors()
            self.driving.steer_dir = 0
        elif path == "/api/camera":
            self.set_camera(q["name"], float(q["value"]))
        elif path == "/api/crosshair":
            self.vision.crosshair = not self.vision.crosshair
        elif path == "/api/save":
            return {"path": self.save_frame("snapshot")}
        elif path == "/api/analyze":
            return self.analyze()
        elif path == "/api/record":
            return {"line": self.record(q.get("label", "").strip())}
        else:
            raise ValueError("알 수 없는 요청: %s" % path)
        return {}

    def set_camera(self, name: str, value: float):
        if self.picam is None:
            raise HardwareError("picamera2 카메라가 아니라서 조절할 수 없습니다")
        spec = {s[0]: s for s in CAMERA_CONTROL_SPEC}
        if name not in spec:
            raise ValueError("조절할 수 없는 항목: %s" % name)
        value = max(spec[name][1], min(spec[name][2], value))
        self.picam.set_controls({name: value})
        self.controls[name] = value
        log.info("camera %s = %.1f", name, value)

    def save_frame(self, kind: str, frame=None) -> str:
        frame = self.vision.latest_frame() if frame is None else frame
        if frame is None:
            raise HardwareError("카메라 프레임이 아직 없습니다")
        os.makedirs(C.LOG_DIR, exist_ok=True)
        path = os.path.join(C.LOG_DIR, "debug_%s_%s.jpg" % (kind, time.strftime("%Y%m%d_%H%M%S")))
        cv2.imwrite(path, frame)
        log.info("사진 저장: %s", path)
        return path

    def analyze(self) -> dict:
        """지금 화면을 여러 전처리로 읽어 보고 어떤 방법이 되는지 비교 (QR 인식 개선 방향 찾기)"""
        frame = self.vision.latest_frame()
        if frame is None:
            raise HardwareError("카메라 프레임이 아직 없습니다")
        path = self.save_frame("analyze", frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        h, w = gray.shape[:2]
        middle = gray[h // 4: h * 3 // 4, w // 4: w * 3 // 4]
        variants = [
            ("원본", gray),
            ("1.5배 확대", cv2.resize(gray, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)),
            ("2배 확대", cv2.resize(gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)),
            ("가운데만 2배 (먼 QR)", cv2.resize(middle, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)),
            ("대비 향상 (CLAHE)", cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)),
            ("이진화 (Otsu)", cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]),
            ("적응형 이진화", cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 5)),
            ("흐리게 (모니터 줄무늬 제거)", cv2.GaussianBlur(gray, (5, 5), 0)),
            ("선명하게", cv2.filter2D(gray, -1, np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32))),
            ("좌우 반전", cv2.flip(gray, 1)),
        ]
        det = cv2.QRCodeDetector()
        results = []
        for name, img in variants:
            t0 = time.monotonic()
            results.append({"method": name, "pyzbar": _pyzbar_texts(self.vision._pyzbar, img),
                            "opencv": _opencv_texts(det, img), "ms": round((time.monotonic() - t0) * 1000)})
        with self.vision.qr_lock:
            mission = self.robot.qr.decode(frame)
        log.info("분석: 미션 방식=%s, 읽힌 방법=%s", mission,
                 [r["method"] for r in results if r["pyzbar"] or r["opencv"]])
        return {"results": results, "mission": mission, "path": path}

    def record(self, label: str) -> str:
        get = self.robot.servos.get
        angles = " ".join("%s=%s" % (C.CH_NAMES[ch], "?" if get(ch) is None else "%d" % round(get(ch)))
                          for ch in SERVO_CHANNELS)
        line = "%s  %-20s %s" % (time.strftime("%H:%M:%S"), label or "-", angles)
        os.makedirs(C.LOG_DIR, exist_ok=True)
        with open(os.path.join(C.LOG_DIR, "debug_poses.txt"), "a", encoding="utf-8") as f:
            f.write(line + "\n")
        self.records.append(line)
        log.info("기록: %s", line)
        return line


class Handler(BaseHTTPRequestHandler):
    app: App = None
    protocol_version = "HTTP/1.1"            # 연결을 재사용: 주행 명령(0.15초마다)마다 새로 연결하면 PC 에 따라 0.3초씩 걸림
    disable_nagle_algorithm = True           # 재사용하는 연결에서 응답이 지연되지 않게
    timeout = 30                             # 와이파이가 끊겨 사라진 연결을 정리

    def log_message(self, fmt, *args):
        log.debug("http " + fmt, *args)

    def _send(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code: int = 200):
        self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/status":
            self._json(self.app.status())
        elif path == "/stream":
            self._stream()
        elif path == "/snapshot.jpg":
            frame = self.app.vision.latest_frame()
            ok, buf = (False, None) if frame is None else cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if ok:
                self._send(200, buf.tobytes(), "image/jpeg")
            else:
                self._send(503, "카메라 프레임이 아직 없습니다".encode("utf-8"), "text/plain; charset=utf-8")
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        self.rfile.read(int(self.headers.get("Content-Length") or 0))   # 연결을 재사용하므로 본문이 있으면 비워 둠
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        try:
            body, code = dict(ok=True, **self.app.action(u.path, q)), 200
        except KeyError as e:                    # 없는 포즈 이름 등: 메시지만 전달
            body, code = {"ok": False, "error": str(e.args[0]) if e.args else "KeyError"}, 400
        except Exception as e:
            log.warning("요청 실패 %s: %s", self.path, e)
            body, code = {"ok": False, "error": "%s: %s" % (type(e).__name__, e)}, 400
        self._json(body, code)                   # 응답을 못 보내도(취소된 주행 명령 등) 명령은 이미 처리됨

    def _stream(self):
        """MJPEG: 새 JPEG 가 나올 때마다 이어서 보냄 (브라우저 <img> 가 동영상처럼 표시)"""
        v = self.app.vision
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")  # 길이를 모르는 응답: 끊어서 끝을 알림
        self.end_headers()
        last = 0
        try:
            while v.running:
                with v.cond:
                    v.cond.wait_for(lambda: v.jpeg_seq != last or not v.running, timeout=2.0)
                    jpeg, last = v.jpeg, v.jpeg_seq
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except OSError:
            pass


class Server(ThreadingHTTPServer):
    def handle_error(self, request, client_address):
        if not isinstance(sys.exc_info()[1], OSError):           # 브라우저가 먼저 끊었거나 와이파이가 끊긴 것은 오류가 아님
            super().handle_error(request, client_address)


# =====================================================================
# 웹 화면 (HTML + JS)
# =====================================================================
PAGE = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PiCar 디버그</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; background: #121212; color: #e8e8e8; font: 14px/1.45 system-ui, -apple-system, "Malgun Gothic", sans-serif; }
  .wrap { display: flex; flex-wrap: wrap; gap: 12px; padding: 12px; }
  .left { flex: 3 1 520px; min-width: 0; }
  .right { flex: 2 1 340px; min-width: 0; }
  .card { background: #1d1d1d; border: 1px solid #333; border-radius: 8px; padding: 10px 12px; margin-top: 12px; }
  .right .card:first-child { margin-top: 0; }
  h2 { font-size: 14px; margin: 0 0 8px; }
  #cam { width: 100%; display: block; border-radius: 8px; background: #000; min-height: 180px; max-height: 60vh; object-fit: contain; }
  .drive { display: flex; flex-wrap: wrap; align-items: center; gap: 14px; }
  .pad { display: grid; grid-template-columns: repeat(3, 64px); grid-auto-rows: 54px; gap: 6px; }
  .pad button { padding: 0; font-size: 20px; line-height: 1.15; touch-action: none; user-select: none;
                -webkit-user-select: none; -webkit-touch-callout: none; }
  .pad small { display: block; font-size: 11px; color: #aaa; }
  .pad button.on { background: #1e4f30; border-color: #5fd38a; }
  .pad button.stop { background: #4a1f1f; border-color: #8a3a3a; }
  .dinfo { flex: 1 1 200px; min-width: 0; }
  .dist { font: 700 26px ui-monospace, Consolas, monospace; }
  .stale { opacity: .35; }
  button:disabled { opacity: .5; cursor: default; }
  .row { display: flex; align-items: center; gap: 6px; flex-wrap: wrap; margin: 4px 0; }
  .name { width: 70px; font-weight: 600; }
  .val { width: 46px; text-align: right; font: 600 18px ui-monospace, Consolas, monospace; }
  .sub { color: #9a9a9a; font-size: 12px; font-weight: normal; }
  button, select, input { font: inherit; color: #e8e8e8; background: #2a2a2a; border: 1px solid #444; border-radius: 6px; padding: 5px 9px; }
  button { cursor: pointer; }
  button:hover { background: #363636; }
  button.primary { background: #1e4f30; border-color: #2e7a4a; }
  input[type=number] { width: 64px; }
  input[type=range] { flex: 1; min-width: 120px; padding: 0; }
  pre { background: #0b0b0b; border: 1px solid #2a2a2a; border-radius: 6px; padding: 8px; margin: 6px 0; white-space: pre-wrap; word-break: break-all; font: 12.5px ui-monospace, Consolas, monospace; }
  .big { font-size: 20px; font-weight: 700; }
  .ok { color: #5fd38a; } .bad { color: #ff7b72; } .warn { color: #f2cc60; }
  ul { margin: 6px 0; padding-left: 18px; }
  table { border-collapse: collapse; width: 100%; font-size: 13px; }
  th, td { border-bottom: 1px solid #2f2f2f; padding: 4px 6px; text-align: left; vertical-align: top; }
  #toast { position: fixed; left: 50%; bottom: 16px; transform: translateX(-50%); background: #333; border: 1px solid #555; padding: 8px 14px; border-radius: 6px; display: none; max-width: 90vw; }
  @media (max-width: 420px) {
    .wrap { padding: 8px; }
    .name { width: 58px; }
    .val { width: 36px; }
    input[type=number] { width: 52px; }
    .servo-hint { margin-left: 0 !important; }
  }
  @media (orientation: landscape) and (max-height: 500px) {    /* 폰 가로: 카메라와 주행 버튼을 나란히 */
    .left { flex-basis: 100%; display: grid; grid-template-columns: minmax(200px, 1fr) minmax(0, 440px); gap: 12px; align-items: start; }
    .dinfo { flex-basis: 170px; }
    .dinfo input[type=range] { min-width: 80px; }
    #cam { max-height: calc(100vh - 24px); }
    .left > .card:first-of-type { margin-top: 0; }
    .left > .card:not(:first-of-type) { grid-column: 1 / -1; }
  }
</style>
</head>
<body>
<div class="wrap">
  <div class="left">
    <img id="cam" src="/stream" alt="카메라 화면" onerror="setTimeout(() => this.src = '/stream?' + Date.now(), 1000)">
    <div class="card">
      <h2>주행 <span class="sub">누르고 있는 동안만 움직이고 떼면 멈춤 · 연결이 끊기면 약 0.5초 뒤 자동 정지</span></h2>
      <div class="drive">
        <div class="pad">
          <span></span><button class="hold" data-kind="drive" data-dir="1">▲<small>전진</small></button><span></span>
          <button class="hold" data-kind="steer" data-dir="-1">◀<small>왼쪽</small></button>
          <button class="stop" onpointerdown="stopDrive()">■<small>정지</small></button>
          <button class="hold" data-kind="steer" data-dir="1">▶<small>오른쪽</small></button>
          <span></span><button class="hold" data-kind="drive" data-dir="-1">▼<small>후진</small></button><span></span>
        </div>
        <div class="dinfo">
          <div class="sub">초음파 거리 · 바퀴 <span id="motor">-</span></div>
          <div id="dist" class="dist">-</div>
          <div id="zone" class="sub">-</div>
          <div class="row">속도
            <input type="range" id="speed" min="0" max="100" step="1" value="40"
                   oninput="$('speedv').textContent = this.value" onpointerup="this.blur()">
            <span id="speedv" style="width:28px;text-align:right">40</span>
          </div>
          <div class="row">
            <button class="pbtn" onclick="pulse(1)">펄스 ▲</button><button class="pbtn" onclick="pulse(-1)">펄스 ▼</button>
            <span id="pulseinfo" class="sub"></span>
          </div>
          <div id="pulsehint" class="sub"></div>
        </div>
      </div>
      <div class="sub">키보드: ↑ ↓ 전진/후진 · ← → 조향 · 스페이스 정지</div>
    </div>
    <div class="card">
      <h2>QR 인식 <span class="sub">초록 테두리 = 읽힘 · 노랑 = QR 은 보이는데 해독 실패</span></h2>
      <div id="qr" class="big">-</div>
      <div id="qrinfo" class="sub"></div>
      <ul id="hints"></ul>
      <div class="row">
        <button onclick="analyze()">분석: 여러 방법으로 읽어 보기</button>
        <button onclick="post('/api/save').then(j => j.ok && toast('저장: ' + j.path))">원본 사진 저장</button>
        <button onclick="window.open('/snapshot.jpg', '_blank')">원본 보기</button>
        <button onclick="post('/api/crosshair')">십자선</button>
      </div>
      <div id="analysis"></div>
    </div>
    <div class="card" id="camcard" style="display:none">
      <h2>카메라 조절 <span class="sub">잘 읽히는 값을 찾으면 아래 줄을 config.py 에</span></h2>
      <div id="ctls"></div>
      <pre id="ctlcfg"></pre>
    </div>
  </div>
  <div class="right">
    <div class="card">
      <h2>서보 각도</h2>
      <div class="row sub">스텝
        <select id="step"><option>1</option><option>2</option><option selected>5</option><option>10</option></select>
        키보드: a/d base · w/s shoulder · i/k wrist · [ ] grip · , . steer
      </div>
      <div id="servos"></div>
    </div>
    <div class="card">
      <h2>config 에 붙여넣기</h2>
      <pre id="cfg">-</pre>
      <div class="row">
        <button onclick="copyCfg()">복사</button>
        <input id="label" placeholder="이름 (예: pick_grab LEFT)" style="flex:1;min-width:140px">
        <button onclick="record()">기록</button>
      </div>
      <pre id="records" class="sub" style="display:none"></pre>
    </div>
    <div class="card">
      <h2>자세 · 집게 · 방향</h2>
      <div class="row">side
        <select id="side"><option value="">없음</option><option>LEFT</option><option>RIGHT</option></select>
        <span class="sub">pick_* / place_* 포즈의 쪽별 보정용</span>
      </div>
      <div class="row" id="poses"></div>
      <div class="row">
        <button onclick="post('/api/grip?state=open')">집게 열기</button>
        <button onclick="post('/api/grip?state=close')">집게 닫기</button>
      </div>
      <div class="row">
        <button onclick="post('/api/base?side=LEFT')">base 왼쪽</button>
        <button onclick="post('/api/base?side=CENTER')">가운데</button>
        <button onclick="post('/api/base?side=RIGHT')">오른쪽</button>
      </div>
      <div class="row"><button class="primary" onclick="post('/api/init')">모터 초기화 (기본값)</button></div>
    </div>
  </div>
</div>
<div id="toast"></div>
<script>
const SERVOS = [
  [0, "steer", "↑ 오른쪽 / ↓ 왼쪽 (주행 ◀▶ 를 떼면 config 중립으로 돌아감)"],
  [1, "base", "↑ 팔 왼쪽 / ↓ 오른쪽"],
  [2, "shoulder", "↑ 팔 내리며 앞으로 / ↓ 올리기"],
  [3, "wrist", "↑ 손 들어 앞으로 / ↓ 내리기"],
  [4, "grip", "↑ 벌리기 / ↓ 잡기"],
];
const KEYS = {a: [1, 1], d: [1, -1], w: [2, -1], s: [2, 1], i: [3, 1], k: [3, -1], "[": [4, -1], "]": [4, 1], ",": [0, -1], ".": [0, 1]};
const $ = id => document.getElementById(id);
const esc = v => String(v).replace(/[&<>"]/g, c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
let posesBuilt = false, ctlsBuilt = false;

function toast(msg) {
  const t = $("toast");
  t.textContent = msg;
  t.style.display = "block";
  clearTimeout(t._h);
  t._h = setTimeout(() => t.style.display = "none", 3000);
}

async function post(url) {
  try {
    const j = await (await fetch(url, {method: "POST"})).json();
    if (!j.ok) toast("실패: " + j.error);
    refresh();
    return j;
  } catch (e) {
    toast("연결 실패: " + e);
    return {ok: false};
  }
}

function nudge(ch, dir) { post(`/api/servo?ch=${ch}&delta=${dir * Number($("step").value)}`); }
function setAngle(ch) { const v = $("in" + ch).value; if (v !== "") post(`/api/servo?ch=${ch}&angle=${v}`); }

function buildServos() {
  $("servos").innerHTML = SERVOS.map(([ch, name, hint]) => `
    <div class="row">
      <span class="name">${name}</span><span class="val" id="v${ch}">-</span>
      <button onclick="nudge(${ch}, -1)">−</button><button onclick="nudge(${ch}, 1)">+</button>
      <input type="number" id="in${ch}" min="0" max="180" placeholder="각도" onkeydown="if (event.key === 'Enter') setAngle(${ch}); else if (event.key === 'Escape') this.blur();">
      <button onclick="setAngle(${ch})">이동</button>
    </div>
    <div class="sub servo-hint" style="margin:-2px 0 6px 76px">${hint}</div>`).join("");
}

function buildPoses(names) {
  $("poses").innerHTML = names.map(n => `<button onclick="post('/api/pose?name=${esc(n)}&side=' + $('side').value)">${esc(n)}</button>`).join("");
  posesBuilt = true;
}

function buildControls(spec, values) {
  $("camcard").style.display = "block";
  $("ctls").innerHTML = spec.map(([name, lo, hi, step]) => `
    <div class="row"><span style="width:110px">${name}</span>
      <input type="range" min="${lo}" max="${hi}" step="${step}" value="${values[name]}"
             oninput="$('cv_${name}').textContent = this.value" onpointerup="this.blur()"
             onchange="post('/api/camera?name=${name}&value=' + this.value)">
      <span id="cv_${name}" style="width:40px;text-align:right">${values[name]}</span></div>`).join("");
  ctlsBuilt = true;
}

async function getStatus() {             // 연결이 막혔을 때 오래 붙잡혀 있지 않게 시간 초과를 둠
  const ctl = new AbortController(), timer = setTimeout(() => ctl.abort(), STATUS_TIMEOUT_MS);
  try {
    return await (await fetch("/status", {signal: ctl.signal})).json();
  } finally {
    clearTimeout(timer);
  }
}

async function refresh() {
  if (refreshing) return;                // 이전 요청이 안 끝났으면 건너뜀: 연결이 느릴 때 쌓여서 정지 명령의 자리를 차지하지 않게
  refreshing = true;
  let s;
  try {
    s = await getStatus();
  } catch (e) {
    $("qr").innerHTML = "<span class='bad'>연결 끊김 (debug_tool.py 가 실행 중인지 확인)</span>";
    $("dist").classList.add("stale");    // 멈춘 값을 실시간 값으로 착각하지 않게
    $("zone").innerHTML = "<span class='bad'>연결 끊김 (값 없음)</span>";
    $("motor").textContent = "?";
    return;
  } finally {
    refreshing = false;
  }
  // 끊어진 카메라 스트림은 오류 이벤트가 안 오므로, 연결이 끊겼다 돌아오거나 PC 가 잠들었다 깨면 다시 연결
  if (Date.now() - statusAt > 3000) $("cam").src = "/stream?" + Date.now();
  statusAt = Date.now();
  if (!dcfg) {
    dcfg = s.drive_cfg;
    $("speed").value = dcfg.speed;
    $("speedv").textContent = $("speed").value;
    $("pulsehint").textContent = `펄스 = 정밀 정지에서 쓰는 한 번 (속도 ${dcfg.creep_speed}, ${dcfg.creep_s}초)`;
  }
  showDrive(s.drive);
  if (steerDirty) {                      // 페이지를 새로 열었거나 조향 명령이 실패했으면 서버와 맞춤
    steerDirty = false;
    if (s.drive.steer !== want("steer")) {
      steerSent = want("steer");
      cmd(`/api/steer?dir=${steerSent}`, "steer");
    }
  }
  for (const [ch, name] of SERVOS) {
    const a = s.servos[name];
    $("v" + ch).textContent = a == null ? "?" : Math.round(a);
  }
  $("cfg").textContent = s.config;
  if (!posesBuilt) buildPoses(s.poses);
  if (s.controls && !ctlsBuilt) buildControls(s.control_spec, s.controls);
  if (s.controls_text) $("ctlcfg").textContent = s.controls_text;
  const q = s.qr;
  if (q.start) {
    $("qr").innerHTML = `<span class="ok">출발 QR: ${esc(q.start)}</span> <span class="sub">(연속 ${q.streak}번 · 미션은 ${s.confirm}번 연속이면 출발)</span>`;
  } else if (q.texts.length) {
    $("qr").innerHTML = `<span class="warn">읽힘: ${esc(q.texts.join(", "))}</span>`;
  } else {
    $("qr").innerHTML = "<span class='bad'>QR 못 읽음</span>";
  }
  const boxes = s.boxes.map(b => `${b.text == null ? "해독 실패" : b.text} ${b.size}px`).join(", ") || "화면에 QR 없음";
  const cam = s.camera;
  $("qrinfo").textContent = `QR 위치: ${boxes} · 밝기 ${q.brightness ?? "-"} · 선명도 ${q.sharpness ?? "-"} · 인식 ${q.decode_ms ?? "-"}ms`
    + ` · 카메라 ${cam.kind} ${cam.size.join("x")} ${cam.fps}fps` + (cam.error ? " · " + cam.error : "");
  $("hints").innerHTML = q.hints.map(h => `<li>${esc(h)}</li>`).join("");
  if (s.records.length) {
    $("records").style.display = "block";
    $("records").textContent = s.records.join("\n");
  }
}

async function analyze() {
  $("analysis").innerHTML = "<p class='sub'>분석 중... (몇 초 걸릴 수 있어요)</p>";
  const j = await post("/api/analyze");
  if (!j.ok) { $("analysis").innerHTML = ""; return; }
  const cell = v => v == null ? "<span class='sub'>설치 안 됨</span>"
    : (v.length ? `<span class="ok">${esc(v.join(", "))}</span>` : "<span class='bad'>✗</span>");
  const rows = j.results.map(r => `<tr><td>${esc(r.method)}</td><td>${cell(r.pyzbar)}</td><td>${cell(r.opencv)}</td><td class="sub">${r.ms}ms</td></tr>`).join("");
  $("analysis").innerHTML = `
    <p>미션 방식으로 읽은 결과: ${j.mission.length ? `<b class="ok">${esc(j.mission.join(", "))}</b>` : "<b class='bad'>못 읽음</b>"}
       <span class="sub">· 사진 저장: ${esc(j.path)}</span></p>
    <table><tr><th>방법</th><th>pyzbar</th><th>OpenCV</th><th></th></tr>${rows}</table>
    <p class="sub">원본은 못 읽는데 다른 방법은 읽힌다면 그 방법을 인식 코드에 넣으면 됩니다.</p>`;
}

function copyCfg() {
  const text = $("cfg").textContent;
  const fallback = () => {
    const r = document.createRange();
    r.selectNodeContents($("cfg"));
    getSelection().removeAllRanges();
    getSelection().addRange(r);
    toast("선택했어요. Ctrl+C 로 복사하세요");
  };
  if (navigator.clipboard && window.isSecureContext) navigator.clipboard.writeText(text).then(() => toast("복사했어요"), fallback);
  else fallback();
}

function record() {
  post("/api/record?label=" + encodeURIComponent($("label").value)).then(j => j.ok && toast("기록: " + j.line));
}

// ---- 주행: 누르는 동안 BEAT_MS 마다 같은 명령을 다시 보냄 (서버는 DRIVE_HOLD_S 동안 명령이 없으면 자동 정지) ----
const BEAT_MS = 150, BEAT_TIMEOUT_MS = 400, STATUS_TIMEOUT_MS = 1500;
const DRIVE_KEYS = {ArrowUp: ["drive", 1], ArrowDown: ["drive", -1], ArrowLeft: ["steer", -1], ArrowRight: ["steer", 1]};
const ZONES = {RED: ["ok", "빨간 선 (만점)"], YELLOW: ["warn", "노란 선 (감점)"], OVER: ["bad", "정지선 넘어감 (실격)"], SHORT: ["", "정지선 전"]};
const SID = Math.random().toString(36).slice(2, 10);   // 이 탭의 id: 서버가 탭마다 명령 순번을 보고 늦게 도착한 옛 명령을 거름
const seqs = {drive: 0, steer: 0};
const holds = {drive: [], steer: []};                  // 누르고 있는 버튼/키 [출처, 방향]. 나중에 누른 것이 우선
let beat = null, beatCtl = null, steerSent = 0, steerDirty = true, autoStops = null, dcfg = null;
let refreshing = false, statusAt = Date.now(), pulsing = false, movedAt = 0, failAt = 0;

function want(kind) { const h = holds[kind]; return h.length ? h[h.length - 1][1] : 0; }

async function cmd(path, kind, signal) {
  const url = path + (path.includes("?") ? "&" : "?") + `sid=${SID}&seq=${++seqs[kind]}`;
  try {
    const j = await (await fetch(url, {method: "POST", signal, keepalive: true})).json();
    if (!j.ok) {
      toast("실패: " + j.error);
      if (kind === "steer") steerDirty = true;
    }
    if (j.drive) showDrive(j.drive);
    return j;
  } catch (e) {
    if (kind === "steer") steerDirty = true;
    if (e.name !== "AbortError") {
      if (Date.now() - failAt > 3000) toast("연결 실패: " + e + " → 바퀴는 스스로 멈춥니다");
      failAt = Date.now();
    }
    return {ok: false};
  }
}

async function sendDrive() {             // 응답을 기다리는 중이면 건너뜀: 와이파이가 밀릴 때 명령이 쌓였다가 뗀 뒤에 한꺼번에 도착하지 않게
  const d = want("drive");
  if (!d || beatCtl) return;
  const ctl = beatCtl = new AbortController(), timer = setTimeout(() => ctl.abort(), BEAT_TIMEOUT_MS);
  movedAt = Date.now();
  await cmd(`/api/drive?dir=${d}&speed=${$("speed").value}`, "drive", ctl.signal);
  clearTimeout(timer);
  if (beatCtl === ctl) beatCtl = null;
}

function abortBeat() {
  if (beatCtl) beatCtl.abort();
  beatCtl = null;
}

function stopBeat() {                    // 뗄 때 / 정지할 때: 밀려 있는 주행 명령을 취소하고 나서 정지 명령을 보냄
  clearInterval(beat);
  beat = null;
  abortBeat();
  movedAt = Date.now();
}

function paintPad() {
  for (const b of document.querySelectorAll(".pad .hold"))
    b.classList.toggle("on", holds[b.dataset.kind].length > 0 && want(b.dataset.kind) === Number(b.dataset.dir));
}

function apply(kind) {
  const d = want(kind);
  if (kind === "steer") {
    if (d !== steerSent) { steerSent = d; cmd(`/api/steer?dir=${d}`, "steer"); }
  } else if (d) {
    abortBeat();                         // 새로 눌렀거나 방향이 바뀌었으면 기다리지 않고 바로 보냄
    sendDrive();
    if (!beat) beat = setInterval(sendDrive, BEAT_MS);
  } else {
    stopBeat();
    cmd("/api/drive?dir=0", "drive");
  }
  paintPad();
}

function hold(kind, src, dir) {
  holds[kind] = holds[kind].filter(h => h[0] !== src).concat([[src, dir]]);
  apply(kind);
}

function release(kind, src) {
  const n = holds[kind].length;
  holds[kind] = holds[kind].filter(h => h[0] !== src);
  if (holds[kind].length !== n) apply(kind);
}

function releaseAll() {                  // 창을 떠나면 키/손가락을 떼는 신호를 못 받으므로 전부 뗀 것으로
  for (const kind of ["drive", "steer"]) if (holds[kind].length) { holds[kind] = []; apply(kind); }
}

function stopDrive() {
  holds.drive = [];
  stopBeat();
  cmd("/api/stop", "drive");
  paintPad();
}

function showDrive(d) {
  $("dist").classList.remove("stale");
  $("dist").textContent = d.cm == null ? "-" : d.cm.toFixed(1) + " cm";
  const target = dcfg ? dcfg.target : 30;
  let z;
  if (d.error) z = `<span class="bad">${esc(d.error)}</span>`;
  else if (d.cm == null) z = "측정 안 됨 (2~150cm 밖이거나 반사가 안 옴)";
  else {
    const [cls, name] = ZONES[d.zone], diff = d.cm - target;
    z = `<span class="${cls}">${name}</span> · 목표 ${target}cm 에서 ${diff >= 0 ? "+" : "−"}${Math.abs(diff).toFixed(1)}`;
  }
  if (d.age != null && d.age > 1.5) z += ` <span class="warn">(${d.age}초 전 값)</span>`;
  $("zone").innerHTML = z;
  $("motor").textContent = (d.dir ? `${d.dir > 0 ? "전진" : "후진"} ${d.speed}` : "정지")
    + ({"-1": " · 왼쪽", "1": " · 오른쪽"}[d.steer] || "");
  if (autoStops != null && d.auto_stops > autoStops) toast("바퀴를 자동으로 멈췄어요 (주행 명령이 끊겼거나 모터 명령이 실패함)");
  autoStops = d.auto_stops;
}

function setPulsing(on) {
  pulsing = on;
  for (const b of document.querySelectorAll(".pbtn")) b.disabled = on;
}

async function pulse(dir) {              // 한 번에 하나만: 앞 펄스의 이동 거리를 다 잰 뒤에 다음 펄스
  if (pulsing) return;
  const settleMs = (dcfg ? dcfg.settle_s : 0.7) * 1000;
  if (Date.now() - movedAt < settleMs) {
    $("pulseinfo").textContent = "방금 움직여서 거리가 아직 안정되지 않았어요. 잠시 뒤 다시 눌러 주세요";
    return;
  }
  setPulsing(true);
  try {
    const before = (await getStatus()).drive.cm;
    const ctl = new AbortController(), timer = setTimeout(() => ctl.abort(), STATUS_TIMEOUT_MS);
    const j = await cmd(`/api/pulse?dir=${dir}`, "drive", ctl.signal);
    clearTimeout(timer);
    if (!j.ok) {
      $("pulseinfo").textContent = "응답을 못 받았어요";
      return;
    }
    const t = movedAt = Date.now();
    $("pulseinfo").textContent = "재는 중...";
    await new Promise(r => setTimeout(r, settleMs + 500));    // 멈춘 뒤 초음파 값이 안정될 때까지 기다림
    if (movedAt !== t) {
      $("pulseinfo").textContent = "재는 중에 움직여서 이동 거리를 못 쟀어요";
      return;
    }
    const s = await getStatus();
    showDrive(s.drive);
    const after = s.drive.cm;
    $("pulseinfo").textContent = before == null || after == null ? "거리를 못 읽었어요"
      : `${before.toFixed(1)} → ${after.toFixed(1)} cm (누른 방향으로 ${((before - after) * dir).toFixed(1)}cm 이동)`;
  } catch (e) {
    $("pulseinfo").textContent = "";
  } finally {
    setPulsing(false);
  }
}

for (const b of document.querySelectorAll(".pad .hold")) {
  b.addEventListener("pointerdown", e => {
    if (e.button !== 0) return;
    e.preventDefault();
    try { b.setPointerCapture(e.pointerId); } catch (_) {}
    hold(b.dataset.kind, "p" + e.pointerId, Number(b.dataset.dir));
  });
  b.addEventListener("lostpointercapture", e => { release(b.dataset.kind, "p" + e.pointerId); });
}
for (const type of ["pointerup", "pointercancel"])
  window.addEventListener(type, e => { release("drive", "p" + e.pointerId); release("steer", "p" + e.pointerId); });
window.addEventListener("contextmenu", e => {
  if (e.target.closest && e.target.closest(".pad")) e.preventDefault();      // 버튼을 길게 눌러도 메뉴가 안 뜨게
  else releaseAll();                     // 메뉴가 열리면 키를 떼는 신호를 못 받음
});

const typing = t => t.tagName === "TEXTAREA" || (t.tagName === "INPUT" && t.type === "text");

for (const el of [$("step"), $("side")]) {       // 마우스/터치로 고른 뒤에만 포커스를 풂. 키보드로 고르는 중에 풀면 다음 방향키가 바퀴를 움직임
  el.addEventListener("pointerdown", () => { el.byPointer = true; });
  el.addEventListener("keydown", () => { el.byPointer = false; });
  el.addEventListener("change", () => { if (el.byPointer) el.blur(); });
}

document.addEventListener("keydown", e => {
  if (e.key === "Meta") {                // macOS: Cmd 를 누른 동안에는 다른 키를 떼도 keyup 이 안 옴
    releaseAll();
    return;
  }
  if (e.key === " " && !typing(e.target)) {      // 스페이스 = 정지: 글자를 입력하는 칸이 아니면 언제나
    e.preventDefault();
    if (!e.repeat) stopDrive();
    return;
  }
  if (["INPUT", "SELECT", "TEXTAREA"].includes(e.target.tagName) || e.metaKey || e.ctrlKey || e.altKey) return;
  const dk = DRIVE_KEYS[e.key], k = KEYS[e.key];
  if (dk) {
    e.preventDefault();
    if (!e.repeat) hold(dk[0], "k" + e.key, dk[1]);
  } else if (k) {
    e.preventDefault();
    nudge(k[0], k[1]);
  }
});
document.addEventListener("keyup", e => {
  const dk = DRIVE_KEYS[e.key];
  if (dk) release(dk[0], "k" + e.key);
  else if (e.key === "Meta") releaseAll();
  else if (e.key === " " && !typing(e.target)) e.preventDefault();          // 포커스된 버튼이 눌리지 않게
});
window.addEventListener("blur", releaseAll);
document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAll(); });
window.addEventListener("pagehide", () => {
  if (!navigator.sendBeacon) return;
  navigator.sendBeacon(`/api/stop?sid=${SID}&seq=${++seqs.drive}`);
  if (steerSent) navigator.sendBeacon(`/api/steer?dir=0&sid=${SID}&seq=${++seqs.steer}`);
});

buildServos();
refresh();
setInterval(refresh, 500);
</script>
</body>
</html>
"""


# =====================================================================
# main
# =====================================================================
def page_urls(port: int) -> list:
    urls = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))          # 패킷은 보내지 않고 내 IP 만 확인
        urls.append("http://%s:%d" % (s.getsockname()[0], port))
        s.close()
    except OSError:
        pass
    urls.append("http://%s.local:%d" % (socket.gethostname(), port))
    return urls


def _ignore_exit_signals():
    for s in EXIT_SIGNALS:
        signal.signal(s, signal.SIG_IGN)


def _exit_on_signal(signum, frame):
    """Ctrl+C / kill(SIGTERM) / 터미널 닫힘(SIGHUP) → finally 로 가서 바퀴를 세움. 두 번째 신호가 정리를 끊지 못하게 막음"""
    _ignore_exit_signals()
    raise KeyboardInterrupt


def _log_in_background():
    """로그 출력을 별도 스레드로: 터미널이 멈춰도(Ctrl+S 등) 로그를 남기던 스레드가 같이 멈춰서 바퀴 정지가 막히지 않게"""
    root = logging.getLogger()
    q = queue.Queue()
    listener = logging.handlers.QueueListener(q, *root.handlers, respect_handler_level=True)
    root.handlers[:] = [logging.handlers.QueueHandler(q)]
    listener.start()
    return listener


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="카메라(QR) + 서보 각도 + 수동 주행 디버깅 웹 도구", epilog=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--stream-width", type=int, default=960, help="화면 전송 폭(px). 끊기면 640 으로")
    p.add_argument("--stream-fps", type=float, default=12)
    p.add_argument("--stop-service", action="store_true", help="실행 중인 Adeept_Robot.service 를 자동 정지")
    p.add_argument("--no-service-check", action="store_true")
    p.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING"])
    args = p.parse_args(argv)
    setup_logging(args.log_level, to_file=False)
    if not check_service(args):
        return 2
    try:
        server = Server(("0.0.0.0", args.port), Handler)
    except OSError as e:
        log.error("포트 %d 를 열 수 없습니다 (%s) → --port 8080 처럼 바꿔 보세요", args.port, e)
        return 4
    server.daemon_threads = True
    for s in EXIT_SIGNALS:
        if signal.getsignal(s) != signal.SIG_IGN:   # nohup 으로 실행했으면 터미널을 닫아도 계속 돌도록 그대로 둠
            signal.signal(s, _exit_on_signal)
    listener = _log_in_background()

    robot = Robot()
    vision = driving = None
    code = 0
    try:
        robot.init_motors()                         # run_mission.py 처럼 모터 초기화부터
        log.info("camera: %s", robot.camera.kind)   # 카메라를 먼저 열어서 실패하면 바로 알림
        vision = Vision(robot, args.stream_width, args.stream_fps)
        vision.start()
        driving = Driving(robot)
        driving.start()
        Handler.app = App(robot, vision, driving)
        print("\n  디버그 화면 →  " + "   ".join(page_urls(args.port)))
        print("  (노트북/폰이 파이와 같은 와이파이에 있어야 합니다. 끝내려면 Ctrl+C → 바퀴는 멈추고 서보는 그 자리에 둡니다)\n",
              flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("종료")
    except HardwareError as e:
        log.error("하드웨어 오류: %s", e)
        code = 3
    except Exception as e:
        log.exception("오류: %s", e)
        code = 4
    finally:
        _ignore_exit_signals()                      # Ctrl+C 를 연달아 눌러도 끝까지 정리
        try:
            if driving is not None:
                driving.close()                     # 바퀴부터 정지, 이후 들어오는 주행 명령은 무시
            server.server_close()
            if vision is not None:
                vision.stop()
        finally:
            robot.shutdown()
            stopper = threading.Thread(target=listener.stop, daemon=True)
            stopper.start()
            stopper.join(2.0)                       # 터미널이 막혀 있어도 로그 때문에 오래 기다리지 않음
            for s in EXIT_SIGNALS:                  # 이제 프로그램 종료만 남음: 그래도 멈춰 있으면 Ctrl+C / kill 로 끝낼 수 있게
                signal.signal(s, signal.SIG_DFL)
    return code


if __name__ == "__main__":
    sys.exit(main())
