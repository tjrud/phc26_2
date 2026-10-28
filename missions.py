# -*- coding: utf-8 -*-
"""
missions.py - MISSION 1 ~ 5
===========================
각 미션은 Mission 을 상속한 클래스 하나입니다.  execute() 안에 그 미션의 동작만 있고,
시간 측정 / 예외 처리 / 로그 / 비상 정지는 Mission.run() 이 공통으로 처리합니다.

  Mission1QRStart         출발선 대기 → 모니터 QR(Left/Right) 인식 → 방향 저장
  Mission2DriveAndStop    초음파로 모니터 30cm 앞 빨간 정지선에 정밀 정지
  Mission3FirstTransfer   QR 방향 상자의 물체 → 반대편 상자
  Mission4SecondTransfer  반대편 물체 → 처음 방향 상자
  Mission5Finish          팔 정리, 모터 정지, 결과 요약/저장

MissionContext 에 미션 사이에 공유되는 상태(방향, 결과)가 들어 있습니다.
"""
import json
import logging
import os
import time
import traceback
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import config as C
from hardware import Robot, HardwareError


class MissionError(RuntimeError):
    """미션 실패 (하드웨어는 정상이지만 조건을 못 만족)"""


# =====================================================================
# 결과 / 컨텍스트
# =====================================================================
@dataclass
class MissionResult:
    number: int
    name: str
    ok: bool = False
    started: float = 0.0
    ended: float = 0.0
    info: Dict = field(default_factory=dict)
    error: Optional[str] = None

    @property
    def duration(self) -> float:
        return (self.ended or time.time()) - self.started

    def to_dict(self):
        return {"number": self.number, "name": self.name, "ok": self.ok, "duration_s": round(self.duration, 2),
                "info": self.info, "error": self.error}


class MissionContext:
    def __init__(self, direction: Optional[str] = None, pause_steps: bool = False):
        self.direction = direction            # "LEFT" / "RIGHT"  (MISSION 1 에서 결정, --direction 으로 미리 지정 가능)
        self.results: Dict[int, MissionResult] = {}
        self.t_start = time.time()
        self.pause_steps = pause_steps        # True 면 팔 동작을 한 단계씩 Enter 로 진행 (디버그)

    @property
    def opposite(self) -> Optional[str]:
        return C.opposite(self.direction) if self.direction else None

    def require_direction(self) -> str:
        if self.direction not in ("LEFT", "RIGHT"):
            raise MissionError("방향(LEFT/RIGHT)이 정해지지 않았습니다. MISSION 1 을 먼저 실행하거나 --direction 으로 지정하세요.")
        return self.direction


# =====================================================================
# 공통 베이스
# =====================================================================
class Mission:
    number = 0
    name = "base"

    def __init__(self, robot: Robot, ctx: MissionContext):
        self.robot = robot
        self.ctx = ctx
        self.log = logging.getLogger("M%d" % self.number)

    # ---- 템플릿 메서드 ----
    def run(self) -> MissionResult:
        res = MissionResult(self.number, self.name, started=time.time())
        self.log.info("========== MISSION %d. %s 시작 ==========", self.number, self.name)
        try:
            self.execute(res)
            if res.ok is False and res.error is None:
                res.ok = True            # execute 가 명시적으로 실패시키지 않았으면 성공
        except KeyboardInterrupt:
            self.robot.stop_all()
            res.error = "사용자 중단 (Ctrl+C)"
            raise
        except (MissionError, HardwareError) as e:
            self.robot.stop_all()
            res.ok, res.error = False, str(e)
            self.log.error("실패: %s", e)
        except Exception as e:
            self.robot.stop_all()
            res.ok, res.error = False, "%s: %s" % (type(e).__name__, e)
            self.log.error("예외:\n%s", traceback.format_exc())
        finally:
            res.ended = time.time()
            self.log.info("========== MISSION %d %s (%.1fs) %s ==========", self.number,
                          "성공" if res.ok else "실패", res.duration, res.info if res.info else "")
        return res

    def execute(self, res: MissionResult):
        raise NotImplementedError

    def fail(self, msg: str):
        raise MissionError(msg)


# =====================================================================
# MISSION 1. QR 코드 인식 및 출발
# =====================================================================
class Mission1QRStart(Mission):
    number = 1
    name = "QR 코드 인식 및 출발"

    def execute(self, res):
        robot, ctx = self.robot, self.ctx
        if ctx.direction:
            self.log.info("방향이 미리 지정됨(--direction %s) → QR 대기 생략", ctx.direction)
            res.info.update(direction=ctx.direction, source="preset")
            return

        # 카메라가 모니터를 보는 자세 (출발 전 준비 동작)
        robot.arm.pose("scan_qr")
        # 카메라를 미리 열어 둠 (첫 프레임 지연 제거)
        robot.read_qr()
        self.log.info("READY ▶ 출발선에서 QR 대기 중 (시작 버튼 → 5초 후 QR 표시). Ctrl+C 로 중단")

        t0 = robot.now()
        direction = self._wait_for_start_qr(t0)
        wait_s = robot.now() - t0
        ctx.direction = direction
        self.log.info("★ QR 인식: %s  (대기 %.1fs)", direction, wait_s)
        res.info.update(direction=direction, wait_s=round(wait_s, 1), source="qr")

        if C.TUCK_ARM_BEFORE_DRIVE:
            robot.arm.pose("drive", speed=1.5)

    def _wait_for_start_qr(self, t0) -> str:
        robot = self.robot
        last, streak, frames = None, 0, 0
        next_status = next_save = robot.now()
        debug_path = os.path.join(C.LOG_DIR, "qr_last.jpg")
        while True:
            now = robot.now()
            save = C.QR_SAVE_DEBUG_FRAME_S > 0 and now >= next_save
            texts = robot.read_qr(save_path=debug_path if save else None)
            if save:
                next_save = now + C.QR_SAVE_DEBUG_FRAME_S
            frames += 1

            hit = None
            for t in texts:
                if t in C.QR_START_MAP:
                    hit = C.QR_START_MAP[t]
                    break
            if hit:
                streak = streak + 1 if hit == last else 1
                last = hit
                self.log.info("QR 읽음: %s (%d/%d)", texts, streak, C.QR_CONFIRM_COUNT)
                if streak >= C.QR_CONFIRM_COUNT:
                    return hit
            else:
                if texts:
                    self.log.debug("다른 QR: %s", texts)
                last, streak = None, 0

            if now >= next_status:
                self.log.info("대기 중... %.0fs, %d 프레임, 마지막 읽은 값=%s", now - t0, frames, texts or "-")
                next_status = now + C.QR_STATUS_LOG_EVERY_S
            if C.QR_WAIT_TIMEOUT_S and now - t0 > C.QR_WAIT_TIMEOUT_S:
                self.fail("QR 대기 시간 초과 (%.0fs)" % C.QR_WAIT_TIMEOUT_S)
            robot.sleep(C.QR_POLL_INTERVAL_S)


# =====================================================================
# MISSION 2. 직진 주행 및 정밀 정지
# =====================================================================
class Mission2DriveAndStop(Mission):
    number = 2
    name = "직진 주행 및 정밀 정지"

    def execute(self, res):
        robot = self.robot
        d0 = robot.ultra.read()
        self.log.info("출발 전 거리: %s cm (목표 %.1f)", "%.1f" % d0 if d0 is not None else "측정 실패", C.STOP_TARGET_CM)
        if d0 is None:
            self.fail("초음파 측정값이 없습니다 (센서/배선 확인, --test-ultra)")
        if d0 <= C.STOP_TARGET_CM + C.RED_HALF_WIDTH_CM:
            self.fail("이미 정지선 안쪽입니다 (%.1fcm). 로봇을 출발선에 두세요." % d0)
        res.info["start_cm"] = round(d0, 1)

        stopper = PrecisionStopper(robot, self.log)
        result = stopper.run()
        res.info.update(result)
        zone = result["zone"]
        if zone in ("RED", "YELLOW"):
            res.ok = True
            if zone == "YELLOW":
                self.log.warning("노란선 정지 (감점). 오차 %.1fcm", result["error_cm"])
        else:
            self.fail("정지 실패: %s (최종 %.1fcm, 오차 %.1fcm)" % (zone, result["final_cm"], result["error_cm"]))


class PrecisionStopper:
    """APPROACH_CM 까지 펄스 없이 연속 주행(FAST → SLOW) → 정지 → 펄스로 앞/뒤 재배치(creep) → 판정.
    MISSION 2 의 핵심 로직 (단독 테스트: --only 2)"""

    def __init__(self, robot: Robot, log: logging.Logger):
        self.robot = robot
        self.log = log
        self.target = C.STOP_TARGET_CM
        self.gyro = None

    def run(self) -> Dict:
        robot = self.robot
        robot.steer.center()
        if C.USE_GYRO_HEADING_HOLD:
            self.gyro = robot.gyro
            if self.gyro:
                self.gyro.calibrate()
        stop_at = C.APPROACH_CM + C.APPROACH_LEAD_CM      # 밀리는 거리만큼 일찍 정지 명령
        try:
            d_fast = self._drive_until(stop_at + C.SLOW_ZONE_CM, C.DRIVE_SPEED_FAST, "FAST")
            d_slow = self._drive_until(stop_at, C.DRIVE_SPEED_SLOW, "SLOW")
        finally:
            robot.drive.stop()
            robot.steer.center()
        robot.sleep(C.STOP_SETTLE_S)
        d = robot.ultra.read()
        approach = self._report_approach(d_slow if d_slow is not None else d_fast, d)
        d, pulses = self._creep(d)
        out = self.classify(d, pulses)
        out.update(approach)
        return out

    def _report_approach(self, d_cmd: Optional[float], d: Optional[float]) -> Dict:
        """연속 주행 결과. 밀림 = 정지 명령 때 거리 - 멈춘 뒤 거리 → APPROACH_LEAD_CM 권장값"""
        if d_cmd is None or d is None:
            return {}
        slide = d_cmd - d
        self.log.info("[APPROACH] 정지 명령 %.1fcm → 멈춘 뒤 %.1fcm (밀림 %.1fcm). %.0fcm 에 맞추려면 APPROACH_LEAD_CM = %.1f",
                      d_cmd, d, slide, C.APPROACH_CM, max(slide, 0.0))
        return {"approach_cm": round(d, 1), "slide_cm": round(slide, 1)}

    # ---- 연속 주행 ----
    def _drive_until(self, threshold: float, speed: int, label: str) -> Optional[float]:
        """threshold 를 읽을 때까지 speed 로 연속 주행 (정지는 호출한 쪽에서). 도달한 거리, 이미 안쪽이면 None"""
        robot = self.robot
        d = robot.ultra.read_raw()
        if d is not None and d <= threshold:
            self.log.info("[%s] 생략 (%.1fcm ≤ %.1f)", label, d, threshold)
            return None
        self.log.info("[%s] 속도 %d, %.1fcm 까지 주행", label, speed, threshold)
        if self.gyro:
            self.gyro.reset()
        t0 = robot.now()
        last_log = t0
        robot.drive.forward(speed)
        invalid = 0
        while True:
            d = robot.ultra.read_raw()
            if d is None:
                invalid += 1
                if invalid >= 20:
                    self.fail_stop("초음파 값이 계속 무효입니다")
            else:
                invalid = 0
                if d <= threshold:
                    self.log.info("[%s] 도달 %.1fcm (%.1fs)", label, d, robot.now() - t0)
                    return d
            if self.gyro:
                yaw = self.gyro.update()
                steer = max(-C.GYRO_MAX_STEER, min(C.GYRO_MAX_STEER, -C.GYRO_KP * yaw))
                robot.steer.set(steer)
            now = robot.now()
            if now - last_log >= 0.5:
                self.log.debug("[%s] d=%s yaw=%s", label, "%.1f" % d if d is not None else "-",
                               "%.1f" % self.gyro.yaw if self.gyro else "-")
                last_log = now
            if now - t0 > C.DRIVE_TIMEOUT_S:
                self.fail_stop("[%s] 주행 시간 초과 (%.0fs)" % (label, C.DRIVE_TIMEOUT_S))
            robot.sleep(C.DRIVE_LOOP_S)

    def fail_stop(self, msg):
        self.robot.drive.stop()
        raise MissionError(msg)

    # ---- 펄스 재배치 ----
    def _creep(self, d_first: Optional[float] = None):
        """측정 → 펄스(앞/뒤) → 대기 반복. d_first 가 있으면 첫 측정으로 사용.
        - 목표를 지나쳐 방향이 바뀌면 펄스를 줄여(CREEP_SHRINK) 진동 없이 수렴
        - 2회 연속 거의 안 움직이면 펄스를 늘림 (모터 데드존 대응)"""
        robot = self.robot
        pulse_s = C.CREEP_PULSE_S
        prev, prev_sign, stall = None, None, 0
        for i in range(1, C.MAX_CREEP_PULSES + 1):
            d = d_first if i == 1 and d_first is not None else robot.ultra.read()
            if d is None:
                self.log.warning("[CREEP] 측정 실패, 재시도")
                robot.sleep(C.CREEP_SETTLE_S)
                continue
            err = d - self.target
            moved = (prev - d) if prev is not None else None
            self.log.info("[CREEP %2d] d=%.1fcm 오차=%+.1fcm%s  pulse=%.2fs", i, d, err,
                          "  (이동 %+.1fcm)" % moved if moved is not None else "", pulse_s)
            if abs(err) <= C.STOP_TOLERANCE_CM:
                self.log.info("[CREEP] 목표 도달")
                return d, i - 1

            sign = 1 if err > 0 else -1
            if prev_sign is not None and sign != prev_sign:
                pulse_s = max(pulse_s * C.CREEP_SHRINK, C.CREEP_PULSE_MIN_S)
                self.log.info("[CREEP] 목표 지나침 → 펄스 %.2fs 로 감소", pulse_s)
                stall = 0
            elif moved is not None and abs(moved) < C.CREEP_STALL_CM:
                stall += 1
                if stall >= 2:
                    pulse_s = min(pulse_s * 1.5, C.CREEP_PULSE_MAX_S)
                    stall = 0
                    self.log.warning("[CREEP] 2회 연속 안 움직임 → 펄스 %.2fs 로 증가 (CREEP_SPEED/CREEP_PULSE_S 확인)", pulse_s)
            else:
                stall = 0

            if err > 0:
                dur = pulse_s * (C.CREEP_FAR_MULTIPLIER if err > 3.0 else 1.0)
                robot.drive.pulse(+1, C.CREEP_SPEED, dur)
            else:
                if not C.ALLOW_REVERSE_FIX:
                    self.log.warning("[CREEP] 목표 지나침, 후진 보정 비활성 (ALLOW_REVERSE_FIX)")
                    return d, i - 1
                if -err > C.REVERSE_FIX_LIMIT_CM:
                    self.log.error("[CREEP] %.1fcm 넘어감 (실격 영역) → 보정 포기", -err)
                    return d, i - 1
                robot.drive.pulse(-1, C.CREEP_SPEED, pulse_s)
            prev, prev_sign = d, sign
            robot.sleep(C.CREEP_SETTLE_S)
        self.log.warning("[CREEP] 최대 펄스 횟수(%d) 도달", C.MAX_CREEP_PULSES)
        return robot.ultra.read(), C.MAX_CREEP_PULSES

    # ---- 판정 ----
    def classify(self, d: Optional[float], pulses: int) -> Dict:
        if d is None:
            return {"zone": "UNKNOWN", "final_cm": None, "error_cm": None, "pulses": pulses}
        err = d - self.target
        if abs(err) <= C.RED_HALF_WIDTH_CM:
            zone = "RED"
        elif abs(err) <= C.YELLOW_HALF_WIDTH_CM:
            zone = "YELLOW"
        elif err < 0:
            zone = "OVER"      # 정지선 넘어감 (실격)
        else:
            zone = "SHORT"     # 너무 멀리서 멈춤
        self.log.info("최종 정지 %.1fcm (오차 %+.1fcm) → %s, 펄스 %d회", d, err, zone, pulses)
        return {"zone": zone, "final_cm": round(d, 1), "error_cm": round(err, 1), "pulses": pulses}


# =====================================================================
# MISSION 3 / 4 공용: 집어서 옮기기
# =====================================================================
class PickPlaceTask:
    """config.PICK_PLACE_SEQUENCE 를 해석해서 실행. (pick_side 의 물체 → place_side 상자)"""

    def __init__(self, robot: Robot, ctx: MissionContext, log: logging.Logger):
        self.robot = robot
        self.ctx = ctx
        self.log = log
        self.facing = "CENTER"      # 팔이 현재 향한 쪽 (LEFT / RIGHT / CENTER)

    def transfer(self, pick_side: str, place_side: str, info: Dict):
        self.log.info("물체 이동: %s 상자 → %s 상자", pick_side, place_side)
        sides = {"PICK": pick_side, "PLACE": place_side, "CENTER": "CENTER"}
        info["qr_seen"] = {}
        total = len(C.PICK_PLACE_SEQUENCE)
        for idx, step in enumerate(C.PICK_PLACE_SEQUENCE, 1):
            kind, arg = step[0], step[1]
            label = "%d/%d %s %s" % (idx, total, kind, arg)
            if self.ctx.pause_steps:
                ans = self._pause(label)
                if ans == "skip":
                    self.log.warning("step %s 건너뜀", label)
                    continue
            self.log.info("step %s", label)
            self._do_step(kind, arg, sides, info)
        self.log.info("물체 이동 완료 (%s → %s)", pick_side, place_side)

    def _do_step(self, kind, arg, sides, info):
        arm = self.robot.arm
        if kind == "pose":
            # 포즈 이름에 pick/place 가 들어 있으면 그 쪽의 쪽별 덮어쓰기, 아니면 지금 향한 쪽
            if "pick" in arg:
                side = sides["PICK"]
            elif "place" in arg:
                side = sides["PLACE"]
            else:
                side = self.facing
            arm.pose(arg, side=None if side == "CENTER" else side)
        elif kind == "base":
            arm.base_to(sides[arg])
            self.facing = sides[arg]
        elif kind == "grip":
            if arg == "open":
                arm.grip_open()
            elif arg == "close":
                arm.grip_close()
            else:
                arm.grip(float(arg))
        elif kind == "qr":
            seen = self._box_qr_check(expect=arg)
            info["qr_seen"]["%s@%s" % (arg, sides[arg])] = seen
        elif kind == "wait":
            self.robot.sleep(float(arg))
        else:
            raise MissionError("PICK_PLACE_SEQUENCE 에 알 수 없는 단계: %r" % ((kind, arg),))

    @staticmethod
    def _pause(label) -> str:
        try:
            ans = input("\n  ▶ 다음 단계 [%s]  Enter=실행 / s=건너뜀 / q=중단: " % label).strip().lower()
        except EOFError:
            return "run"
        if ans == "q":
            raise KeyboardInterrupt
        return "skip" if ans == "s" else "run"

    def _box_qr_check(self, expect: str) -> Optional[List[str]]:
        mode = C.QR_BOX_CHECK_MODE
        if mode == "off":
            return None
        robot = self.robot
        t0 = robot.now()
        seen: List[str] = []
        while robot.now() - t0 < C.QR_BOX_CHECK_TIMEOUT_S:
            texts = robot.read_qr()
            for t in texts:
                if t not in seen:
                    seen.append(t)
            if expect in texts:
                break
            robot.sleep(0.05)
        ok = expect in seen
        self.log.info("상자 QR 확인 (%s 기대): %s%s", expect, seen or "없음", "" if ok else "  ← 기대값 안 보임")
        if mode == "require" and not ok:
            raise MissionError("상자 QR '%s' 가 보이지 않습니다 (본 것: %s)" % (expect, seen))
        return seen


def _run_pick_place(mission: Mission, res: MissionResult, pick_side: str, place_side: str):
    res.info.update(pick=pick_side, place=place_side)
    PickPlaceTask(mission.robot, mission.ctx, mission.log).transfer(pick_side, place_side, res.info)


# =====================================================================
# MISSION 3. 첫 번째 물체 이동
# =====================================================================
class Mission3FirstTransfer(Mission):
    number = 3
    name = "첫 번째 물체 이동"

    def execute(self, res):
        d = self.ctx.require_direction()
        _run_pick_place(self, res, pick_side=d, place_side=C.opposite(d))


# =====================================================================
# MISSION 4. 두 번째 물체 이동
# =====================================================================
class Mission4SecondTransfer(Mission):
    number = 4
    name = "두 번째 물체 이동"

    def execute(self, res):
        d = self.ctx.require_direction()
        _run_pick_place(self, res, pick_side=C.opposite(d), place_side=d)


# =====================================================================
# MISSION 5. 미션 완료 및 기록 종료
# =====================================================================
class Mission5Finish(Mission):
    number = 5
    name = "미션 완료 및 기록 종료"

    def execute(self, res):
        robot, ctx = self.robot, self.ctx
        robot.stop_all()
        if C.FINISH_POSE:
            robot.arm.pose(C.FINISH_POSE)
        robot.stop_all()
        total = time.time() - ctx.t_start
        res.info["total_s"] = round(total, 1)
        self.log.info("모든 로봇 동작 완료. 총 %.1fs ▶ 조교 확인 후 종료 버튼을 누르세요.", total)


# =====================================================================
# 러너
# =====================================================================
ALL_MISSIONS = [Mission1QRStart, Mission2DriveAndStop, Mission3FirstTransfer, Mission4SecondTransfer, Mission5Finish]


class MissionRunner:
    def __init__(self, robot: Robot, ctx: MissionContext, numbers: List[int] = None, continue_on_error: bool = False):
        self.robot = robot
        self.ctx = ctx
        self.numbers = sorted(numbers or [m.number for m in ALL_MISSIONS])
        self.continue_on_error = continue_on_error
        self.log = logging.getLogger("runner")

    def run(self) -> bool:
        by_num = {m.number: m for m in ALL_MISSIONS}
        all_ok = True
        self.log.info("실행할 미션: %s  (방향=%s)", self.numbers, self.ctx.direction or "QR로 결정")
        try:
            for n in self.numbers:
                res = by_num[n](self.robot, self.ctx).run()
                self.ctx.results[n] = res
                if not res.ok:
                    all_ok = False
                    if not self.continue_on_error:
                        self.log.error("MISSION %d 실패로 중단합니다 (계속하려면 --continue-on-error)", n)
                        break
        finally:
            self.robot.stop_all()
        self.summary()
        self.save_log()
        return all_ok

    def summary(self):
        lines = ["", "┌──────── 결과 요약 ────────"]
        for n, r in sorted(self.ctx.results.items()):
            lines.append("│ M%d %-14s %s %6.1fs  %s" % (n, r.name[:14], "OK  " if r.ok else "FAIL", r.duration,
                                                       r.error or _short(r.info)))
        lines.append("│ 방향: %s   총 %.1fs" % (self.ctx.direction, time.time() - self.ctx.t_start))
        lines.append("└───────────────────────────")
        self.log.info("\n".join(lines))

    def save_log(self):
        try:
            os.makedirs(C.LOG_DIR, exist_ok=True)
            path = os.path.join(C.LOG_DIR, "result_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
            data = {"direction": self.ctx.direction,
                    "missions": [r.to_dict() for _, r in sorted(self.ctx.results.items())]}
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2, default=str)
            self.log.info("결과 저장: %s", path)
        except Exception as e:
            self.log.warning("결과 저장 실패: %s", e)


def _short(info: Dict) -> str:
    keep = {k: v for k, v in info.items() if k in ("direction", "zone", "final_cm", "error_cm", "pulses", "pick", "place", "total_s")}
    return " ".join("%s=%s" % kv for kv in keep.items())
