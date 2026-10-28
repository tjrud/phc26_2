#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_mission.py - 미션 실행 / 디버그 진입점
==========================================
실제 로봇 (Adeept_PiCar-Pro/Server/mission/ 안에서):
  sudo systemctl stop Adeept_Robot.service      # 웹서버가 카메라/GPIO 를 잡고 있으므로 먼저 정지
  python3 run_mission.py                        # MISSION 1~5 전체
  python3 run_mission.py --only 2               # MISSION 2 만 (정밀 정지 튜닝)
  python3 run_mission.py --from 3 --direction LEFT   # 정지선에 세워 놓고 3,4,5 만 (QR/주행 생략)
  python3 run_mission.py --only 3 --direction RIGHT --pause-steps   # 팔 동작을 한 단계씩 Enter 로 진행

하드웨어 없이 흐름만 확인 (PC/노트북):
  python3 run_mission.py --sim                  # 가짜 하드웨어로 1~5 전체
  python3 run_mission.py --sim --sim-direction RIGHT --only 3

점검 도구:
  --calibrate      키보드로 서보를 움직이며 포즈 각도 찾기 (p 로 config 에 붙여넣을 값 출력)
  --pose NAME      ARM_POSES 의 포즈로 이동해서 확인      (--side LEFT|RIGHT)
  --home           모든 서보 홈(90도)
  --test-ultra     초음파 거리 연속 출력
  --test-qr        카메라 QR 연속 인식 (logs/qr_last.jpg 저장)
  --test-gyro      자이로 yaw 출력 (축/부호 확인)
  --test-drive S T 속도 S 로 T 초 전진 후 이동 거리/속도(cm/s) 출력
  --test-pulse N   creep 펄스 N 번 → 펄스당 이동 거리 확인
  --measure-stopline  센서를 빨간 정지선 위에 놓고 실행 → ULTRA_OFFSET_CM 권장값 (가장 중요한 보정!)
"""
import argparse
import logging
import os
import subprocess
import sys
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as C                                   # noqa: E402
from hardware import make_robot, HardwareError       # noqa: E402
from missions import MissionContext, MissionRunner, ALL_MISSIONS   # noqa: E402

log = logging.getLogger("run")


# =====================================================================
# 로깅
# =====================================================================
def setup_logging(level: str, to_file: bool = True):
    fmt = logging.Formatter("%(asctime)s.%(msecs)03d [%(name)-6s] %(levelname).1s %(message)s", "%H:%M:%S")
    root = logging.getLogger()
    root.setLevel(getattr(logging, level.upper(), logging.INFO))
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt)
    root.addHandler(ch)
    if to_file:
        try:
            os.makedirs(C.LOG_DIR, exist_ok=True)
            path = os.path.join(C.LOG_DIR, "run_%s.log" % time.strftime("%Y%m%d_%H%M%S"))
            fh = logging.FileHandler(path, encoding="utf-8")
            fh.setFormatter(fmt)
            fh.setLevel(logging.DEBUG)
            root.addHandler(fh)
            log.info("로그 파일: %s", path)
        except Exception as e:
            log.warning("로그 파일 생성 실패: %s", e)


# =====================================================================
# Adeept 웹서버 서비스 확인
# =====================================================================
def service_active(name: str) -> bool:
    try:
        r = subprocess.run(["systemctl", "is-active", name], capture_output=True, text=True, timeout=5)
        return r.stdout.strip() == "active"
    except Exception:
        return False


def check_service(args) -> bool:
    if args.sim or args.no_service_check:
        return True
    if not service_active(C.ROBOT_SERVICE_NAME):
        return True
    if args.stop_service:
        log.info("%s 정지 중...", C.ROBOT_SERVICE_NAME)
        r = subprocess.run(["sudo", "systemctl", "stop", C.ROBOT_SERVICE_NAME])
        time.sleep(1.5)
        if r.returncode == 0 and not service_active(C.ROBOT_SERVICE_NAME):
            return True
        log.error("서비스 정지 실패")
        return False
    log.error("%s 가 실행 중입니다 (카메라/GPIO 점유). 먼저 정지하세요:\n"
              "    sudo systemctl stop %s\n"
              "  또는  --stop-service  옵션으로 자동 정지, 끝나고 다시 켜려면  sudo systemctl start %s",
              C.ROBOT_SERVICE_NAME, C.ROBOT_SERVICE_NAME, C.ROBOT_SERVICE_NAME)
    return False


# =====================================================================
# 점검 도구
# =====================================================================
def _getch():
    """터미널에서 키 하나 읽기 (Ctrl+C → 'x')"""
    try:
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return "x" if ch in ("\x03", "\x04") else ch
    except Exception:
        return (input("> ").strip() + " ")[0]


CALIB_HELP = """
┌── 서보 캘리브레이션 ──────────────────────────────────────────────────
│  a / d : base 왼쪽 / 오른쪽 (ch1)      w / s : shoulder 올림 / 내림 (ch2)
│  i / k : wrist 올림 / 내림 (ch3)       [ / ] : grip 닫힘(-) / 열림(+) (ch4)
│  o / c : grip 을 GRIP_OPEN / GRIP_CLOSED 로     , / . : 조향 좌 / 우 (ch0)
│  + / - : 스텝 크기 변경 (1,2,5,10)      h : 홈(90)    g : config 의 포즈로 이동 (이름 입력)
│  p : 현재 각도를 config 에 붙여넣을 형식으로 출력   u : 초음파 거리   r : QR 한 번 읽기
│  x : 종료
└──────────────────────────────────────────────────────────────────────
"""


def tool_calibrate(robot):
    arm, servos = robot.arm, robot.servos
    arm.init_home()
    robot.steer.center()
    steps = [1, 2, 5, 10]
    si = 2
    keymap = {
        "a": (C.CH_BASE, +1), "d": (C.CH_BASE, -1),
        "w": (C.CH_SHOULDER, -1), "s": (C.CH_SHOULDER, +1),
        "i": (C.CH_WRIST, +1), "k": (C.CH_WRIST, -1),
        "[": (C.CH_GRIP, -1), "]": (C.CH_GRIP, +1),
        ",": (C.CH_STEER, -1), ".": (C.CH_STEER, +1),
    }
    print(CALIB_HELP)
    _print_pose(servos)
    while True:
        ch = _getch()
        if ch == "x":
            break
        elif ch in keymap:
            c, sign = keymap[ch]
            cur = servos.get(c) if servos.get(c) is not None else 90
            servos.write(c, cur + sign * steps[si])
            _print_pose(servos)
        elif ch == "+":
            si = min(si + 1, len(steps) - 1)
            print("step = %d" % steps[si])
        elif ch == "-":
            si = max(si - 1, 0)
            print("step = %d" % steps[si])
        elif ch == "o":
            arm.grip_open()
            _print_pose(servos)
        elif ch == "c":
            arm.grip_close()
            _print_pose(servos)
        elif ch == "h":
            arm.init_home()
            _print_pose(servos)
        elif ch == "g":
            name = input("\n포즈 이름 (%s): " % ", ".join(C.ARM_POSES)).strip()
            side = input("side (LEFT/RIGHT/엔터=없음): ").strip().upper() or None
            try:
                arm.pose(name, side=side)
            except KeyError as e:
                print(e)
            _print_pose(servos)
        elif ch == "p":
            _print_pose(servos, as_config=True)
        elif ch == "u":
            print("ultra: raw=%s  median=%s" % (robot.ultra.read_raw(), robot.ultra.read()))
        elif ch == "r":
            path = os.path.join(C.LOG_DIR, "qr_last.jpg")
            print("QR: %s  (프레임 저장: %s)" % (robot.read_qr(save_path=path), path))
        elif ch == "?":
            print(CALIB_HELP)
    print("종료 (서보는 현재 위치 유지)")


def _print_pose(servos, as_config=False):
    chs = [C.CH_STEER, C.CH_BASE, C.CH_SHOULDER, C.CH_WRIST, C.CH_GRIP]
    vals = {c: servos.get(c) for c in chs}
    if as_config:
        arm = ", ".join("CH_%s: %d" % (C.CH_NAMES[c].upper(), round(vals[c] or 90))
                        for c in (C.CH_BASE, C.CH_SHOULDER, C.CH_WRIST, C.CH_GRIP))
        print("\n  config.py 에 붙여넣기:   {%s}" % arm)
        print("  base 만: BASE_xxx = %d   grip 만: GRIP_xxx = %d   steer trim: %d\n"
              % (round(vals[C.CH_BASE] or 90), round(vals[C.CH_GRIP] or 90), round((vals[C.CH_STEER] or 90) - 90)))
    else:
        print("\r  " + "  ".join("%s=%s" % (C.CH_NAMES[c], "?" if vals[c] is None else "%.0f" % vals[c]) for c in chs)
              + "      ", end="", flush=True)


def tool_pose(robot, name, side):
    try:
        C.get_pose(name, side)
    except KeyError as e:
        print(e.args[0])
        return
    robot.arm.init_home()
    robot.arm.pose(name, side=side)
    print("포즈 '%s'%s 이동 완료: %s" % (name, " (%s)" % side if side else "", robot.arm.current_pose()))
    input("확인 후 Enter (서보는 그대로 유지) ")


def tool_test_ultra(robot):
    print("초음파 거리 (Ctrl+C 종료)  ULTRA_OFFSET_CM=%.1f" % C.ULTRA_OFFSET_CM)
    while True:
        raw = robot.ultra.read_raw()
        med = robot.ultra.read()
        print("  raw=%6s cm   median=%6s cm" % ("%.1f" % raw if raw is not None else "-", "%.1f" % med if med is not None else "-"))
        time.sleep(0.2)


def tool_test_qr(robot, no_arm):
    if not no_arm:
        robot.arm.init_home()
        robot.arm.pose("scan_qr")
    path = os.path.join(C.LOG_DIR, "qr_last.jpg")
    print("QR 연속 인식 (Ctrl+C 종료). 프레임은 %s 에 저장" % path)
    n = 0
    t0 = time.time()
    while True:
        n += 1
        texts = robot.read_qr(save_path=path if n % 10 == 0 else None)
        fps = n / max(1e-6, time.time() - t0)
        print("  [%4d] %.1f fps  %s" % (n, fps, texts or "-"))
        time.sleep(0.05)


def tool_test_gyro(robot):
    g = robot.gyro
    if g is None:
        print("자이로를 사용할 수 없습니다 (mpu6050 라이브러리/배선 확인)")
        return
    print("정지 상태로 두세요... 바이어스 측정")
    g.calibrate()
    print("로봇을 손으로 왼쪽(반시계)으로 돌려 보세요 → yaw 가 + 로 커져야 정상 (아니면 GYRO_YAW_SIGN 또는 GYRO_YAW_AXIS 변경). Ctrl+C 종료")
    while True:
        yaw = g.update()
        print("  yaw=%+7.1f°  rate=%+6.1f°/s" % (yaw, g.rate()))
        time.sleep(0.1)


def tool_test_drive(robot, speed, seconds):
    d0 = robot.ultra.read()
    print("전진 전 거리: %s cm" % d0)
    robot.steer.center()
    robot.drive.pulse(+1, speed, seconds)
    time.sleep(C.STOP_SETTLE_S)
    d1 = robot.ultra.read()
    print("전진 후 거리: %s cm" % d1)
    if d0 is not None and d1 is not None:
        print("이동 %.1f cm / %.2f s = %.1f cm/s  (속도 %d)" % (d0 - d1, seconds, (d0 - d1) / seconds, speed))


def tool_measure_stopline(robot, n=20):
    """로봇(초음파 센서)을 빨간 정지선 위에 놓고 실행 → ULTRA_OFFSET_CM 권장값 출력"""
    print("초음파 센서를 빨간 정지선 위에 정확히 놓고 기다리세요... (%d회 측정)" % n)
    vals = []
    for i in range(n):
        d = robot.ultra.read_raw()
        if d is not None:
            vals.append(d)
        time.sleep(0.1)
    if not vals:
        print("측정 실패")
        return
    import statistics
    mean, sd = statistics.mean(vals), (statistics.pstdev(vals) if len(vals) > 1 else 0.0)
    raw_mean = mean - C.ULTRA_OFFSET_CM
    print("측정값(보정 포함) 평균 %.2f cm, 표준편차 %.2f, 샘플 %d개 (현재 ULTRA_OFFSET_CM=%.2f)" % (mean, sd, len(vals), C.ULTRA_OFFSET_CM))
    print("▶ 정지선이 목표 %.1fcm 이므로  config.py 에  ULTRA_OFFSET_CM = %.2f  로 설정하세요"
          % (C.STOP_TARGET_CM, C.STOP_TARGET_CM - raw_mean))
    if sd > 0.6:
        print("  (편차가 큽니다: 모니터/노트북 면이 센서 높이에서 평평하게 보이는지 확인)")


def tool_test_pulse(robot, n):
    print("creep 펄스 테스트: speed=%d pulse=%.2fs" % (C.CREEP_SPEED, C.CREEP_PULSE_S))
    robot.steer.center()
    prev = robot.ultra.read()
    for i in range(1, n + 1):
        robot.drive.pulse(+1, C.CREEP_SPEED, C.CREEP_PULSE_S)
        time.sleep(C.CREEP_SETTLE_S)
        d = robot.ultra.read()
        print("  [%d] %s → %s cm   이동 %s cm" % (i, prev, d, "%.2f" % (prev - d) if prev is not None and d is not None else "-"))
        prev = d


# =====================================================================
# main
# =====================================================================
def parse_args(argv=None):
    p = argparse.ArgumentParser(description="PiCar-Pro 미션 실행기", formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__)
    sel = p.add_argument_group("미션 선택")
    sel.add_argument("--only", help="실행할 미션 번호 (예: 2 또는 3,4)")
    sel.add_argument("--from", dest="from_", type=int, help="이 번호부터 5까지")
    sel.add_argument("--to", type=int, help="이 번호까지")
    sel.add_argument("--direction", choices=["LEFT", "RIGHT"], help="QR 대신 방향을 미리 지정 (MISSION 1 생략)")
    sel.add_argument("--continue-on-error", action="store_true", help="미션이 실패해도 다음 미션 계속")
    sel.add_argument("--pause-steps", action="store_true", help="팔 동작(3,4)을 한 단계씩 Enter 로 진행")
    sel.add_argument("--no-init", action="store_true", help="시작 시 서보 홈 이동 생략 (팔이 이미 홈에 있다고 가정)")

    sim = p.add_argument_group("시뮬레이션")
    sim.add_argument("--sim", action="store_true", help="하드웨어 없이 가짜 로봇으로 실행")
    sim.add_argument("--sim-direction", choices=["LEFT", "RIGHT"], default="LEFT", help="시뮬레이션 QR 방향")
    sim.add_argument("--sim-speedup", type=float, default=None, help="시뮬레이션 배속 (기본 config.SIM_SPEEDUP)")

    tool = p.add_argument_group("점검 도구")
    tool.add_argument("--calibrate", action="store_true")
    tool.add_argument("--pose", metavar="NAME")
    tool.add_argument("--side", choices=["LEFT", "RIGHT"])
    tool.add_argument("--home", action="store_true")
    tool.add_argument("--stop", action="store_true", help="모터 정지만 하고 종료")
    tool.add_argument("--test-ultra", action="store_true")
    tool.add_argument("--test-qr", action="store_true")
    tool.add_argument("--no-arm", action="store_true", help="--test-qr 에서 팔을 움직이지 않음")
    tool.add_argument("--test-gyro", action="store_true")
    tool.add_argument("--test-drive", nargs=2, type=float, metavar=("SPEED", "SECONDS"))
    tool.add_argument("--test-pulse", type=int, metavar="N")
    tool.add_argument("--measure-stopline", action="store_true", help="센서를 빨간 정지선에 놓고 ULTRA_OFFSET_CM 권장값 계산")

    misc = p.add_argument_group("기타")
    misc.add_argument("--stop-service", action="store_true", help="실행 중인 Adeept_Robot.service 를 자동 정지")
    misc.add_argument("--no-service-check", action="store_true")
    misc.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING"])
    misc.add_argument("--no-log-file", action="store_true")
    return p.parse_args(argv)


def select_missions(args):
    nums = [m.number for m in ALL_MISSIONS]
    if args.only:
        nums = [int(x) for x in args.only.replace(" ", "").split(",") if x]
    else:
        lo = args.from_ or 1
        hi = args.to or 5
        nums = [n for n in nums if lo <= n <= hi]
    bad = [n for n in nums if n not in (1, 2, 3, 4, 5)]
    if bad:
        raise SystemExit("잘못된 미션 번호: %s" % bad)
    return nums


def main(argv=None) -> int:
    args = parse_args(argv)
    setup_logging(args.log_level, to_file=not args.no_log_file)
    if not check_service(args):
        return 2

    robot = make_robot(sim=args.sim, sim_direction=args.sim_direction, sim_speedup=args.sim_speedup)
    code = 0
    try:
        # ---- 점검 도구 ----
        if args.stop:
            robot.drive.stop()
            print("모터 정지")
        elif args.home:
            robot.arm.init_home()
            robot.steer.center()
            print("홈 이동 완료")
        elif args.calibrate:
            tool_calibrate(robot)
        elif args.pose:
            tool_pose(robot, args.pose, args.side)
        elif args.test_ultra:
            tool_test_ultra(robot)
        elif args.test_qr:
            tool_test_qr(robot, args.no_arm)
        elif args.test_gyro:
            tool_test_gyro(robot)
        elif args.test_drive:
            tool_test_drive(robot, int(args.test_drive[0]), float(args.test_drive[1]))
        elif args.test_pulse:
            tool_test_pulse(robot, args.test_pulse)
        elif args.measure_stopline:
            tool_measure_stopline(robot)
        # ---- 미션 실행 ----
        else:
            nums = select_missions(args)
            ctx = MissionContext(direction=args.direction, pause_steps=args.pause_steps)
            if args.no_init:
                for ch, a in C.get_pose("home").items():
                    robot.servos.angles[ch] = a
            else:
                robot.arm.init_home()
            robot.steer.center()
            ok = MissionRunner(robot, ctx, nums, continue_on_error=args.continue_on_error).run()
            code = 0 if ok else 1
    except KeyboardInterrupt:
        robot.stop_all()
        log.warning("Ctrl+C → 모터 정지, 종료")
        code = 130
    except HardwareError as e:
        robot.stop_all()
        log.error("하드웨어 오류: %s", e)
        code = 3
    except Exception as e:
        robot.stop_all()
        log.error("오류: %s: %s", type(e).__name__, e)
        log.debug("traceback:\n%s", traceback.format_exc())
        code = 4
    finally:
        robot.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
