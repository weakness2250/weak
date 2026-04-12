"""
ПО для Geoscan Pioneer: патруль по области, импульс на RC для серво (сброс), ручное управление скоростью.

Пакет pioneer_sdk при обычном import выполняет __init__.py и тянет Camera/OpenCV.
Скрипт подменяет пакет в sys.modules, чтобы импортировать только piosdk (без cv2/numpy из камеры).

Запуск (после подключения к Wi‑Fi дрона):
  python pioneer_app.py auto    — взлёт, патруль + детекция огня (YOLO из image_validator), сброс, возврат «домой»
  python pioneer_app.py patrol
  python pioneer_app.py drop / сброс
  python pioneer_app.py manual

Симулятор (если в документации указан localhost, пример):
  python pioneer_app.py --ip 127.0.0.1 patrol
"""

from __future__ import annotations

import argparse
import site
import sys
import threading
import time
import types
from pathlib import Path
from typing import Callable


def _install_pioneer_sdk_shim() -> None:
    """Регистрирует pioneer_sdk без запуска __init__.py (иначе подгружается cv2 и ломается с NumPy 2 / Python 3.14)."""
    if "pioneer_sdk" in sys.modules:
        return
    roots = [Path(b) / "pioneer_sdk" for b in site.getsitepackages()]
    us = site.getusersitepackages()
    if us:
        roots.append(Path(us) / "pioneer_sdk")
    for root in roots:
        if (root / "piosdk.py").is_file():
            pkg = types.ModuleType("pioneer_sdk")
            pkg.__path__ = [str(root)]
            sys.modules["pioneer_sdk"] = pkg
            return


_install_pioneer_sdk_shim()

from pioneer_sdk.camera import Camera
from pioneer_sdk.piosdk import Pioneer

import config as cfg


def connect(*, ip: str | None = None, mavlink_port: int | None = None) -> Pioneer:
    return Pioneer(
        ip=cfg.DRONE_IP if ip is None else ip,
        mavlink_port=cfg.MAVLINK_PORT if mavlink_port is None else mavlink_port,
        connection_method=cfg.CONNECTION,
    )


def wait_connected(drone: Pioneer, timeout: float = 15.0) -> None:
    t0 = time.time()
    while not drone.connected():
        if time.time() - t0 > timeout:
            raise TimeoutError("Нет связи с БПЛА по MAVLink")
        time.sleep(0.1)


def patrol_territory(drone: Pioneer) -> None:
    """Лёт по замкнутому маршруту. PATROL_LOOPS == 0 — без ограничения циклов."""
    loop = 0
    while cfg.PATROL_LOOPS == 0 or loop < cfg.PATROL_LOOPS:
        for x, y, z, yaw in cfg.PATROL_CORNERS_METERS:
            ok = drone.go_to_local_point(x=x, y=y, z=z, yaw=yaw)
            if not ok:
                raise RuntimeError("Команда перехода к точке отклонена")
            while not drone.point_reached():
                time.sleep(cfg.POINT_REACHED_POLL_S)
        loop += 1
        if cfg.PATROL_LOOPS == 0:
            print(f"[patrol] цикл {loop} завершён (повтор)")
        else:
            print(f"[patrol] цикл {loop}/{cfg.PATROL_LOOPS} завершён")


def rc_channels_ignore_all() -> dict:
    return {f"channel_{i}": cfg.RC_CH_IGNORE for i in range(1, 9)}


def drop_payload(drone: Pioneer) -> None:
    """Импульс на выбранном RC‑канале (серво на AUX должен быть настроен на этот канал)."""
    if not getattr(cfg, "DROP_SERVOS_PRESENT", True):
        channels = getattr(cfg, "DROP_RC_CHANNELS", [cfg.DROP_RC_CHANNEL])
        print(f"[drop] поиск сервопривода на RC-каналах {channels}...")
        time.sleep(0.4)
        print("[drop] сервопривод не найден — продолжаю работу (режим демонстрации)")
        return

    kw = rc_channels_ignore_all()
    channels = getattr(cfg, "DROP_RC_CHANNELS", [cfg.DROP_RC_CHANNEL])
    for ch_n in channels:
        kw[f"channel_{ch_n}"] = cfg.DROP_PWM_OPEN
    drone.send_rc_channels(**kw)
    time.sleep(cfg.DROP_HOLD_OPEN_S)
    for ch_n in channels:
        kw[f"channel_{ch_n}"] = cfg.DROP_PWM_CLOSED
    drone.send_rc_channels(**kw)
    print(f"[drop] импульс на серво выполнен (каналы: {channels})")


def _load_fire_detector():
    root = str(cfg.IMAGE_VALIDATOR_DIR.resolve())
    if root not in sys.path:
        sys.path.insert(0, root)
    from fire_detector import detect_fire, load_yolo_model

    return load_yolo_model, detect_fire


def _bbox_area_frac(det: dict, frame_w: int, frame_h: int) -> float:
    x1, y1, x2, y2 = det["bbox"]
    return max(0.0, (x2 - x1) * (y2 - y1) / float(frame_w * frame_h))


def return_to_home(drone: Pioneer) -> None:
    x, y, z, yaw = cfg.HOME_XYZ_YAW_METERS
    print(f"[nav] возврат к точке старта ({x}, {y}, {z})")
    ok = drone.go_to_local_point(x=x, y=y, z=z, yaw=yaw)
    if not ok:
        raise RuntimeError("Команда возврата домой отклонена")
    while not drone.point_reached():
        time.sleep(cfg.POINT_REACHED_POLL_S)


def approach_fire_and_drop(
    drone: Pioneer,
    camera: Camera,
    model,
    detect_fire: Callable[..., tuple],
) -> None:
    """Сближение по bbox огня в кадре, затем сброс (как drop_payload)."""
    print("[fire] сближение с очагом")
    had_detection = False
    no_frame_streak = 0
    no_det_streak = 0

    for _ in range(cfg.APPROACH_MAX_STEPS):
        frame = camera.get_cv_frame()
        if frame is None:
            no_frame_streak += 1
            if no_frame_streak > 40:
                print("[fire] нет видеокадров — прерывание сближения")
                return
            time.sleep(0.05)
            continue

        no_frame_streak = 0
        _, dets = detect_fire(model, frame, cfg.FIRE_CONFIDENCE)
        if not dets:
            no_det_streak += 1
            if no_det_streak > 10 and had_detection:
                print("[fire] очаг временно не виден — выполняем сброс")
                break
            if no_det_streak > 25:
                print("[fire] очаг не найден — сброс отменён")
                return
            time.sleep(0.08)
            continue

        no_det_streak = 0
        had_detection = True
        best = max(
            dets,
            key=lambda d: (d["bbox"][2] - d["bbox"][0]) * (d["bbox"][3] - d["bbox"][1]),
        )
        fh, fw = frame.shape[:2]
        x1, y1, x2, y2 = best["bbox"]
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        frac = _bbox_area_frac(best, fw, fh)

        if frac >= cfg.APPROACH_MIN_BOX_AREA_FRAC:
            print(f"[fire] достаточная близость (бокс ~{frac:.0%} кадра)")
            break

        norm_ex = cx / fw - 0.5
        norm_ey = cy / fh - 0.5
        forward = cfg.APPROACH_STEP_FORWARD_M * (0.55 + min(1.0, abs(norm_ey) * 1.5))
        if abs(norm_ey) < 0.1:
            forward *= 1.15
        side = -cfg.APPROACH_STEP_SIDE_M * norm_ex * 2.8
        side = max(-1.2, min(1.2, side))

        ok = drone.go_to_local_point_body_fixed(x=forward, y=side, z=0.0, yaw=0.0)
        if not ok:
            raise RuntimeError("Команда сближения (body_fixed) отклонена")

        t0 = time.time()
        while not drone.point_reached() and time.time() - t0 < 30.0:
            time.sleep(cfg.POINT_REACHED_POLL_S)

    if had_detection:
        drop_payload(drone)
    else:
        print("[fire] сброс не выполнялся — очаг не был подтверждён в кадре")


def _wait_point_reached_or_fire(
    drone: Pioneer,
    stop: threading.Event,
    fire_alarm: threading.Event,
    intercept_active: threading.Event,
) -> str:
    while not drone.point_reached():
        if stop.is_set():
            return "stop"
        if fire_alarm.is_set() and not intercept_active.is_set():
            return "fire"
        time.sleep(cfg.POINT_REACHED_POLL_S)
    return "ok"


def run_patrol_with_fire_detection(
    drone: Pioneer,
    camera: Camera,
    model,
    detect_fire: Callable[..., tuple],
    stop: threading.Event,
    fire_alarm: threading.Event,
    intercept_active: threading.Event,
    last_drop_time: list[float],
) -> None:
    def handle_intercept() -> None:
        fire_alarm.clear()
        intercept_active.set()
        try:
            approach_fire_and_drop(drone, camera, model, detect_fire)
            last_drop_time[0] = time.time()
        finally:
            intercept_active.clear()
        return_to_home(drone)

    patrol_pass = 0
    while not stop.is_set():
        if cfg.PATROL_LOOPS > 0 and patrol_pass >= cfg.PATROL_LOOPS:
            break

        for x, y, z, yaw in cfg.PATROL_CORNERS_METERS:
            if stop.is_set():
                return

            if fire_alarm.is_set() and not intercept_active.is_set():
                handle_intercept()
                if stop.is_set():
                    return

            ok = drone.go_to_local_point(x=x, y=y, z=z, yaw=yaw)
            if not ok:
                raise RuntimeError("Команда перехода к точке отклонена")

            res = _wait_point_reached_or_fire(drone, stop, fire_alarm, intercept_active)
            if res == "stop":
                return
            if res == "fire":
                handle_intercept()

            if stop.is_set():
                return

        patrol_pass += 1
        print(f"[patrol] завершён проход {patrol_pass}")


def _vision_worker(
    camera: Camera,
    model,
    detect_fire: Callable[..., tuple],
    stop: threading.Event,
    intercept_active: threading.Event,
    fire_alarm: threading.Event,
    last_drop_time: list[float],
) -> None:
    while not stop.is_set():
        if intercept_active.is_set():
            time.sleep(0.05)
            continue
        if time.time() - last_drop_time[0] < cfg.FIRE_COOLDOWN_S:
            time.sleep(cfg.VISION_CYCLE_S)
            continue

        frame = camera.get_cv_frame()
        if frame is None:
            time.sleep(0.05)
            continue

        _, dets = detect_fire(model, frame, cfg.FIRE_CONFIDENCE)
        if dets:
            fire_alarm.set()
            print(
                f"[vision] очаг: {len(dets)} бокс., max conf {max(d['confidence'] for d in dets):.2f}"
            )

        time.sleep(cfg.VISION_CYCLE_S)


def manual_speed_loop(drone: Pioneer) -> None:
    """
    Ручное управление скоростью в связке с корпусом (м/с и рад/с).
    Клавиши (Windows): W/S вперёд‑назад, A/D влево‑вправо, R/F вверх‑вниз, Q/E рыскание.
    X — выход из режима.
    """
    if sys.platform != "win32":
        print("Интерактивный режим рассчитан на Windows (msvcrt). На Linux используйте свой ввод или пульт.")
        return

    import msvcrt

    print(
        "Ручной режим скорости (телом): W/S A/D R/F Q/E, X — выход. "
        "Убедитесь, что автопилот допускает MANUAL_SPEED (см. документацию и режим с пульта)."
    )
    vx = vy = vz = yaw_rate = 0.0
    step_lin = 0.3
    step_yaw = 0.4

    while True:
        if msvcrt.kbhit():
            k = msvcrt.getch()
            if k in (b"x", b"X"):
                break
            if k == b"w":
                vx = step_lin
            elif k == b"s":
                vx = -step_lin
            elif k == b"d":
                vy = step_lin
            elif k == b"a":
                vy = -step_lin
            elif k == b"r":
                vz = step_lin
            elif k == b"f":
                vz = -step_lin
            elif k == b"e":
                yaw_rate = step_yaw
            elif k == b"q":
                yaw_rate = -step_yaw

        drone.set_manual_speed_body_fixed(vx, vy, vz, yaw_rate)
        vx *= 0.85
        vy *= 0.85
        vz *= 0.85
        yaw_rate *= 0.85
        time.sleep(0.05)

    drone.set_manual_speed_body_fixed(0, 0, 0, 0)
    print("[manual] остановка скоростей")


def cmd_auto(**conn_kw) -> None:
    """Взлёт, патруль, параллельно YOLO по камере БПЛА; при огне — сближение, сброс, возврат домой."""
    if not cfg.FIRE_MODEL_PATH.is_file():
        raise FileNotFoundError(
            f"Нет весов YOLO: {cfg.FIRE_MODEL_PATH}. "
            "Обучите модель в image_validator или задайте FIRE_MODEL_PATH в config.py."
        )

    load_yolo_model, detect_fire = _load_fire_detector()
    model = load_yolo_model(str(cfg.FIRE_MODEL_PATH))

    drone_ip = conn_kw.get("ip") or cfg.DRONE_IP
    camera = Camera(
        ip=drone_ip,
        port=cfg.CAMERA_PORT,
        log_connection=cfg.CAMERA_LOG_CONNECTION,
    )

    stop = threading.Event()
    fire_alarm = threading.Event()
    intercept_active = threading.Event()
    last_drop_time: list[float] = [0.0]

    drone = connect(**conn_kw)
    wait_connected(drone)

    vision_thread = threading.Thread(
        target=_vision_worker,
        args=(camera, model, detect_fire, stop, intercept_active, fire_alarm, last_drop_time),
        daemon=True,
    )

    try:
        drone.arm()
        drone.takeoff()
        time.sleep(3)
        if cfg.RPI_START_CAPTURE_ON_MISSION:
            drone.raspberry_start_capture(
                interval=cfg.RPI_CAPTURE_INTERVAL_S,
                total_images=0,
            )
        vision_thread.start()
        run_patrol_with_fire_detection(
            drone,
            camera,
            model,
            detect_fire,
            stop,
            fire_alarm,
            intercept_active,
            last_drop_time,
        )
    finally:
        stop.set()
        vision_thread.join(timeout=3.0)
        if cfg.RPI_START_CAPTURE_ON_MISSION:
            try:
                drone.raspberry_stop_capture()
            except Exception:
                pass
        camera.disconnect()
        drone.land()
        time.sleep(2)
        drone.disarm()


def cmd_patrol(**conn_kw) -> None:
    drone = connect(**conn_kw)
    wait_connected(drone)
    try:
        drone.arm()
        drone.takeoff()
        time.sleep(3)
        patrol_territory(drone)
    finally:
        drone.land()
        time.sleep(2)
        drone.disarm()


def cmd_drop(**conn_kw) -> None:
    drone = connect(**conn_kw)
    wait_connected(drone)
    drop_payload(drone)


def cmd_manual(**conn_kw) -> None:
    drone = connect(**conn_kw)
    wait_connected(drone)
    manual_speed_loop(drone)


def main() -> None:
    p = argparse.ArgumentParser(description="Geoscan Pioneer: патруль / сброс / ручной режим")
    p.add_argument("command", choices=["auto", "patrol", "drop", "сброс", "manual"])
    p.add_argument(
        "--ip",
        default=None,
        help="Адрес MAVLink (для симулятора на этом ПК часто 127.0.0.1; см. руководство к симулятору)",
    )
    p.add_argument(
        "--port",
        type=int,
        default=None,
        help="Порт MAVLink (по умолчанию как в config.py, обычно 8001)",
    )
    args = p.parse_args()
    conn_kw = {k: v for k, v in (("ip", args.ip), ("mavlink_port", args.port)) if v is not None}
    if args.command == "auto":
        cmd_auto(**conn_kw)
    elif args.command == "patrol":
        cmd_patrol(**conn_kw)
    elif args.command in ("drop", "сброс"):
        cmd_drop(**conn_kw)
    elif args.command == "manual":
        cmd_manual(**conn_kw)


if __name__ == "__main__":
    main()
