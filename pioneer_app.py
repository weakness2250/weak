"""
ПО для Geoscan Pioneer: патруль по области, импульс на RC для серво (сброс), ручное управление скоростью.

Пакет pioneer_sdk при обычном import выполняет __init__.py и тянет Camera/OpenCV.
Скрипт подменяет пакет в sys.modules, чтобы импортировать только piosdk (без cv2/numpy из камеры).

Запуск (после подключения к Wi‑Fi дрона):
  python pioneer_app.py patrol
  python pioneer_app.py drop
  python pioneer_app.py manual

Симулятор (если в документации указан localhost, пример):
  python pioneer_app.py --ip 127.0.0.1 patrol
"""

from __future__ import annotations

import argparse
import site
import sys
import time
import types
from pathlib import Path


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
    """Лёт по замкнутому маршруту (малый периметр)."""
    for loop in range(cfg.PATROL_LOOPS):
        for x, y, z, yaw in cfg.PATROL_CORNERS_METERS:
            ok = drone.go_to_local_point(x=x, y=y, z=z, yaw=yaw)
            if not ok:
                raise RuntimeError("Команда перехода к точке отклонена")
            while not drone.point_reached():
                time.sleep(cfg.POINT_REACHED_POLL_S)
        print(f"[patrol] цикл {loop + 1}/{cfg.PATROL_LOOPS} завершён")


def rc_channels_ignore_all() -> dict:
    return {f"channel_{i}": cfg.RC_CH_IGNORE for i in range(1, 9)}


def drop_payload(drone: Pioneer) -> None:
    """Импульс на выбранном RC‑канале (серво на AUX должен быть настроен на этот канал)."""
    kw = rc_channels_ignore_all()
    ch = f"channel_{cfg.DROP_RC_CHANNEL}"
    kw[ch] = cfg.DROP_PWM_OPEN
    drone.send_rc_channels(**kw)
    time.sleep(cfg.DROP_HOLD_OPEN_S)
    kw[ch] = cfg.DROP_PWM_CLOSED
    drone.send_rc_channels(**kw)
    print("[drop] импульс на серво выполнен")


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
    p.add_argument("command", choices=["patrol", "drop", "manual"])
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
    if args.command == "patrol":
        cmd_patrol(**conn_kw)
    elif args.command == "drop":
        cmd_drop(**conn_kw)
    elif args.command == "manual":
        cmd_manual(**conn_kw)


if __name__ == "__main__":
    main()
