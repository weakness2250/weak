"""Параметры полёта и сброса — подстройте под площадку и схему подключения сервоприводов."""

from pathlib import Path

# Сеть по умолчанию: Wi‑Fi точка доступа Pioneer.
# Для «Геоскан Симулятор» на том же компьютере зачастую указывают 127.0.0.1 —
# точный IP/порт смотрите в руководстве по эксплуатации симулятора (MAVLink / внешнее управление).
DRONE_IP = "192.168.4.1"
MAVLINK_PORT = 8001
CONNECTION = "udpout"  # как в документации Pioneer SDK

# Патруль: прямоугольник в локальной СК (метры, ENU: x‑восток, y‑север, z‑высота), yaw в радианах
PATROL_HEIGHT = 1.5
PATROL_CORNERS_METERS = [
    (0.0, 0.0, PATROL_HEIGHT, 0.0),
    (5.0, 0.0, PATROL_HEIGHT, 0.0),
    (5.0, 4.0, PATROL_HEIGHT, 0.0),
    (0.0, 4.0, PATROL_HEIGHT, 0.0),
]
# 0 — патрулировать без ограничения числа циклов (останов скрипта — Ctrl+C)
PATROL_LOOPS = 0
POINT_REACHED_POLL_S = 0.1

# Точка возврата после реагирования на огонь (локальная СК ENU, как у PATROL_CORNERS_METERS)
HOME_XYZ_YAW_METERS = (0.0, 0.0, PATROL_HEIGHT, 0.0)

# YOLO из C:\Users\ПП\image_validator (fire_detector.py)
IMAGE_VALIDATOR_DIR = Path(r"C:\Users\ПП\image_validator")
FIRE_MODEL_PATH = IMAGE_VALIDATOR_DIR / "fire_models" / "train" / "weights" / "best.pt"
FIRE_CONFIDENCE = 0.25

CAMERA_PORT = 8888
CAMERA_LOG_CONNECTION = False
RPI_START_CAPTURE_ON_MISSION = True
RPI_CAPTURE_INTERVAL_S = 0.15

VISION_CYCLE_S = 0.2
FIRE_COOLDOWN_S = 12.0

APPROACH_STEP_FORWARD_M = 0.35
APPROACH_STEP_SIDE_M = 0.25
APPROACH_MAX_STEPS = 28
APPROACH_MIN_BOX_AREA_FRAC = 0.12

# Сброс: свободный RC‑канал/каналы, см. назначение AUX в документации и настройку приёмника.
# Значения типичные для PWM микросекунд / RC (уточните по конфигурации полётника).
#
# Можно задать несколько каналов (несколько сервоприводов): например [7, 8].
# Для обратной совместимости можно оставить один канал в DROP_RC_CHANNEL.
DROP_RC_CHANNELS = [7]  # channel_7 ... channel_8 в send_rc_channels
DROP_RC_CHANNEL = DROP_RC_CHANNELS[0]
DROP_PWM_CLOSED = 1000
DROP_PWM_OPEN = 2000
DROP_HOLD_OPEN_S = 0.6

# Демонстрация без сервоприводов:
# Если False — команда «сброс»/drop не шлёт RC, а пишет лог "ищет сервопривод → не найден"
# и продолжает работу.
DROP_SERVOS_PRESENT = True

# Не переопределяем остальные каналы (MAVLink: UINT16_MAX = без изменения)
RC_CH_IGNORE = 65535
