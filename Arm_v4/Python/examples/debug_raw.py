# -*- coding: utf-8 -*-
"""
Диагностика: печатает СЫРЫЕ кватернионы груди/руки прямо с BLE, без
калибровки и без расчёта углов. Нужен, чтобы понять, обновляются ли
вообще данные с датчиков во времени, или замирают на одном значении.

Запуск (после pip install -e . из папки Python/):
    python examples/debug_raw.py

Подвигайте рукой и корпусом во время работы и посмотрите, меняются ли
числа. Ctrl+C для остановки.
"""

import time

from posetracker.ble_link import BleQuatLink

link = BleQuatLink()
link.start()
print("Подключение...")
link.wait_connected()
print("Подключено. Печатаю сырые кватернионы (w x y z) груди и руки.\n")

try:
    while True:
        q_chest, q_arm = link.get_quats()
        print(
            f"ГРУДЬ: w={q_chest[0]:+.4f} x={q_chest[1]:+.4f} "
            f"y={q_chest[2]:+.4f} z={q_chest[3]:+.4f}   |   "
            f"РУКА: w={q_arm[0]:+.4f} x={q_arm[1]:+.4f} "
            f"y={q_arm[2]:+.4f} z={q_arm[3]:+.4f}"
        )
        time.sleep(0.3)
except KeyboardInterrupt:
    print("\nОстановлено.")
finally:
    link.stop()
