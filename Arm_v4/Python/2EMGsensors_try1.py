# -*- coding: utf-8 -*-
"""
Определение положения руки относительно тела по двум IMU-датчикам
(BNO08x на STM32, подключение по BLE — модуль E104-BT5032A), без
магнитометра.

БЫЛО: два "сырых" потока LSL (ax,ay,az,gx,gy,gz), кватернион считался
здесь локально фильтром Маджвика.
СТАЛО: STM32 сам вычисляет Game Rotation Vector на каждом BNO08x и шлёт
ГОТОВЫЕ кватернионы по BLE одной строкой:
    "w1 x1 y1 z1  w2 x2 y2 z2\\r\\n"
(сначала датчик с CS1, потом датчик с CS2 — см. main.c). Никакого
фильтра/интегрирования гироскопа здесь больше не нужно — вся сенсорная
фьюжн уже сделана на самом BNO08x.

Вся калибровочная/угловая математика (G/F/L, compute_calibration,
compute_outputs, classify_pose) НЕ МЕНЯЛАСЬ — она и раньше работала
только с кватернионами, откуда бы они ни брались.

===========================================================================
ЛОГИКА / МАТЕМАТИКА (коротко, подробнее — по ходу кода)
===========================================================================
Кватернион q(t) каждого датчика: локальная система датчика -> "мир
датчика" — внешняя, НЕ вращающаяся вместе с телом система координат, у
которой ось Z всегда точно вертикальна (гравитация — одна и та же
физическая величина для обоих датчиков). Горизонтальные оси X,Y "мира"
каждого датчика — условные (нулевой курс = тот, что был у датчика в момент
включения), и у двух датчиков они, вообще говоря, РАЗНЫЕ (отличаются на
неизвестный поворот вокруг Z). Задача калибровки — свести всё к одной общей
системе "тело" (x1 — вправо, y1 — вперёд, z1 — вверх по позвоночнику),
которая двигается вместе с датчиком на груди (CS1), поэтому углы руки
относительно неё не меняются при повороте/наклоне/лежании всего тела
целиком.

Калибровки:
  G — стоя прямо, руки вдоль тела, 2-4 сек, датчики неподвижны.
      -> ось z1 (в локальной системе груди) = направление мировой
         вертикали в этот момент.
      -> ось "вдоль руки" P (в локальной системе датчика руки) =
         направление мировой вертикали вниз в этот момент (рука висит).
  F — рука вытянута вперёд (горизонтально), корпус не двигается.
      -> определяет рассогласование по курсу (yaw) между "миром" датчика
         руки и "миром" датчика груди (calibrate_offset), приравнивая
         направление руки к направлению "вперёд" тела.
  L — корпус наклоняется вперёд из положения стоя.
      -> ось поворота этого наклона (в локальной системе груди) = ось x1
         (вправо); y1 достаётся из условия правой тройки x1×y1=z1.

Три выводимых значения:
  1) Нутация руки относительно z1 (0° — рука вверх, 180° — рука вниз,
     90° — рука горизонтально)                              [система ТЕЛО]
  2) Прецессия руки в плоскости x1y1 (0° — вперёд, +90° — вправо,
     -90° — влево, ±180° — назад)                            [система ТЕЛО]
  3) Наклон корпуса в мировой системе (плоскости xz и yz)     [система ЗЕМЛЯ]

Установка зависимостей:
    pip install bleak numpy
===========================================================================
"""

import asyncio
import math
import queue
import sys
import threading
import time

import numpy as np

from bleak import BleakClient

DEVICE_ADDRESS = "d6:7d:55:ae:d7:54"
SLAVE_CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"

quat_lock = threading.Lock()
current_chest_quat = np.array([1.0, 0.0, 0.0, 0.0])
current_arm_quat = np.array([1.0, 0.0, 0.0, 0.0])
ble_connected = False
running = True

_rx_buffer = ""
_debug_raw_count = 0
_notify_count = 0
_last_notify_count = 0


def parse_ble_line(line):
    """"w1 x1 y1 z1 w2 x2 y2 z2" -> (chest_quat, arm_quat) или None.
    ПОМЕНЯНО МЕСТАМИ: физически CS1 оказался на руке, CS2 - на груди."""
    try:
        parts = [float(p) for p in line.strip().split(" ") if p]
    except ValueError:
        return None
    if len(parts) != 8:
        return None
    arm_q = np.array(parts[0:4])
    chest_q = np.array(parts[4:8])
    return chest_q, arm_q


def _ble_notification_handler(sender, data: bytearray):
    global _rx_buffer, current_chest_quat, current_arm_quat, _debug_raw_count, _notify_count
    _notify_count += 1
    print(f"[NOTIFY #{_notify_count}] [RAW BYTES] {list(data)}", flush=True)
    try:
        chunk = data.decode("utf-8")
    except UnicodeDecodeError:
        print(f"[RAW TEXT] <decode error>", flush=True)
        return

    print(f"[RAW TEXT] {chunk!r}", flush=True)
    _rx_buffer += chunk
    while "\n" in _rx_buffer:
        line, _rx_buffer = _rx_buffer.split("\n", 1)
        print(f"[LINE] {line!r}", flush=True)
        parsed = parse_ble_line(line)
        if parsed is None:
            print("[PARSE FAIL]", flush=True)
            continue
        chest_q, arm_q = parsed
        _debug_raw_count += 1
        print(f"[PARSED] chest={np.array2string(chest_q, precision=6, separator=', ')}", flush=True)
        print(f"[PARSED] arm  ={np.array2string(arm_q, precision=6, separator=', ')}", flush=True)
        with quat_lock:
            current_chest_quat = chest_q
            current_arm_quat = arm_q


async def _ble_client_loop():
    global ble_connected, running
    while running:
        try:
            async with BleakClient(DEVICE_ADDRESS) as client:
                print(f"[BLE] Подключено к {DEVICE_ADDRESS}", flush=True)

                print("[BLE] Сервисы/характеристики устройства:", flush=True)
                for service in client.services:
                    print(f"  [SVC] {service.uuid}", flush=True)
                    for ch in service.characteristics:
                        print(f"    [CHR] {ch.uuid}  props={ch.properties}", flush=True)
                await client.start_notify(SLAVE_CHAR_UUID, _ble_notification_handler)
                print(f"[BLE] start_notify OK на {SLAVE_CHAR_UUID}, жду уведомления...", flush=True)
                ble_connected = True
                while running and client.is_connected:
                    await asyncio.sleep(0.2)
                print("[BLE] Соединение разорвано (client.is_connected == False)", flush=True)
                ble_connected = False
        except Exception as e:
            ble_connected = False
            print(f"[BLE] Ошибка подключения ({e}), повтор через 3 сек...", flush=True)
            await asyncio.sleep(3.0)


def ble_reader_thread():
    asyncio.run(_ble_client_loop())


def get_current_quats():
    with quat_lock:
        return current_chest_quat.copy(), current_arm_quat.copy()


def main():
    global running, _last_notify_count

    print("=== RAW BLE DATA ===", flush=True)
    threading.Thread(target=ble_reader_thread, daemon=True).start()

    try:
        tick = 0
        while True:
            time.sleep(0.2)
            tick += 1
            q_chest, q_arm = get_current_quats()
            print(f"stored chest={np.array2string(q_chest, precision=6, separator=', ')} | "
                  f"stored arm={np.array2string(q_arm, precision=6, separator=', ')}", flush=True)

            if tick % 5 == 0:  # раз в секунду
                delta = _notify_count - _last_notify_count
                _last_notify_count = _notify_count
                print(f"[HEARTBEAT] ble_connected={ble_connected} "
                      f"notify_count_total={_notify_count} (+{delta} за посл. сек)", flush=True)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        running = False


if __name__ == "__main__":
    main()
