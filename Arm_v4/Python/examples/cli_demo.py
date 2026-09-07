# -*- coding: utf-8 -*-
"""
Консольная демонстрация пакета posetracker: интерактивная калибровка +
непрерывный вывод текущего положения руки на экран.

ЭТОТ ФАЙЛ - ПОТРЕБИТЕЛЬ БИБЛИОТЕКИ, НЕ ЧАСТЬ БИБЛИОТЕКИ.
Раньше вся эта логика (input(), countdown, print с форматированием,
чтение команд из stdin) была перемешана прямо с BLE-кодом и математикой
в одном файле Get_calculate_show.py. Теперь всё, что "про консоль" -
здесь, а всё, что "про датчики и математику" - в пакете posetracker/. Это
и есть практический пример того, зачем нужен "чёрный ящик": если завтра
понадобится не консольный вывод, а, скажем, окошко на tkinter или
веб-страница - пишется НОВЫЙ файл-потребитель (examples/gui_demo.py или
что угодно), а пакет posetracker/ вообще не трогается - весь код внутри
него ничего не знает про input()/print() и не привязан к консоли.

ЗАПУСК:
Пакет должен быть "виден" интерпретатору. Проще всего - установить его
в editable-режиме один раз (из папки Python/):
    pip install -e .
и дальше просто:
    python examples/cli_demo.py
Если лень ставить пакет - альтернативный (менее правильный, но рабочий
для быстрой проверки) способ - запускать из папки Python/, тогда Python
найдёт папку posetracker/ рядом сам, без pip install.
"""

import math
import queue
import sys
import threading
import time

from posetracker import (
    DEFAULT_DURATION_F,
    DEFAULT_DURATION_G,
    DEFAULT_DURATION_L,
    PoseTracker,
)

# Частота вывода значений на экран, Гц. Это настройка именно ВЫВОДА (как
# часто перерисовывать строку в консоли), а не того, как часто реально
# обновляются данные с датчиков - поэтому она живёт здесь, а не в пакете.
PRINT_RATE_HZ = 10.0

RECAL_COMMANDS = {"r", "р", "recal", "перекалибровка"}


def countdown(seconds):
    """Печатает обратный отсчёт в консоль. Чисто UI-функция - в пакете
    posetracker для неё нет и не должно быть места."""
    n = int(math.ceil(seconds))
    for i in range(n, 0, -1):
        print(f"  {i}...", end=" ", flush=True)
        time.sleep(min(1.0, seconds - (n - i)))
    print()


def prompt_zone_counts():
    """Спрашивает у пользователя, на сколько зон делить нутацию (0-180°) и
    прецессию (360°) в начале сеанса."""
    def ask(text):
        while True:
            raw = input(text).strip()
            try:
                n = int(raw)
            except ValueError:
                print("  Введите целое число >= 1.")
                continue
            if n < 1:
                print("  Введите целое число >= 1.")
                continue
            return n

    n_nutation = ask("На сколько зон делить нутацию (0-180°)? ")
    n_precession = ask("На сколько зон делить прецессию (360°)? ")
    return n_nutation, n_precession


def stdin_reader_loop(cmd_queue):
    """Работает в фоновом потоке: построчно читает stdin и кладёт строки
    в очередь. Используется вместо input(), чтобы не конфликтовать с
    основным циклом вывода значений на экран (два одновременных input()
    на одной консоли друг другу мешают)."""
    while True:
        try:
            line = sys.stdin.readline()
        except Exception:
            break
        if not line:
            break
        cmd_queue.put(line.strip())


def wait_for_line(cmd_queue, prompt):
    """Печатает prompt и блокируется, пока в очереди команд не появится
    любая строка (аналог input(), но через фоновый поток чтения stdin)."""
    print(prompt)
    while True:
        try:
            cmd_queue.get(timeout=0.1)
            return
        except queue.Empty:
            continue


def run_calibration(tracker: PoseTracker):
    """Три интерактивные позы подряд: G, F, L. Здесь, а не в пакете,
    потому что именно эта конкретная последовательность prompt/countdown -
    решение КОНКРЕТНОГО консольного сценария, а не универсальное свойство
    трекера (другой потребитель мог бы, например, просто ждать нажатия
    кнопки на джойстике вместо Enter)."""
    input("\nКалибровка G: встаньте прямо, руки свободно вдоль тела. "
          "Нажмите Enter и замрите...")
    countdown(DEFAULT_DURATION_G)
    tracker.calibrate_g()
    print("  калибровка G выполнена.")

    input("\nКалибровка F: вытяните руку строго вперёд (горизонтально, "
          "локоть прямой), корпус не двигайте. Нажмите Enter и замрите...")
    countdown(DEFAULT_DURATION_F)
    tracker.calibrate_f()
    print("  калибровка F выполнена.")

    input("\nКалибровка L: вернитесь в положение стоя, затем наклонитесь "
          "вперёд и задержитесь в наклоне. Нажмите Enter и выполните наклон...")
    countdown(DEFAULT_DURATION_L)
    tracker.calibrate_l()
    print("  калибровка L выполнена.")

    cal = tracker.finalize_calibration()
    print(f"\nКалибровка завершена. Рассогласование курса рука/грудь: "
          f"{cal.delta_psi_deg:.1f}°\n")


def run_recalibration(tracker: PoseTracker):
    """Переделывает только курс/наклон (F и L) - см. объяснение про дрейф
    курса без магнитометра в docstring исходного проекта."""
    print("\n=== ПЕРЕКАЛИБРОВКА (курс руки + наклон) ===")
    input("Вытяните руку строго вперёд (горизонтально, локоть прямой), "
          "корпус не двигайте. Нажмите Enter и замрите...")
    countdown(DEFAULT_DURATION_F)
    tracker.calibrate_f()
    print("  калибровка F выполнена.")

    input("Встаньте прямо, затем наклонитесь вперёд и задержитесь в "
          "наклоне. Нажмите Enter и выполните наклон...")
    countdown(DEFAULT_DURATION_L)
    tracker.calibrate_l()
    print("  калибровка L выполнена.")

    cal = tracker.finalize_calibration()
    print(f"Перекалибровка выполнена. Новое рассогласование курса: "
          f"{cal.delta_psi_deg:.1f}°\n")


def main():
    print("=" * 70)
    print("Определение положения руки относительно тела (BLE, BNO08x)")
    print("=" * 70)

    n_nutation_zones, n_precession_zones = prompt_zone_counts()
    print(f"Зон нутации: {n_nutation_zones} (по 180/{n_nutation_zones}="
          f"{180.0 / n_nutation_zones:.1f}° каждая), "
          f"зон прецессии: {n_precession_zones} (по 360/{n_precession_zones}="
          f"{360.0 / n_precession_zones:.1f}° каждая)\n")

    # Вся конфигурация "чёрного ящика" - в одном месте создания объекта.
    # Дальше tracker - это всё, с чем работает остальной код: ни одного
    # упоминания BLE-адреса/UUID/деталей SPI-протокола STM32 за пределами
    # posetracker/ble_link.py.
    tracker = PoseTracker(
        n_nutation_zones=n_nutation_zones,
        n_precession_zones=n_precession_zones,
    )

    print(f"Подключение к BLE-модулю {tracker.device_address}...")

    def on_waiting():
        print("Не удалось подключиться за 15 сек. Проверьте, что модуль "
              "включён и STM32 передаёт данные. Продолжаю ждать в фоне...")

    tracker.connect(on_waiting=on_waiting)
    print("Подключено, данные идут.\n")

    try:
        run_calibration(tracker)
    except RuntimeError as e:
        print(f"ОШИБКА: {e}\nПроверьте подключение BLE и повторите запуск.")
        sys.exit(1)

    # с этого момента stdin читает только фоновый поток — input() больше
    # не вызываем, чтобы не конфликтовать с ним за одну и ту же консоль
    cmd_queue = queue.Queue()
    threading.Thread(target=stdin_reader_loop, args=(cmd_queue,), daemon=True).start()

    print("В любой момент введите 'r' и нажмите Enter, чтобы "
          "перекалибровать курс руки и наклон (F и L) заново — это "
          "нужно делать периодически, т.к. курс между датчиками может "
          "немного расходиться со временем, особенно после активных "
          "движений рукой.")
    print("Начинаю вывод углов (Ctrl+C для остановки)...\n")

    print_period = 1.0 / PRINT_RATE_HZ
    last_print = 0.0

    try:
        while True:
            try:
                cmd = cmd_queue.get_nowait()
            except queue.Empty:
                cmd = None
            if cmd is not None and cmd.lower() in RECAL_COMMANDS:
                run_recalibration(tracker)
                print("Продолжаю вывод углов (Ctrl+C для остановки, "
                      "'r'+Enter для перекалибровки)...\n")

            now = time.time()
            if now - last_print >= print_period:
                last_print = now
                result = tracker.get_pose()
                pose_str = (f"нутация {result.nutation_zone}/{result.n_nutation_zones}, "
                            f"прецессия {result.precession_zone}/{result.n_precession_zones}")
                link_str = "" if tracker.connected else "  [BLE ОТКЛЮЧЕНО]"
                print(f"[ТЕЛО] Нутация: {result.nutation_deg:7.1f}°   "
                      f"Прецессия: {result.precession_deg:7.1f}°   |   "
                      f"[ЗЕМЛЯ] Наклон корпуса XZ: {result.tilt_xz_deg:6.1f}°  "
                      f"YZ: {result.tilt_yz_deg:6.1f}°   |   Положение: {pose_str}{link_str}")

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nОстановлено пользователем.")
    finally:
        tracker.disconnect()


if __name__ == "__main__":
    main()
