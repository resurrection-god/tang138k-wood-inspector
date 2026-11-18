#!/usr/bin/env python3
"""Безопасная сетевая загрузка прошивки (OTA A/B Slot) для Tang Mega 138K Pro.

Протокол VMSP СТП-ПАК-026 (опкоды 0x0170..0x0179).
Схема двойного слота A/B:
  - Золотой слот A (0x600000): защищён аппаратно в RTL и коде, никогда не перезаписывается по сети.
  - Рабочий слот B (0xF60000): обновляется по сети через буфер в DDR3 (0x11000000).
  - При сбое или watchdog-таймауте загрузчик автоматически откатывается на слот A.
"""

import argparse
import os
import sys
import time
import zlib

# Добавляем корень репозитория в sys.path
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common import vmsp


def format_size(num_bytes):
    if num_bytes < 1024:
        return f"{num_bytes} B"
    elif num_bytes < 1024 * 1024:
        return f"{num_bytes / 1024:.1f} KB"
    return f"{num_bytes / (1024 * 1024):.2f} MB"


def print_status(st):
    print("------------------------------------------------------------")
    print(" [OTA СТАТУС ПЛАТЫ]")
    print(f"  Активный слот:   {st['slot_name']} (код {st['active_slot']})")
    print(f"  Статус загрузки: {st['boot_status_name']} (код {st['boot_status']})")
    print(f"  Попыток старта:  {st['boot_attempts']}")
    print(f"  Адрес слота A:   0x{st['slot_a_addr']:06X}")
    print(f"  Адрес слота B:   0x{st['slot_b_addr']:06X}")
    print(f"  CRC слота B:     0x{st['slot_b_crc']:08X}")
    print(f"  Размер слота B:  {st['slot_b_size']} Б ({format_size(st['slot_b_size'])})")
    print(f"  Build ID SPI:    0x{st['build_id']:08X}")
    print("------------------------------------------------------------")


def wait_for_board(host, timeout_s=15.0):
    t0 = time.time()
    print(f"Ожидание перезапуска платы ({host})...", end="", flush=True)
    time.sleep(1.0)
    while time.time() - t0 < timeout_s:
        try:
            st = vmsp.ota_status(host=host, timeout=1.5)
            print(" ОК!")
            return st
        except Exception:
            print(".", end="", flush=True)
            time.sleep(0.5)
    print(" ТАЙМАУТ!")
    return None


def main():
    parser = argparse.ArgumentParser(description="Безопасная сетевая загрузка прошивки (OTA A/B Slot)")
    parser.add_argument("--bin", default=os.path.join(REPO_ROOT, "firmware", "Debug", "output", "ae350_scanner.bin"),
                        help="Путь к бинарному файлу прошивки (.bin)")
    parser.add_argument("--host", default=vmsp.HOST, help=f"IP-адрес платы (по умолчанию: {vmsp.HOST})")
    parser.add_argument("--status", action="store_true", help="Только запросить текущий OTA статус")
    parser.add_argument("--no-reboot", action="store_true", help="Не перезагружать плату после записи")
    parser.add_argument("--chunk-size", type=int, default=1024, help="Размер пакета передачи (по умолчанию 1024 Б)")
    args = parser.parse_args()

    print("============================================================")
    print(" [OTA] Сетевая загрузка Andes AE350 (Dual-Slot A/B Safe OTA)")
    print("============================================================")
    print(f"Целевой хост: {args.host}")

    if args.status:
        try:
            st = vmsp.ota_status(host=args.host)
            print_status(st)
            return 0
        except Exception as e:
            print(f"ОШИБКА: Не удалось получить статус платы: {e}", file=sys.stderr)
            return 1

    if not os.path.isfile(args.bin):
        print(f"ОШИБКА: Файл прошивки не найден: {args.bin}", file=sys.stderr)
        return 1

    with open(args.bin, "rb") as f:
        fw_data = f.read()

    fw_size = len(fw_data)
    fw_crc = zlib.crc32(fw_data) & 0xFFFFFFFF

    print(f"Файл прошивки: {args.bin}")
    print(f"Размер образа: {fw_size} Б ({format_size(fw_size)})")
    print(f"CRC-32 образа: 0x{fw_crc:08X}")

    if fw_size < 1024 or fw_size > 512 * 1024:
        print(f"ОШИБКА: Недопустимый размер прошивки ({fw_size} Б). Допустимо: 1 КБ .. 512 КБ.", file=sys.stderr)
        return 1

    # Шаг 1: Проверка связи и текущего слота
    print("\n[1/4] Проверка состояния платы...")
    try:
        init_st = vmsp.ota_status(host=args.host)
        print_status(init_st)
    except Exception as e:
        print(f"ОШИБКА: Плата не отвечает по VMSP (порт {vmsp.PORT_CONTROL}): {e}", file=sys.stderr)
        return 1

    # Шаг 2: Потоковая передача в DDR3 staging буфер
    print("\n[2/4] Передача образа в промежуточный буфер DDR3 (0x11000000)...")
    t_start = time.time()
    last_print = 0

    def progress(cur, total):
        nonlocal last_print
        now = time.time()
        if now - last_print >= 0.1 or cur == total:
            last_print = now
            pct = (cur * 100.0) / total
            elapsed = max(now - t_start, 0.001)
            speed_kbps = (cur / 1024.0) / elapsed
            print(f"\r  Прогресс: {cur}/{total} Б ({pct:5.1f}%) [{speed_kbps:6.1f} КБ/с]", end="", flush=True)

    try:
        uploaded_bytes, calc_crc = vmsp.ota_upload(
            fw_data,
            host=args.host,
            chunk_size=args.chunk_size,
            progress_cb=progress
        )
        print(f"\n  Передача завершена за {time.time() - t_start:.2f} с. CRC буфера ОЗУ: 0x{calc_crc:08X}")
    except Exception as e:
        print(f"\nОШИБКА передачи: {e}", file=sys.stderr)
        return 1

    # Шаг 3: Фиксация (стирание слота B во флеш, запись, верификация read-back)
    print("\n[3/4] Запись и верификация во флеш Slot B (0x00F60000)...")
    t_commit = time.time()
    try:
        commit_res = vmsp.ota_commit(fw_crc, host=args.host)
        dur = time.time() - t_commit
        print(f"  Стёрто секторов: {commit_res['sectors_erased']} (4 КБ каждый)")
        print(f"  Проверено аппаратным read-back CRC: 0x{commit_res['verified_crc']:08X} (совпадает!)")
        print(f"  Время записи и верификации: {dur:.2f} с")
    except Exception as e:
        print(f"ОШИБКА фиксации во флеш: {e}", file=sys.stderr)
        return 1

    # Шаг 4: Перезагрузка и подтверждение
    if args.no_reboot:
        print("\n[4/4] Перезагрузка пропущена по флагу --no-reboot.")
        print("Для активации выполните перезагрузку вручную или отправьте команду OTA_REBOOT.")
        return 0

    print("\n[4/4] Перезагрузка платы и проверка нового слота...")
    vmsp.ota_reboot(delay_ms=100, host=args.host)

    new_st = wait_for_board(args.host, timeout_s=15.0)
    if not new_st:
        print("ВНИМАНИЕ: Плата не ответила после перезагрузки!", file=sys.stderr)
        return 1

    print_status(new_st)

    if new_st["active_slot"] == 1:
        print("\n============================================================")
        print(" [УСПЕХ] Прошивка успешно обновлена и запущена из Slot B!")
        if new_st["boot_status"] == 2:  # CONFIRMED
            print(" [СТАТУС] Слот B подтверждён как стабильный (CONFIRMED).")
        print("============================================================")
        return 0
    else:
        print("\nВНИМАНИЕ: Плата загрузилась из Slot A (откат или режим по умолчанию)!", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
