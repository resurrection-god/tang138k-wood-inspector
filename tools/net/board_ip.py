"""The board's IP address: find the board, change its address, remember it on the PC.

Since 15.10.2026 the firmware keeps the address, mask and gateway in its flash (EEPROM until 22.10.2026)
(net_config.c) and changes them on 0x0003 (spec § 4.1). The PC keeps the
address in tools/board_host.txt (not in git), which every tool takes as
vmsp.HOST (VMSP_HOST still wins). Moving the board between networks needs no
code change:

    python tools/net/board_ip.py                                 # boards found, the PC's address
    python tools/net/board_ip.py --set 10.20.0.57/24 --gw 10.20.0.1
    python tools/net/board_ip.py --set 192.168.31.125/24         # back to the direct cable
    python tools/net/board_ip.py --set 10.20.0.57/24 --no-store  # until the next reboot
    python tools/net/board_ip.py --remember 10.20.0.57           # only the PC side

The board answers 0x0001/0x0003 only in STANDBY; --set puts it there over TCP
first when it is reachable. A board in another subnet is found and re-addressed
by broadcast, by its MAC. Lost the address altogether: UART '6' shows it, '7'
returns to the default 192.168.31.125.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common import vmsp  # noqa: E402


def find(timeout=1.5):
    return vmsp.discover_all(timeout=timeout, hosts=[vmsp.HOST])


def show():
    src = ("VMSP_HOST" if os.environ.get("VMSP_HOST") else
           vmsp.HOST_FILE if os.path.exists(vmsp.HOST_FILE) else "по умолчанию")
    print(f"ПК: адрес платы {vmsp.HOST} ({src})")
    boards = find()
    for b in boards:
        print(f"плата {b['mac']}: {b['ip']} (ответ с {b.get('addr')})")
    if not boards:
        try:
            mode = vmsp.get_mode(vmsp.HOST)
            print(f"discovery молчит, но {vmsp.HOST} отвечает по TCP: режим "
                  f"{vmsp.MODE_NAMES.get(mode, mode)} (discovery работает только в STANDBY)")
        except Exception as e:  # noqa: BLE001
            print(f"плата не найдена: discovery молчит, {vmsp.HOST} по TCP — {e}")
    return boards


def to_standby():
    try:
        if vmsp.get_mode(vmsp.HOST) != vmsp.MODE_STANDBY:
            vmsp.set_mode(vmsp.MODE_STANDBY, vmsp.HOST)
            print(f"{vmsp.HOST}: переведена в STANDBY")
    except Exception:  # noqa: BLE001
        pass            # не отвечает по TCP: видимо, в другой подсети — ищем широковещанием


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", metavar="IP/PREFIX", help="новый адрес платы, например 10.20.0.57/24")
    ap.add_argument("--gw", default="0.0.0.0", help="шлюз (по умолчанию нет)")
    ap.add_argument("--mac", help="MAC платы, если их несколько")
    ap.add_argument("--no-store", action="store_true", help="не писать во флеш (до перезагрузки)")
    ap.add_argument("--remember", metavar="IP", help="только запомнить адрес на ПК")
    ap.add_argument("--set-mac", metavar="MAC", help="записать плате новый MAC (со следующей загрузки); "
                    "00:00:00:00:00:00 — стереть, вернётся MAC по умолчанию")
    a = ap.parse_args()

    if a.set_mac:
        new = vmsp.mac_bytes(a.set_mac)          # ValueError on a malformed MAC
        to_standby()
        boards = find()
        if a.mac:
            boards = [b for b in boards if b["mac"].lower() == a.mac.lower()]
        if len(boards) != 1:
            print(f"нужна ровно одна плата, найдено {len(boards)}")
            return 1
        st = vmsp.assign_mac(boards[0]["mac"], a.set_mac)
        print(f"плата {boards[0]['mac']}: новый MAC {':'.join(f'{x:02X}' for x in new)} -> {st or 'нет ответа'}"
              + (" — перезагрузите плату, MAC применится при загрузке" if st == "OK" else
                 " — MAC не одноадресный (младший бит первого байта = 1)" if st == "DENIED" else ""))
        return 0 if st == "OK" else 1
    if a.remember:
        vmsp.save_host(a.remember)
        print(f"ПК: адрес платы {a.remember} записан в {vmsp.HOST_FILE}")
        return 0
    if not a.set:
        show()
        return 0

    ip, _, prefix = a.set.partition("/")
    mask = vmsp.prefix_to_mask(prefix or 24)
    to_standby()
    boards = find()
    if a.mac:
        boards = [b for b in boards if b["mac"].lower() == a.mac.lower()]
    if len(boards) != 1:
        print(f"нужна ровно одна плата, найдено {len(boards)}" + ("" if boards else
              " (в STANDBY ли она? UART '6' покажет её адрес)"))
        for b in boards:
            print(f"  {b['mac']}: {b['ip']}")
        return 1
    b = boards[0]
    print(f"плата {b['mac']}: {b['ip']} -> {ip} маска {mask} шлюз {a.gw}"
          + (" (без записи во флеш)" if a.no_store else ""))
    st = vmsp.assign_ip(b["mac"], ip, mask, a.gw, store=not a.no_store)
    if st != "OK":
        print(f"0x0003: {st or 'нет ответа'}" + (" — адрес, маска или шлюз не годятся" if st == "DENIED" else
              " — запись во флеш не удалась" if st == "HW_ERROR" else ""))
        return 1
    time.sleep(0.3)
    again = [x for x in find() if x["mac"] == b["mac"]]
    got = again[0]["ip"] if again else None
    if got != ip:
        print(f"плата не подтвердила новый адрес (discovery: {got})")
        return 1
    vmsp.save_host(ip)
    print(f"готово: плата отвечает с {ip}; ПК запомнил его в {vmsp.HOST_FILE}")
    if ip.rsplit(".", 1)[0] not in " ".join(vmsp._local_ipv4()):
        print("внимание: у ПК нет адреса в этой подсети /24 — TCP до платы не дойдёт, "
              "пока ПК не окажется в той же сети")
    return 0


if __name__ == "__main__":
    sys.exit(main())
