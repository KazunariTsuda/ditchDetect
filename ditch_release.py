#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""側溝検知停止の解除を 1 回送って終わる。

  送信 0x102  DLC 1  byte0=0x01

使い方（Raspberry Pi、can0 が 500 kbit/s で up 済み）:
  python3 ditch_release.py
  python3 ditch_release.py --channel can0

依存: pip install python-can
"""

from __future__ import annotations

import argparse
import sys
import time

CAN_RELEASE_ID = 0x102
CAN_RELEASE_DATA = bytes((0x01,))


def open_bus(channel: str, bitrate: int):
    try:
        import can
    except ImportError:
        print("python-can がありません。pip install python-can", file=sys.stderr)
        return None
    try:
        return can.interface.Bus(
            channel=channel,
            interface="socketcan",
            bitrate=bitrate,
        )
    except Exception as exc:
        print(f"CAN を開けませんでした ({channel}): {exc}", file=sys.stderr)
        print(
            f"例: sudo ip link set {channel} up type can bitrate {bitrate}",
            file=sys.stderr,
        )
        return None


def send_release(channel: str, bitrate: int) -> int:
    bus = open_bus(channel, bitrate)
    if bus is None:
        return 1

    import can

    msg = can.Message(
        arbitration_id=CAN_RELEASE_ID,
        data=CAN_RELEASE_DATA,
        is_extended_id=False,
    )
    try:
        bus.send(msg)
        # キューに入っただけでは、すぐ閉じるとフレームが落ちることがある
        time.sleep(0.05)
    except can.CanError as exc:
        print(f"解除の送信に失敗しました: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass
    print(f"停止解除を送信しました ({channel} 0x102 01)", flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="側溝検知停止の解除を 1 回送る")
    parser.add_argument("--channel", default="can0", help="SocketCAN のインターフェース（既定 can0）")
    parser.add_argument("--bitrate", type=int, default=500000, help="ビットレート（既定 500000）")
    args = parser.parse_args()
    return send_release(args.channel, args.bitrate)


if __name__ == "__main__":
    sys.exit(main())
