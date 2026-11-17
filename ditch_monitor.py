#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""側溝検知のモニタ。

PSoC が CAN で出す四隅の検知結果と生距離を表示し、
実行した日時の名前の CSV に、停止するまで 1 行ずつ残す。
停止解除は送らない。解除は ditch_release.py。

  受信 0x0E1  DLC 7  約 100 ms。FL, FR, RL, RR の検知と IGNms、詰まり
         1=路上 / 2=不整地 / 3=側溝 / 4=停止 / 5=詰まり / 8=異常 / 0xFF=未確定
         byte4,5 は無視の残り ms。上位が先。無視中でなければ 0
         byte6 は詰まり。bit0=FL bit1=FR bit2=RL bit3=RR
  受信 0x0E0  DLC 8  同じ周期の生距離 mm。各 2 バイト、上位が先。
         0xFFFF は無効。0x0E1 の直後に届く
  受信 0x0D0  DLC 5  側溝のときだけ。hole, FL, FR, RL, RR
  受信 0x102  DLC 1  byte0=0x01 で、記録上の停止表示を監視中に戻す

使い方（Raspberry Pi、can0 が 500 kbit/s で up 済み）:
  python3 ditch_monitor.py
  python3 ditch_monitor.py --channel can0

CSV はスクリプトと同じ場所の log に置く。無ければ作る。

依存: pip install python-can
"""

from __future__ import annotations

import argparse
import csv
import queue
import sys
import threading
from datetime import datetime
from pathlib import Path

CAN_EVENT_ID = 0x0D0
CAN_CLASS_ID = 0x0E1
CAN_RAW_ID = 0x0E0
CAN_RELEASE_ID = 0x102
CAN_RELEASE_CMD = 0x01
RAW_INVALID = 0xFFFF
CODE_PENDING = 0xFF

CORNERS = ("FL", "FR", "RL", "RR")
CODE_NAME = {1: "路上", 2: "不整地", 3: "側溝", 4: "停止", 5: "詰まり", 8: "異常", CODE_PENDING: "未確定"}


def decode_class(data: bytes) -> tuple[tuple[int, int, int, int], int, int] | None:
    """0x0E1。四隅の検知コード、残り無視時間 ms、詰まりビット。"""
    if len(data) < 4:
        return None
    codes = (data[0], data[1], data[2], data[3])
    ign_ms = 0
    clog = 0
    if len(data) >= 6:
        ign_ms = (data[4] << 8) | data[5]
    if len(data) >= 7:
        clog = data[6]
    return codes, ign_ms, clog


def decode_event(data: bytes) -> tuple[int, tuple[int, int, int, int]] | None:
    """0x0D0。hole と四隅コード。短いフレームは捨てる。"""
    if len(data) < 5:
        return None
    return data[0], (data[1], data[2], data[3], data[4])


def decode_raw(data: bytes) -> tuple[int, int, int, int] | None:
    """0x0E0。四隅の距離 mm（上位バイトが先）。"""
    if len(data) < 8:
        return None
    return (
        (data[0] << 8) | data[1],
        (data[2] << 8) | data[3],
        (data[4] << 8) | data[5],
        (data[6] << 8) | data[7],
    )


def surface_text(codes: list[int | None]) -> str:
    """PSoC の SUM と同じ順。側溝、詰まり、異常、不整地、路上、停止。"""
    known = [c for c in codes if c is not None and c != CODE_PENDING]
    if not known:
        return "未確定"
    if 3 in known:
        return "側溝"
    if 5 in known:
        return "詰まり"
    if 8 in known:
        return "異常"
    if 2 in known:
        return "不整地"
    if 1 in known:
        return "路上"
    if 4 in known:
        return "停止"
    return "未確定"


def code_text(code: int | None) -> str:
    if code is None:
        return "-"
    name = CODE_NAME.get(code)
    if name is None:
        return str(code)
    return f"{code}({name})"


def mm_text(mm: int | None) -> str:
    if mm is None:
        return "----"
    if mm == RAW_INVALID:
        return "無効"
    return f"{mm}mm"


def mm_cell(mm: int | None) -> str:
    if mm is None:
        return ""
    return str(mm)


def stamp_text(moment: datetime | None = None) -> str:
    moment = datetime.now() if moment is None else moment
    return moment.strftime("%Y-%m-%d %H:%M:%S.") + f"{moment.microsecond // 1000:03d}"


def csv_path(out_dir: Path, moment: datetime | None = None) -> Path:
    """実行日時。同じ秒に再実行したら _2, _3 を付ける。"""
    moment = datetime.now() if moment is None else moment
    stem = moment.strftime("%Y%m%d_%H%M%S")
    path = out_dir / f"{stem}.csv"
    suffix = 2
    while path.exists():
        path = out_dir / f"{stem}_{suffix}.csv"
        suffix += 1
    return path


class Log:
    """最新の生値と検知を 1 行にまとめて追記する。"""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.rows = 0
        self.raw: list[int | None] = [None, None, None, None]
        self.codes: list[int | None] = [None, None, None, None]
        self.ign_ms = 0
        self.clog = 0
        self.stopped = False
        self._raw_new = False
        self._class_new = False
        self._file = path.open("w", encoding="utf-8-sig", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            ["時刻", "FL_mm", "FR_mm", "RL_mm", "RR_mm", "検知", "停止", "FL", "FR", "RL", "RR", "IGNms", "CLOG"]
        )
        self._file.flush()

    def close(self) -> None:
        if self._file.closed:
            return
        self._file.flush()
        self._file.close()

    def set_raw(self, values: tuple[int, int, int, int]) -> None:
        self.raw = list(values)
        self._raw_new = True

    def set_codes(self, codes: tuple[int, int, int, int], ign_ms: int = 0, clog: int = 0) -> None:
        self.codes = list(codes)
        self.ign_ms = ign_ms
        self.clog = clog
        self._class_new = True

    def pair_ready(self) -> bool:
        """生値と検知がそろった。届く順が前後しても 1 行にする。"""
        if not (self._raw_new and self._class_new):
            return False
        self._raw_new = False
        self._class_new = False
        return True

    def set_event(self, codes: tuple[int, int, int, int]) -> None:
        self.set_codes(codes, self.ign_ms)
        self.stopped = True

    def status_text(self) -> str:
        return "停止中" if self.stopped else "監視中"

    def write(self, moment: datetime | None = None) -> str:
        text = stamp_text(moment)
        self._writer.writerow(
            [
                text,
                *[mm_cell(v) for v in self.raw],
                surface_text(self.codes),
                self.status_text(),
                *["" if c is None else str(c) for c in self.codes],
                self.ign_ms,
                self.clog,
            ]
        )
        self._file.flush()
        self.rows += 1
        return text

    def line(self) -> str:
        raw = " ".join(f"{name}={mm_text(v)}" for name, v in zip(CORNERS, self.raw))
        codes = " ".join(f"{name}={code_text(c)}" for name, c in zip(CORNERS, self.codes))
        return (
            f"{raw}  検知={surface_text(self.codes)}  停止={self.status_text()}"
            f"  {codes}  IGNms={self.ign_ms}  CLOG=0x{self.clog:02X}"
        )


def open_bus(channel: str, bitrate: int):
    try:
        import can
    except ImportError:
        print("python-can がありません。pip install python-can", file=sys.stderr)
        return None
    filters = [
        {"can_id": CAN_EVENT_ID, "can_mask": 0x7FF, "extended": False},
        {"can_id": CAN_CLASS_ID, "can_mask": 0x7FF, "extended": False},
        {"can_id": CAN_RAW_ID, "can_mask": 0x7FF, "extended": False},
        {"can_id": CAN_RELEASE_ID, "can_mask": 0x7FF, "extended": False},
    ]
    try:
        return can.interface.Bus(
            channel=channel,
            interface="socketcan",
            bitrate=bitrate,
            can_filters=filters,
        )
    except Exception as exc:
        print(f"CAN を開けませんでした ({channel}): {exc}", file=sys.stderr)
        print(
            f"例: sudo ip link set {channel} up type can bitrate {bitrate}",
            file=sys.stderr,
        )
        return None


def stdin_reader(commands: queue.Queue[str]) -> None:
    while True:
        line = sys.stdin.readline()
        if line == "":
            return
        commands.put(line.strip().lower())


def handle_command(cmd: str) -> bool:
    """True なら監視を終える。"""
    if cmd in ("q", "quit", "exit"):
        return True
    if cmd:
        print("q と Enter または Ctrl+C で終了。停止解除は ditch_release.py", flush=True)
    return False


def run(channel: str, bitrate: int, out_dir: Path) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    bus = open_bus(channel, bitrate)
    if bus is None:
        return 1

    import can

    log = Log(csv_path(out_dir))
    commands: queue.Queue[str] = queue.Queue()
    reader = threading.Thread(target=stdin_reader, args=(commands,), daemon=True)
    reader.start()

    print(f"記録先: {log.path.resolve()}", flush=True)
    print(f"CAN: {channel}  約 100 ms で 0x0E1（検知と IGNms）と 0x0E0（生値 mm）", flush=True)
    print("側溝のときだけ 0x0D0。停止解除は ditch_release.py。q と Enter または Ctrl+C で終了", flush=True)
    print("隅コード: 1=路上 2=不整地 3=側溝 4=停止 5=詰まり 8=異常    生値 65535 は無効", flush=True)

    try:
        while True:
            while True:
                try:
                    cmd = commands.get_nowait()
                except queue.Empty:
                    break
                if handle_command(cmd):
                    return 0

            try:
                msg = bus.recv(timeout=0.1)
            except can.CanError as exc:
                print(f"CAN 受信エラー: {exc}", flush=True)
                continue
            if msg is None or msg.is_extended_id:
                continue

            if msg.arbitration_id == CAN_CLASS_ID:
                decoded = decode_class(bytes(msg.data))
                if decoded is None:
                    continue
                codes, ign_ms, clog = decoded
                log.set_codes(codes, ign_ms, clog)
                if log.pair_ready():
                    when = log.write()
                    print(f"{when}  {log.line()}", flush=True)
            elif msg.arbitration_id == CAN_RAW_ID:
                raw = decode_raw(bytes(msg.data))
                if raw is None:
                    continue
                log.set_raw(raw)
                if log.pair_ready():
                    when = log.write()
                    print(f"{when}  {log.line()}", flush=True)
            elif msg.arbitration_id == CAN_RELEASE_ID:
                if len(msg.data) >= 1 and msg.data[0] == CAN_RELEASE_CMD:
                    log.stopped = False
                    print("停止解除 0x102 を受信しました。記録の停止は監視中に戻します", flush=True)
            elif msg.arbitration_id == CAN_EVENT_ID:
                event = decode_event(bytes(msg.data))
                if event is None:
                    continue
                _hole, codes = event
                log.set_event(codes)
                when = log.write()
                print(f"{when}  側溝検知  {log.line()}", flush=True)
    except KeyboardInterrupt:
        return 0
    finally:
        rows = log.rows
        path = log.path
        log.close()
        try:
            bus.shutdown()
        except Exception:
            pass
        print(f"\n記録を閉じました: {path.resolve()} ({rows} 行)", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="側溝検知の表示と CSV 記録")
    parser.add_argument("--channel", default="can0", help="SocketCAN のインターフェース（既定 can0）")
    parser.add_argument("--bitrate", type=int, default=500000, help="ビットレート（既定 500000）")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent / "log",
        help="CSV の保存先ディレクトリ（既定はスクリプト横の log）",
    )
    args = parser.parse_args()
    return run(args.channel, args.bitrate, args.out)


if __name__ == "__main__":
    sys.exit(main())
