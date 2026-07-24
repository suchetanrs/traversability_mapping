#!/usr/bin/env python3
"""Compare bandwidth over time of the full grid_map topic vs the sparse update topic.

Subscribes to:
  * global_traversability_gridmap  (grid_map_msgs/msg/GridMap, reliable + transient_local)
  * global_traversability_updates  (traversability_msgs/msg/TraversabilitySparseUpdate, reliable)

Both subscriptions are *raw* (rclpy raw=True), so the callbacks receive the
serialized CDR bytes and we measure the true on-wire payload size of every
message, without having to deserialize.

Per window (default 1 s) we report, for each topic and the cumulative totals:
  * mbps        : instantaneous bandwidth over the window
  * hz          : message rate over the window
  * avg msg size: bytes/message over the window
  * total       : cumulative bytes and message count since start

Output modes:
  * terminal (default): a refreshing table printed to stdout
  * --plot           : a live matplotlib figure (mbps, hz, avg size) in addition
                       to the terminal table. Requires a display / GUI backend.

Examples:
  ./bandwidth_compare.py
  ./bandwidth_compare.py --window 0.5 --plot
  ./bandwidth_compare.py --history 120        # keep 120 s of plot history
"""

import argparse
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# Message types are only needed to obtain the typesupport for the raw
# subscription; we never actually deserialize the payloads.
from grid_map_msgs.msg import GridMap
from traversability_msgs.msg import TraversabilitySparseUpdate


@dataclass
class TopicStats:
    """Accumulates byte/message counts for a single topic."""

    name: str
    # Counters for the current (not-yet-reported) window. Guarded by `lock`.
    window_bytes: int = 0
    window_msgs: int = 0
    # Cumulative since node start.
    total_bytes: int = 0
    total_msgs: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def add(self, nbytes: int) -> None:
        with self.lock:
            self.window_bytes += nbytes
            self.window_msgs += 1
            self.total_bytes += nbytes
            self.total_msgs += 1

    def drain_window(self) -> tuple[int, int]:
        """Return (bytes, msgs) accumulated since the last drain and reset them."""
        with self.lock:
            b, m = self.window_bytes, self.window_msgs
            self.window_bytes = 0
            self.window_msgs = 0
            return b, m


@dataclass
class WindowSample:
    """One reported window's derived metrics for a topic."""

    t: float          # seconds since start (window end)
    mbps: float       # megabits per second over the window
    hz: float         # messages per second over the window
    avg_size: float   # bytes per message over the window (0 if no msgs)


class BandwidthMonitor(Node):
    def __init__(self, window: float, history: float):
        super().__init__("bandwidth_compare")

        self.window = window
        # Number of samples to retain for plotting / sparkline history.
        self.max_samples = max(1, int(round(history / window)))

        self.gridmap = TopicStats("global_traversability_gridmap")
        self.sparse = TopicStats("global_traversability_updates")

        # Per-topic time series of WindowSample, for the plot and sparklines.
        self.series: dict[str, deque] = {
            self.gridmap.name: deque(maxlen=self.max_samples),
            self.sparse.name: deque(maxlen=self.max_samples),
        }
        self.series_lock = threading.Lock()

        # --- Subscriptions -------------------------------------------------
        # Match the publishers' QoS so we are guaranteed compatible.
        gridmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sparse_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            GridMap,
            self.gridmap.name,
            lambda msg: self.gridmap.add(len(msg)),
            gridmap_qos,
            raw=True,
        )
        self.create_subscription(
            TraversabilitySparseUpdate,
            self.sparse.name,
            lambda msg: self.sparse.add(len(msg)),
            sparse_qos,
            raw=True,
        )

        self.start_time = time.monotonic()
        self.create_timer(self.window, self._on_window)

    def _on_window(self) -> None:
        now = time.monotonic()
        t = now - self.start_time
        with self.series_lock:
            for stats in (self.gridmap, self.sparse):
                nbytes, nmsgs = stats.drain_window()
                mbps = (nbytes * 8) / 1e6 / self.window
                hz = nmsgs / self.window
                avg = (nbytes / nmsgs) if nmsgs else 0.0
                self.series[stats.name].append(
                    WindowSample(t=t, mbps=mbps, hz=hz, avg_size=avg)
                )

    # --- Snapshot helpers for the renderers -------------------------------
    def snapshot(self) -> dict:
        """Thread-safe copy of latest sample + totals for each topic."""
        out = {}
        with self.series_lock:
            for stats in (self.gridmap, self.sparse):
                samples = list(self.series[stats.name])
                latest = samples[-1] if samples else WindowSample(0, 0, 0, 0)
                with stats.lock:
                    total_bytes = stats.total_bytes
                    total_msgs = stats.total_msgs
                out[stats.name] = {
                    "latest": latest,
                    "samples": samples,
                    "total_bytes": total_bytes,
                    "total_msgs": total_msgs,
                }
        out["elapsed"] = time.monotonic() - self.start_time
        return out


# ----------------------------------------------------------------------------
# Rendering
# ----------------------------------------------------------------------------

def fmt_bytes(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if abs(n) < 1024.0:
            return f"{n:6.1f} {unit}"
        n /= 1024.0
    return f"{n:6.1f} TB"


SPARK = "▁▂▃▄▅▆▇█"


def sparkline(values: list[float], width: int = 30) -> str:
    if not values:
        return ""
    vals = values[-width:]
    lo, hi = min(vals), max(vals)
    if hi <= lo:
        return SPARK[0] * len(vals)
    span = hi - lo
    return "".join(SPARK[min(len(SPARK) - 1, int((v - lo) / span * (len(SPARK) - 1)))] for v in vals)


class TerminalRenderer:
    LABELS = {
        "global_traversability_gridmap": "FULL gridmap",
        "global_traversability_updates": "SPARSE update",
    }

    def render(self, snap: dict) -> None:
        elapsed = snap["elapsed"]
        lines = []
        lines.append(f"\033[2J\033[H")  # clear screen, cursor home
        lines.append(f"  Traversability bandwidth compare   elapsed: {elapsed:7.1f} s")
        lines.append("  " + "-" * 88)
        header = (
            f"  {'topic':<14} {'mbps':>8} {'hz':>7} {'avg msg':>11} "
            f"{'total data':>11} {'msgs':>8}  trend(mbps)"
        )
        lines.append(header)
        lines.append("  " + "-" * 88)

        totals = {"mbps": 0.0, "bytes": 0, "msgs": 0}
        for name in (
            "global_traversability_gridmap",
            "global_traversability_updates",
        ):
            d = snap[name]
            s = d["latest"]
            totals["mbps"] += s.mbps
            totals["bytes"] += d["total_bytes"]
            totals["msgs"] += d["total_msgs"]
            spark = sparkline([x.mbps for x in d["samples"]])
            lines.append(
                f"  {self.LABELS[name]:<14} {s.mbps:8.3f} {s.hz:7.2f} "
                f"{fmt_bytes(s.avg_size):>11} {fmt_bytes(d['total_bytes']):>11} "
                f"{d['total_msgs']:8d}  {spark}"
            )

        lines.append("  " + "-" * 88)
        lines.append(
            f"  {'TOTAL':<14} {totals['mbps']:8.3f} {'':>7} {'':>11} "
            f"{fmt_bytes(totals['bytes']):>11} {totals['msgs']:8d}"
        )

        # Headline comparison: how much the sparse channel saves vs the full map.
        g = snap["global_traversability_gridmap"]["latest"].mbps
        sp = snap["global_traversability_updates"]["latest"].mbps
        if g > 0:
            ratio = sp / g if sp > 0 else 0.0
            saving = (1 - ratio) * 100
            lines.append("")
            lines.append(
                f"  sparse / full = {ratio:6.2%}   "
                f"=> sparse uses {saving:5.1f}% less bandwidth than the full grid_map"
            )
        lines.append("")
        sys.stdout.write("\n".join(lines))
        sys.stdout.flush()


class PlotRenderer:
    """Live matplotlib renderer. Created lazily so --plot is optional."""

    def __init__(self):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.fig, self.axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        self.fig.suptitle("Traversability bandwidth: full grid_map vs sparse updates")
        self.ax_mbps, self.ax_hz, self.ax_size = self.axes
        self.ax_mbps.set_ylabel("Mbps")
        self.ax_hz.set_ylabel("Hz")
        self.ax_size.set_ylabel("avg msg (bytes)")
        self.ax_size.set_xlabel("time (s)")
        for ax in self.axes:
            ax.grid(True, alpha=0.3)
        self.styles = {
            "global_traversability_gridmap": ("FULL gridmap", "tab:red"),
            "global_traversability_updates": ("SPARSE update", "tab:green"),
        }
        plt.ion()
        plt.show(block=False)

    def render(self, snap: dict) -> None:
        for ax, attr in (
            (self.ax_mbps, "mbps"),
            (self.ax_hz, "hz"),
            (self.ax_size, "avg_size"),
        ):
            ax.clear()
            ax.grid(True, alpha=0.3)
            for name, (label, color) in self.styles.items():
                samples = snap[name]["samples"]
                xs = [s.t for s in samples]
                ys = [getattr(s, attr) for s in samples]
                ax.plot(xs, ys, label=label, color=color)
            ax.legend(loc="upper left", fontsize=8)
        self.ax_mbps.set_ylabel("Mbps")
        self.ax_hz.set_ylabel("Hz")
        self.ax_size.set_ylabel("avg msg (bytes)")
        self.ax_size.set_xlabel("time (s)")
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--window", type=float, default=1.0,
                        help="reporting/integration window in seconds (default 1.0)")
    parser.add_argument("--history", type=float, default=60.0,
                        help="seconds of history to retain for plot/sparklines (default 60)")
    parser.add_argument("--plot", action="store_true",
                        help="show a live matplotlib figure in addition to the terminal table")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = BandwidthMonitor(window=args.window, history=args.history)

    # Spin ROS in a background thread so the main thread can drive rendering
    # (matplotlib in particular must run on the main thread).
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    term = TerminalRenderer()
    plot = PlotRenderer() if args.plot else None

    try:
        while rclpy.ok():
            snap = node.snapshot()
            term.render(snap)
            if plot is not None:
                plot.render(snap)
            time.sleep(args.window)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.stdout.write("\n")


if __name__ == "__main__":
    main()
