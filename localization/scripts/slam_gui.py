#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Control GUI for the EUFS graph SLAM node.

Two modes:
- Mapping (SLAM): build and refine a map; save it to the map directory.
- Localization: pick a saved map, preview it, and localize against it.

The GUI drives the running graph SLAM node through its services
(``~/save_map``, ``~/load_map``, ``~/start_mapping``) and the ``load_map_path``
parameter. Map previews are rendered directly from the CSV files.
"""

import argparse
import csv
import glob
import os
import sys
import threading

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSDurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from hyu_msgs.msg import ConeArrayWithCovariance  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from PyQt5 import QtCore, QtWidgets  # noqa: E402

CONE_COLORS = {
    "blue": "#1a5fff",
    "yellow": "#ffd91a",
    "orange": "#ff7300",
    "big_orange": "#ff3300",
    "unknown": "#999999",
}


def read_map_csv(path):
    """Return (cones, start) where cones is a list of (tag, x, y)."""
    cones = []
    start = None
    with open(path) as f:
        for row in csv.DictReader(f):
            tag = row["tag"].strip()
            x, y = float(row["x"]), float(row["y"])
            if tag == "car_start":
                start = (x, y)
            else:
                cones.append((tag, x, y))
    return cones, start


class GuiNode(Node):
    """ROS side: service clients, parameter client, live-map subscription."""

    def __init__(self, slam_ns, map_topic):
        super().__init__("slam_gui")
        self.slam_ns = slam_ns.rstrip("/")
        self.save_cli = self.create_client(Trigger, f"{self.slam_ns}/save_map")
        self.load_cli = self.create_client(Trigger, f"{self.slam_ns}/load_map")
        self.map_cli = self.create_client(Trigger, f"{self.slam_ns}/start_mapping")
        self.param_cli = self.create_client(
            SetParameters, f"{self.slam_ns}/set_parameters")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.live_cones = 0
        self.create_subscription(
            ConeArrayWithCovariance, map_topic, self.on_map, latched)

    def on_map(self, msg):
        self.live_cones = (
            len(msg.blue_cones) + len(msg.yellow_cones) + len(msg.orange_cones)
            + len(msg.big_orange_cones) + len(msg.unknown_color_cones))

    def set_load_path(self, path, done):
        if not self.param_cli.service_is_ready():
            done(False, "graph SLAM parameter service not available")
            return
        req = SetParameters.Request()
        p = Parameter()
        p.name = "load_map_path"
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=path)
        req.parameters = [p]
        fut = self.param_cli.call_async(req)
        fut.add_done_callback(lambda f: done(True, "param set"))

    def call_trigger(self, cli, done):
        if not cli.service_is_ready():
            done(False, "service not available")
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(
            lambda f: done(f.result().success, f.result().message))


class MainWindow(QtWidgets.QWidget):
    map_status = QtCore.pyqtSignal(str)

    def __init__(self, node, map_dir):
        super().__init__()
        self.node = node
        self.map_dir = map_dir
        self.setWindowTitle("EUFS Graph SLAM")
        self.resize(900, 560)

        root = QtWidgets.QHBoxLayout(self)
        controls = QtWidgets.QVBoxLayout()
        root.addLayout(controls, 0)

        controls.addWidget(self._header("Mode"))
        self.mode_label = QtWidgets.QLabel("Mapping (SLAM)")
        self.mode_label.setStyleSheet("font-weight: bold; color: #1a7f1a;")
        controls.addWidget(self.mode_label)

        map_btn = QtWidgets.QPushButton("Start / Restart Mapping (SLAM)")
        map_btn.clicked.connect(self.on_start_mapping)
        controls.addWidget(map_btn)

        save_btn = QtWidgets.QPushButton("Save current map")
        save_btn.clicked.connect(self.on_save)
        controls.addWidget(save_btn)

        controls.addSpacing(16)
        controls.addWidget(self._header("Localization"))
        row = QtWidgets.QHBoxLayout()
        self.combo = QtWidgets.QComboBox()
        self.combo.currentTextChanged.connect(self.on_select)
        row.addWidget(self.combo, 1)
        refresh = QtWidgets.QPushButton("↻")
        refresh.setFixedWidth(32)
        refresh.clicked.connect(self.refresh_maps)
        row.addWidget(refresh)
        controls.addLayout(row)

        loc_btn = QtWidgets.QPushButton("Localize with selected map")
        loc_btn.clicked.connect(self.on_localize)
        controls.addWidget(loc_btn)

        controls.addStretch(1)
        self.status = QtWidgets.QLabel("ready")
        self.status.setWordWrap(True)
        controls.addWidget(self.status)
        self.live_label = QtWidgets.QLabel("live map: 0 cones")
        controls.addWidget(self.live_label)

        self.fig = Figure(figsize=(5, 5))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasQTAgg(self.fig)
        root.addWidget(self.canvas, 1)

        self.map_status.connect(self.status.setText)
        self.refresh_maps()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(
            lambda: self.live_label.setText(f"live map: {self.node.live_cones} cones"))
        self.timer.start(500)

    def _header(self, text):
        label = QtWidgets.QLabel(text)
        label.setStyleSheet("font-weight: bold; font-size: 14px;")
        return label

    def refresh_maps(self):
        self.combo.blockSignals(True)
        self.combo.clear()
        maps = sorted(glob.glob(os.path.join(self.map_dir, "*.csv")))
        self.combo.addItems([os.path.basename(m) for m in maps])
        self.combo.blockSignals(False)
        if maps:
            self.on_select(self.combo.currentText())
        else:
            self.ax.clear()
            self.ax.set_title(f"no maps in {self.map_dir}")
            self.canvas.draw()

    def selected_path(self):
        name = self.combo.currentText()
        return os.path.join(self.map_dir, name) if name else None

    def on_select(self, _name):
        path = self.selected_path()
        if not path or not os.path.isfile(path):
            return
        try:
            cones, start = read_map_csv(path)
        except Exception as exc:  # noqa: BLE001
            self.status.setText(f"preview error: {exc}")
            return
        self.ax.clear()
        for tag, color in CONE_COLORS.items():
            xs = [c[1] for c in cones if c[0] == tag]
            ys = [c[2] for c in cones if c[0] == tag]
            if xs:
                self.ax.scatter(xs, ys, c=color, s=30, edgecolors="k",
                                linewidths=0.4, label=f"{tag} ({len(xs)})")
        if start:
            self.ax.scatter([start[0]], [start[1]], marker="*", s=220,
                            c="#00c000", edgecolors="k", label="start")
        self.ax.set_aspect("equal", "datalim")
        self.ax.set_title(f"{os.path.basename(path)} ({len(cones)} cones)")
        self.ax.legend(loc="best", fontsize=7)
        self.ax.grid(True, alpha=0.3)
        self.canvas.draw()

    def on_save(self):
        self.status.setText("saving map...")
        self.node.call_trigger(
            self.node.save_cli,
            lambda ok, msg: (self.map_status.emit(msg), self.refresh_maps()))

    def on_start_mapping(self):
        self.status.setText("switching to mapping...")

        def done(ok, msg):
            self.map_status.emit(msg)
            if ok:
                self.mode_label.setText("Mapping (SLAM)")
                self.mode_label.setStyleSheet("font-weight: bold; color: #1a7f1a;")
        self.node.call_trigger(self.node.map_cli, done)

    def on_localize(self):
        path = self.selected_path()
        if not path:
            self.status.setText("select a map first")
            return
        self.status.setText(f"loading {os.path.basename(path)}...")

        def after_param(ok, _msg):
            def loaded(ok2, msg2):
                self.map_status.emit(msg2)
                if ok2:
                    self.mode_label.setText("Localization")
                    self.mode_label.setStyleSheet(
                        "font-weight: bold; color: #1a4fff;")
            self.node.call_trigger(self.node.load_cli, loaded)
        self.node.set_load_path(path, after_param)


def build_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--slam-ns", default="/graph_slam")
    parser.add_argument("--map-topic", default="/localization/cone_map")
    parser.add_argument("--map-dir", default="")
    return parser


def main():
    parser = build_arg_parser()
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    args = parser.parse_args(argv[1:])
    map_dir = args.map_dir or os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "..", "map")
    map_dir = os.path.normpath(map_dir)

    rclpy.init()
    node = GuiNode(args.slam_ns, args.map_topic)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(node, map_dir)
    win.show()
    rc = app.exec_()

    rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
