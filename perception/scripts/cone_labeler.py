#!/usr/bin/env python3
"""Cone bounding-box labeler -- YOLO format in, YOLO format out.

Built for fixing the 0801 misdetections (yellow cones coming out ORANGE) before
fine-tuning models/cone_detect_yolo26n. Point it at a dataset root produced by
extract_bag_images.py:

    python3 src/perception/scripts/cone_labeler.py datasets/0801_cones

Layout it expects (labels/ and classes.txt are created if missing):

    <root>/images/*.jpg       <root>/labels/*.txt       <root>/classes.txt

Labels are written as `<cls> <cx> <cy> <w> <h>`, normalized, one box per line --
the exact format `yolo detect train` consumes. Class indices follow classes.txt,
which extract_bag_images.py writes in the active checkpoint's own order.

Keys (H shows this in-window):

  navigate    D / Right = next        A / Left = previous
              Space = confirm + next  N = next unreviewed     G = go to index
  boxes       drag on empty area = draw box in the current class
              click = select   Ctrl+click = add to selection   Ctrl+A = all
              drag inside = move   drag a handle = resize   Tab = cycle
              1..9 = set class of the selected boxes (or arm the draw class)
              Backspace / right-click = delete selected box(es)
              Ctrl+C / Ctrl+V = copy / paste all boxes of a frame
              F = auto-label this frame with the current checkpoint
  image       Delete = send image to _trash/    Ctrl+Z = undo (boxes + deletes)
              S = save now (saving is automatic on every navigation)
  view        wheel or +/- = zoom    middle-drag = pan    0 = fit    L = labels
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QComboBox,
    QDockWidget,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

DEFAULT_CLASSES = ["BLUE", "ORANGE_BIG", "ORANGE", "UNDEFINED", "YELLOW"]

CLASS_COLORS = {
    "BLUE": "#2979ff",
    "ORANGE_BIG": "#ff3d00",
    "ORANGE": "#ff9800",
    "UNDEFINED": "#b0bec5",
    "YELLOW": "#ffea00",
}
FALLBACK_COLORS = ["#00e676", "#e040fb", "#00bcd4", "#ff4081", "#cddc39"]

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")
MIN_BOX_PX = 3.0
HANDLE_PX = 8.0
UNDO_LIMIT = 200

DEFAULT_MODEL = "src/perception/models/cone_detect_yolo26n/weights/best.pt"

HANDLES = ("tl", "t", "tr", "r", "br", "b", "bl", "l")


def help_lines() -> list[str]:
    """The key table out of the module docstring, so there is one copy of it."""
    parts = (__doc__ or "").split("in-window):", 1)
    return [ln.rstrip() for ln in parts[-1].splitlines() if ln.strip()] if len(parts) > 1 else []


# --------------------------------------------------------------------------- IO


def load_classes(root: Path, override: list[str] | None) -> list[str]:
    if override:
        return override
    path = root / "classes.txt"
    if path.exists():
        names = [ln.strip() for ln in path.read_text().splitlines() if ln.strip()]
        if names:
            return names
    path.write_text("\n".join(DEFAULT_CLASSES) + "\n")
    return list(DEFAULT_CLASSES)


def class_color(name: str, idx: int) -> QColor:
    return QColor(CLASS_COLORS.get(name.upper(), FALLBACK_COLORS[idx % len(FALLBACK_COLORS)]))


def read_yolo(path: Path, w: int, h: int) -> list[list]:
    boxes: list[list] = []
    if not path.exists():
        return boxes
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        try:
            cls = int(float(parts[0]))
            cx, cy, bw, bh = (float(v) for v in parts[1:5])
        except ValueError:
            continue
        boxes.append([cls, (cx - bw / 2) * w, (cy - bh / 2) * h,
                      (cx + bw / 2) * w, (cy + bh / 2) * h])
    return boxes


def write_yolo(path: Path, boxes: list[list], w: int, h: int) -> None:
    lines = []
    for cls, x1, y1, x2, y2 in boxes:
        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))
        x1, x2 = max(0.0, x1), min(float(w), x2)
        y1, y2 = max(0.0, y1), min(float(h), y2)
        bw, bh = x2 - x1, y2 - y1
        if bw < 1.0 or bh < 1.0:
            continue
        lines.append(f"{int(cls)} {(x1 + bw / 2) / w:.6f} {(y1 + bh / 2) / h:.6f} "
                     f"{bw / w:.6f} {bh / h:.6f}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + ("\n" if lines else ""))


def classes_in(path: Path) -> frozenset:
    if not path.exists():
        return frozenset()
    out = set()
    for line in path.read_text().splitlines():
        parts = line.split()
        if parts:
            try:
                out.add(int(float(parts[0])))
            except ValueError:
                pass
    return frozenset(out)


# ------------------------------------------------------------------------ canvas


class Canvas(QWidget):
    """Image view with draw / select / move / resize on normalized YOLO boxes."""

    def __init__(self, win: "Labeler"):
        super().__init__()
        self.win = win
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMinimumSize(640, 400)

        self.qimg: QImage | None = None
        self.iw = self.ih = 0
        self.boxes: list[list] = []
        self.selection: set[int] = set()

        self.scale = 1.0
        self.off = QPointF(0, 0)
        self.show_labels = True
        self.show_help = False
        self._auto_fit = True  # cleared once the user zooms or pans

        self._mode: str | None = None
        self._anchor = QPointF()
        self._handle: str | None = None
        self._draw: list[float] | None = None
        self._start_boxes: list[list] | None = None

    # -- coordinates

    def to_image(self, p) -> QPointF:
        return QPointF((p.x() - self.off.x()) / self.scale, (p.y() - self.off.y()) / self.scale)

    def to_screen(self, x: float, y: float) -> QPointF:
        return QPointF(x * self.scale + self.off.x(), y * self.scale + self.off.y())

    def box_rect(self, b) -> QRectF:
        tl = self.to_screen(min(b[1], b[3]), min(b[2], b[4]))
        br = self.to_screen(max(b[1], b[3]), max(b[2], b[4]))
        return QRectF(tl, br)

    def fit(self) -> None:
        if not self.iw:
            return
        self.scale = min(self.width() / self.iw, self.height() / self.ih)
        self.off = QPointF((self.width() - self.iw * self.scale) / 2,
                           (self.height() - self.ih * self.scale) / 2)
        self._auto_fit = True
        self.update()

    def zoom_at(self, factor: float, pos) -> None:
        before = self.to_image(pos)
        self.scale = max(0.05, min(40.0, self.scale * factor))
        after = self.to_image(pos)
        self.off += (after - before) * self.scale
        self._auto_fit = False
        self.update()

    # -- content

    def set_image(self, qimg: QImage | None, boxes: list[list], keep_view: bool) -> None:
        first = self.qimg is None
        self.qimg = qimg
        self.iw = qimg.width() if qimg else 0
        self.ih = qimg.height() if qimg else 0
        self.boxes = boxes
        self.selection.clear()
        self._mode = None
        self._draw = None
        if first or not keep_view:
            self.fit()
        self.update()

    def resizeEvent(self, ev):
        # The first fit() runs at the widget's minimum size, before the window is
        # shown; refit until the user takes the view over with a zoom or a pan.
        if self.qimg is not None and self._auto_fit:
            self.fit()
        super().resizeEvent(ev)

    # -- hit tests

    def hit_handle(self, pos) -> str | None:
        if len(self.selection) != 1:
            return None
        rect = self.box_rect(self.boxes[next(iter(self.selection))])
        pts = {
            "tl": rect.topLeft(), "tr": rect.topRight(),
            "bl": rect.bottomLeft(), "br": rect.bottomRight(),
            "t": QPointF(rect.center().x(), rect.top()),
            "b": QPointF(rect.center().x(), rect.bottom()),
            "l": QPointF(rect.left(), rect.center().y()),
            "r": QPointF(rect.right(), rect.center().y()),
        }
        for name in HANDLES:
            p = pts[name]
            if abs(p.x() - pos.x()) <= HANDLE_PX and abs(p.y() - pos.y()) <= HANDLE_PX:
                return name
        return None

    def hit_box(self, pos) -> int | None:
        """Topmost box under the cursor; smallest wins so nested cones stay reachable."""
        hits = [i for i, b in enumerate(self.boxes) if self.box_rect(b).adjusted(-2, -2, 2, 2).contains(pos)]
        if not hits:
            return None
        return min(hits, key=lambda i: self.box_rect(self.boxes[i]).width() * self.box_rect(self.boxes[i]).height())

    # -- mouse

    def mousePressEvent(self, ev):
        if self.qimg is None:
            return
        pos = ev.pos()
        if ev.button() == Qt.MiddleButton:
            self._mode, self._anchor = "pan", QPointF(pos)
            self.setCursor(Qt.ClosedHandCursor)
            return

        if ev.button() == Qt.RightButton:
            idx = self.hit_box(pos)
            if idx is not None:
                self.win.push_undo_boxes()
                self.boxes.pop(idx)
                self.selection = set()
                self.win.on_boxes_changed()
                self.update()
            return

        if ev.button() != Qt.LeftButton:
            return

        ctrl = bool(ev.modifiers() & Qt.ControlModifier)
        handle = None if ctrl else self.hit_handle(pos)
        if handle:
            self._mode, self._handle = "resize", handle
            self._anchor = self.to_image(pos)
            self.win.push_undo_boxes()
            return

        idx = self.hit_box(pos)
        if idx is not None:
            if ctrl:
                self.selection.symmetric_difference_update({idx})
                self.win.on_selection_changed()
                self.update()
                return
            if idx not in self.selection:
                self.selection = {idx}
                self.win.on_selection_changed()
            self._mode = "move"
            self._anchor = self.to_image(pos)
            self._start_boxes = [list(b) for b in self.boxes]
            self.win.push_undo_boxes()
            self.update()
            return

        self.selection = set()
        self.win.on_selection_changed()
        p = self.to_image(pos)
        self._mode = "draw"
        self._draw = [p.x(), p.y(), p.x(), p.y()]
        self.update()

    def mouseMoveEvent(self, ev):
        pos = ev.pos()
        if self._mode == "pan":
            self.off += QPointF(pos) - self._anchor
            self._anchor = QPointF(pos)
            self._auto_fit = False
            self.update()
            return
        if self._mode == "draw" and self._draw is not None:
            p = self.to_image(pos)
            self._draw[2], self._draw[3] = p.x(), p.y()
            self.update()
            return
        if self._mode == "move" and self._start_boxes is not None:
            p = self.to_image(pos)
            dx, dy = p.x() - self._anchor.x(), p.y() - self._anchor.y()
            for i in self.selection:
                s = self._start_boxes[i]
                self.boxes[i][1:5] = [s[1] + dx, s[2] + dy, s[3] + dx, s[4] + dy]
            self.update()
            return
        if self._mode == "resize" and len(self.selection) == 1:
            b = self.boxes[next(iter(self.selection))]
            p = self.to_image(pos)
            x1, y1, x2, y2 = b[1], b[2], b[3], b[4]
            if "l" in self._handle or self._handle in ("tl", "bl"):
                x1 = p.x()
            if "r" in self._handle or self._handle in ("tr", "br"):
                x2 = p.x()
            if self._handle in ("t", "tl", "tr"):
                y1 = p.y()
            if self._handle in ("b", "bl", "br"):
                y2 = p.y()
            b[1:5] = [x1, y1, x2, y2]
            self.update()
            return

        if self.qimg is not None:
            over = self.hit_handle(pos)
            if over:
                shape = {"tl": Qt.SizeFDiagCursor, "br": Qt.SizeFDiagCursor,
                         "tr": Qt.SizeBDiagCursor, "bl": Qt.SizeBDiagCursor,
                         "t": Qt.SizeVerCursor, "b": Qt.SizeVerCursor,
                         "l": Qt.SizeHorCursor, "r": Qt.SizeHorCursor}[over]
                self.setCursor(shape)
            elif self.hit_box(pos) is not None:
                self.setCursor(Qt.SizeAllCursor)
            else:
                self.setCursor(Qt.CrossCursor)
            self.win.show_cursor_pos(self.to_image(pos))

    def mouseReleaseEvent(self, ev):
        if self._mode == "draw" and self._draw is not None:
            x1, y1, x2, y2 = self._draw
            if abs(x2 - x1) >= MIN_BOX_PX and abs(y2 - y1) >= MIN_BOX_PX:
                self.win.push_undo_boxes()
                self.boxes.append([self.win.current_class,
                                   min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)])
                self.selection = {len(self.boxes) - 1}
                self.win.on_boxes_changed()
            self._draw = None
        elif self._mode in ("move", "resize"):
            for b in self.boxes:
                b[1], b[3] = sorted((b[1], b[3]))
                b[2], b[4] = sorted((b[2], b[4]))
            self.win.on_boxes_changed()
        self._mode = None
        self._start_boxes = None
        self.setCursor(Qt.CrossCursor)
        self.update()

    def wheelEvent(self, ev):
        if self.qimg is None:
            return
        self.zoom_at(1.15 if ev.angleDelta().y() > 0 else 1 / 1.15, ev.pos())

    def keyPressEvent(self, ev):
        self.win.handle_key(ev)

    # -- paint

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(24, 25, 28))
        if self.qimg is None:
            p.setPen(QColor(150, 150, 150))
            p.drawText(self.rect(), Qt.AlignCenter, "no images")
            return

        p.setRenderHint(QPainter.SmoothPixmapTransform, self.scale < 1.0)
        p.drawImage(QRectF(self.off.x(), self.off.y(), self.iw * self.scale, self.ih * self.scale),
                    self.qimg)

        font = QFont("DejaVu Sans", 8)
        font.setBold(True)
        p.setFont(font)

        for i, b in enumerate(self.boxes):
            color = self.win.color_of(int(b[0]))
            rect = self.box_rect(b)
            selected = i in self.selection
            p.setPen(QPen(color, 3 if selected else 2))
            p.drawRect(rect)
            if selected:
                p.setPen(QPen(QColor(255, 255, 255, 200), 1, Qt.DashLine))
                p.drawRect(rect.adjusted(-2, -2, 2, 2))
            if self.show_labels:
                text = self.win.name_of(int(b[0]))
                fm = p.fontMetrics()
                tw, th = fm.width(text) + 6, fm.height() + 2
                tag = QRectF(rect.left(), max(0.0, rect.top() - th), tw, th)
                p.fillRect(tag, QColor(color.red(), color.green(), color.blue(), 210))
                p.setPen(QColor(0, 0, 0) if color.lightness() > 130 else QColor(255, 255, 255))
                p.drawText(tag, Qt.AlignCenter, text)

        if len(self.selection) == 1:
            rect = self.box_rect(self.boxes[next(iter(self.selection))])
            p.setPen(QPen(QColor(255, 255, 255), 1))
            p.setBrush(QColor(20, 20, 20))
            for cx, cy in ((rect.left(), rect.top()), (rect.center().x(), rect.top()),
                           (rect.right(), rect.top()), (rect.right(), rect.center().y()),
                           (rect.right(), rect.bottom()), (rect.center().x(), rect.bottom()),
                           (rect.left(), rect.bottom()), (rect.left(), rect.center().y())):
                p.drawRect(QRectF(cx - HANDLE_PX / 2, cy - HANDLE_PX / 2, HANDLE_PX, HANDLE_PX))
            p.setBrush(Qt.NoBrush)

        if self._draw is not None:
            x1, y1, x2, y2 = self._draw
            tl, br = self.to_screen(min(x1, x2), min(y1, y2)), self.to_screen(max(x1, x2), max(y1, y2))
            p.setPen(QPen(self.win.color_of(self.win.current_class), 2, Qt.DashLine))
            p.drawRect(QRectF(tl, br))

        if self.show_help:
            self.paint_help(p)

    def paint_help(self, p: QPainter) -> None:
        lines = help_lines()
        if not lines:
            return
        font = QFont("DejaVu Sans Mono", 9)
        p.setFont(font)
        fm = p.fontMetrics()
        w = max(fm.width(ln) for ln in lines) + 28
        h = fm.height() * (len(lines) + 1) + 20
        box = QRectF(14, 14, w, h)
        p.fillRect(box, QColor(0, 0, 0, 215))
        p.setPen(QPen(QColor(120, 130, 140), 1))
        p.drawRect(box)
        p.setPen(QColor(235, 235, 235))
        y = box.top() + fm.height() + 4
        for ln in lines:
            p.drawText(int(box.left() + 14), int(y), ln.rstrip())
            y += fm.height()


# ------------------------------------------------------------------------ window


class Labeler(QMainWindow):
    def __init__(self, root: Path, images: Path, labels: Path,
                 classes: list[str], model_path: Path, conf: float):
        super().__init__()
        self.root = root
        self.images_dir = images
        self.labels_dir = labels
        self.classes = classes
        self.model_path = model_path
        self.conf = conf

        self.labels_dir.mkdir(parents=True, exist_ok=True)
        self.trash = root / "_trash"

        self.files = sorted(p for p in images.iterdir() if p.suffix.lower() in IMAGE_EXTS)
        if not self.files:
            sys.exit(f"no images in {images}")

        self.state_path = root / ".labeler_state.json"
        self.reviewed: set[str] = set()
        self.extra_state: dict = {}
        start_name = None
        if self.state_path.exists():
            try:
                st = json.loads(self.state_path.read_text())
                self.reviewed = set(st.get("reviewed", []))
                start_name = st.get("last")
                # cone_labeler_web.py records who reviewed each frame under "by".
                # Carry anything we do not own through untouched.
                self.extra_state = {k: v for k, v in st.items()
                                    if k not in ("reviewed", "last", "classes")}
            except (json.JSONDecodeError, OSError):
                pass

        self.cls_cache: dict[str, frozenset] = {}
        self.view: list[int] = list(range(len(self.files)))
        self.vpos = 0
        self.current_class = 0
        self.dirty = False
        self.clipboard: list[list] = []
        self.undo: list[tuple] = []
        self._model = None
        self._model_map: dict[int, int] = {}

        self._build_ui()
        if start_name:
            for i, p in enumerate(self.files):
                if p.name == start_name:
                    self.vpos = i
                    break
        self.rebuild_view(keep_current=True)
        self.load_current(keep_view=False)

    # -- ui

    def _build_ui(self) -> None:
        self.canvas = Canvas(self)
        self.setCentralWidget(self.canvas)

        dock = QDockWidget("frames", self)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        panel = QWidget()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(6, 6, 6, 6)

        self.filter = QComboBox()
        self.filter.setFocusPolicy(Qt.NoFocus)
        self.filter.addItems(["all", "unreviewed", "reviewed", "no boxes", "has boxes"]
                             + [f"has {c}" for c in self.classes])
        self.filter.currentIndexChanged.connect(lambda _: self.rebuild_view(keep_current=True))
        lay.addWidget(self.filter)

        self.list = QListWidget()
        self.list.setFocusPolicy(Qt.NoFocus)
        self.list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.list.setUniformItemSizes(True)
        self.list.itemClicked.connect(self._on_list_click)
        lay.addWidget(self.list, 1)

        self.legend = QLabel()
        self.legend.setTextFormat(Qt.RichText)
        lay.addWidget(self.legend)

        row = QHBoxLayout()
        for text, slot in (("stats", self.show_stats), ("help (H)", self.toggle_help)):
            btn = QPushButton(text)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.clicked.connect(slot)
            row.addWidget(btn)
        lay.addLayout(row)

        dock.setWidget(panel)
        dock.setMinimumWidth(250)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

        self.status = self.statusBar()
        self.msg = QLabel("")
        self.status.addPermanentWidget(self.msg)
        self.update_legend()
        self.resize(1500, 900)

    def update_legend(self) -> None:
        counts = {i: 0 for i in range(len(self.classes))}
        for b in self.canvas.boxes:
            counts[int(b[0])] = counts.get(int(b[0]), 0) + 1
        rows = []
        for i, name in enumerate(self.classes):
            c = self.color_of(i).name()
            mark = "&#9679;" if i == self.current_class else "&nbsp;"
            rows.append(f"<tr><td>{mark}</td><td><b>{i + 1}</b></td>"
                        f"<td style='color:{c}'>&#9632;</td>"
                        f"<td>{name}</td><td align='right'>{counts.get(i, 0)}</td></tr>")
        self.legend.setText("<table cellspacing=3>" + "".join(rows) + "</table>")

    # -- class helpers

    def name_of(self, idx: int) -> str:
        return self.classes[idx] if 0 <= idx < len(self.classes) else f"#{idx}"

    def color_of(self, idx: int) -> QColor:
        return class_color(self.name_of(idx), idx)

    # -- file/view helpers

    @property
    def current_file(self) -> Path | None:
        if not self.view:
            return None
        return self.files[self.view[self.vpos]]

    def label_path(self, img: Path) -> Path:
        return self.labels_dir / (img.stem + ".txt")

    def classes_of(self, img: Path) -> frozenset:
        if img.name not in self.cls_cache:
            self.cls_cache[img.name] = classes_in(self.label_path(img))
        return self.cls_cache[img.name]

    def passes_filter(self, img: Path) -> bool:
        mode = self.filter.currentText()
        if mode == "all":
            return True
        if mode == "unreviewed":
            return img.name not in self.reviewed
        if mode == "reviewed":
            return img.name in self.reviewed
        if mode == "no boxes":
            return not self.classes_of(img)
        if mode == "has boxes":
            return bool(self.classes_of(img))
        if mode.startswith("has "):
            want = mode[4:]
            return want in self.classes and self.classes.index(want) in self.classes_of(img)
        return True

    def rebuild_view(self, keep_current: bool) -> None:
        current = self.view[self.vpos] if (keep_current and self.view) else None
        self.view = [i for i, p in enumerate(self.files) if self.passes_filter(p)]
        if not self.view:
            self.vpos = 0
            self.list.clear()
            self.canvas.set_image(None, [], keep_view=True)
            self.update_status()
            return
        if current is not None:
            self.vpos = min(range(len(self.view)), key=lambda k: abs(self.view[k] - current))
        self.vpos = max(0, min(self.vpos, len(self.view) - 1))

        self.list.blockSignals(True)
        self.list.clear()
        self.list.addItems([self.list_text(self.files[i]) for i in self.view])
        self.list.setCurrentRow(self.vpos)
        self.list.blockSignals(False)
        self.list.scrollToItem(self.list.currentItem(), QAbstractItemView.PositionAtCenter)

    def list_text(self, img: Path) -> str:
        n = len(self.classes_of(img))
        if img.name in self.reviewed:
            mark = "OK"
        elif self.label_path(img).exists():
            mark = "--"
        else:
            mark = "  "
        return f"{mark} {img.name}" + (f"  [{n}]" if n else "")

    def _on_list_click(self, item) -> None:
        row = self.list.row(item)
        if row != self.vpos:
            self.commit()
            self.vpos = row
            self.load_current(keep_view=False)

    def refresh_list_row(self) -> None:
        if not self.view or self.vpos >= self.list.count():
            return
        img = self.current_file
        self.list.item(self.vpos).setText(self.list_text(img))

    # -- load / save

    def load_current(self, keep_view: bool) -> None:
        img = self.current_file
        if img is None:
            return
        qimg = QImage(str(img))
        if qimg.isNull():
            self.flash(f"cannot read {img.name}")
            self.canvas.set_image(None, [], keep_view=True)
            return
        qimg = qimg.convertToFormat(QImage.Format_RGB888)
        boxes = read_yolo(self.label_path(img), qimg.width(), qimg.height())
        self.canvas.set_image(qimg, boxes, keep_view)
        self.dirty = False
        self.list.blockSignals(True)
        self.list.setCurrentRow(self.vpos)
        self.list.blockSignals(False)
        self.list.scrollToItem(self.list.currentItem(), QAbstractItemView.EnsureVisible)
        self.setWindowTitle(f"cone labeler - {img.name} - {self.root}")
        self.update_legend()
        self.update_status()

    def commit(self, mark: bool = False) -> None:
        """Persist the current frame. Editing implies review; navigation alone does not."""
        img = self.current_file
        if img is None or self.canvas.qimg is None:
            return
        if self.dirty or mark:
            write_yolo(self.label_path(img), self.canvas.boxes, self.canvas.iw, self.canvas.ih)
            self.cls_cache[img.name] = frozenset(int(b[0]) for b in self.canvas.boxes)
            self.reviewed.add(img.name)
            self.dirty = False
            self.refresh_list_row()
            self.save_state()

    def save_state(self) -> None:
        img = self.current_file
        payload = dict(self.extra_state)
        payload.update({
            "reviewed": sorted(self.reviewed),
            "last": img.name if img else None,
            "classes": self.classes,
        })
        try:
            self.state_path.write_text(json.dumps(payload))
        except OSError as exc:
            self.flash(f"state not saved: {exc}")

    # -- callbacks from the canvas

    def push_undo_boxes(self) -> None:
        img = self.current_file
        if img is None:
            return
        self.undo.append(("boxes", img.name, [list(b) for b in self.canvas.boxes],
                          set(self.canvas.selection)))
        del self.undo[:-UNDO_LIMIT]

    def on_boxes_changed(self) -> None:
        self.dirty = True
        self.update_legend()
        self.update_status()

    def on_selection_changed(self) -> None:
        self.update_status()

    def show_cursor_pos(self, p: QPointF) -> None:
        if self.canvas.qimg is not None:
            self.msg.setText(f"{int(p.x())},{int(p.y())}  x{self.canvas.scale:.2f}")

    def flash(self, text: str) -> None:
        self.status.showMessage(text, 6000)

    def update_status(self) -> None:
        img = self.current_file
        if img is None:
            self.status.showMessage(f"no frames match filter '{self.filter.currentText()}'")
            return
        self.status.showMessage(
            f"{self.vpos + 1}/{len(self.view)} of {len(self.files)}   {img.name}   "
            f"boxes {len(self.canvas.boxes)} (sel {len(self.canvas.selection)})   "
            f"reviewed {len(self.reviewed)}   class {self.current_class + 1}:"
            f"{self.name_of(self.current_class)}" + ("   *unsaved" if self.dirty else "")
        )

    # -- actions

    def go(self, delta: int) -> None:
        if not self.view:
            return
        self.commit()
        self.vpos = max(0, min(len(self.view) - 1, self.vpos + delta))
        self.load_current(keep_view=True)

    def go_to(self, vpos: int) -> None:
        if not self.view:
            return
        self.commit()
        self.vpos = max(0, min(len(self.view) - 1, vpos))
        self.load_current(keep_view=True)

    def next_unreviewed(self) -> None:
        self.commit()
        for k in range(self.vpos + 1, len(self.view)):
            if self.files[self.view[k]].name not in self.reviewed:
                self.vpos = k
                self.load_current(keep_view=True)
                return
        self.flash("no unreviewed frame after this one")

    def set_class(self, idx: int) -> None:
        if idx >= len(self.classes):
            return
        self.current_class = idx
        if self.canvas.selection:
            # A freshly drawn box stays selected, so this also re-classes it --
            # say so, because it is the one keystroke that changes past work.
            self.push_undo_boxes()
            for i in self.canvas.selection:
                self.canvas.boxes[i][0] = idx
            self.on_boxes_changed()
            self.canvas.update()
            self.flash(f"{len(self.canvas.selection)} box(es) -> {self.name_of(idx)}  (Ctrl+Z undoes)")
        self.update_legend()
        self.update_status()

    def delete_selected(self) -> None:
        if not self.canvas.selection:
            self.flash("no box selected (Delete removes the image itself)")
            return
        self.push_undo_boxes()
        for i in sorted(self.canvas.selection, reverse=True):
            self.canvas.boxes.pop(i)
        self.canvas.selection = set()
        self.on_boxes_changed()
        self.canvas.update()

    def delete_image(self) -> None:
        img = self.current_file
        if img is None:
            return
        lbl = self.label_path(img)
        ti = self.trash / "images" / img.name
        tl = self.trash / "labels" / lbl.name
        ti.parent.mkdir(parents=True, exist_ok=True)
        tl.parent.mkdir(parents=True, exist_ok=True)
        file_idx = self.view[self.vpos]
        had_label = lbl.exists()
        try:
            shutil.move(str(img), str(ti))
            if had_label:
                shutil.move(str(lbl), str(tl))
        except OSError as exc:
            self.flash(f"delete failed: {exc}")
            return

        was_reviewed = img.name in self.reviewed
        self.reviewed.discard(img.name)
        self.cls_cache.pop(img.name, None)
        self.files.pop(file_idx)
        self.undo.append(("delete", file_idx, img, ti, tl if had_label else None, was_reviewed))
        del self.undo[:-UNDO_LIMIT]
        self.dirty = False

        self.rebuild_view(keep_current=False)
        self.vpos = min(self.vpos, max(0, len(self.view) - 1))
        if self.view:
            self.load_current(keep_view=True)
        else:
            self.canvas.set_image(None, [], keep_view=True)
            self.update_status()
        self.save_state()
        self.flash(f"{img.name} -> _trash  (Ctrl+Z restores)")

    def undo_last(self) -> None:
        if not self.undo:
            self.flash("nothing to undo")
            return
        entry = self.undo.pop()
        if entry[0] == "boxes":
            _, name, boxes, sel = entry
            if self.current_file is None or self.current_file.name != name:
                for k, i in enumerate(self.view):
                    if self.files[i].name == name:
                        self.vpos = k
                        self.load_current(keep_view=True)
                        break
                else:
                    self.flash(f"{name} is not in the current view")
                    return
            self.canvas.boxes = boxes
            self.canvas.selection = {i for i in sel if i < len(boxes)}
            self.dirty = True
            self.canvas.update()
            self.on_boxes_changed()
            self.flash("undo: boxes")
            return

        _, file_idx, img, trashed_img, trashed_lbl, was_reviewed = entry
        try:
            shutil.move(str(trashed_img), str(img))
            if trashed_lbl is not None:
                shutil.move(str(trashed_lbl), str(self.label_path(img)))
        except OSError as exc:
            self.flash(f"restore failed: {exc}")
            return
        self.files.insert(min(file_idx, len(self.files)), img)
        if was_reviewed:
            self.reviewed.add(img.name)
        self.cls_cache.pop(img.name, None)
        self.rebuild_view(keep_current=False)
        for k, i in enumerate(self.view):
            if self.files[i] == img:
                self.vpos = k
                break
        self.load_current(keep_view=True)
        self.save_state()
        self.flash(f"restored {img.name}")

    def copy_boxes(self) -> None:
        self.clipboard = [list(b) for b in self.canvas.boxes]
        self.flash(f"copied {len(self.clipboard)} box(es)")

    def paste_boxes(self) -> None:
        if not self.clipboard:
            self.flash("clipboard is empty")
            return
        self.push_undo_boxes()
        start = len(self.canvas.boxes)
        self.canvas.boxes.extend([list(b) for b in self.clipboard])
        self.canvas.selection = set(range(start, len(self.canvas.boxes)))
        self.on_boxes_changed()
        self.canvas.update()
        self.flash(f"pasted {len(self.clipboard)} box(es)")

    def autolabel(self) -> None:
        img = self.current_file
        if img is None:
            return
        if self._model is None:
            if not self.model_path.exists():
                self.flash(f"no checkpoint at {self.model_path}")
                return
            self.flash("loading checkpoint ...")
            QApplication.processEvents()
            try:
                from ultralytics import YOLO
            except ImportError as exc:
                self.flash(f"ultralytics not importable: {exc}")
                return
            self._model = YOLO(str(self.model_path))
            names = {i: str(n).upper() for i, n in self._model.names.items()}
            upper = [c.upper() for c in self.classes]
            self._model_map = {i: upper.index(n) for i, n in names.items() if n in upper}
            missing = [n for n in names.values() if n not in upper]
            if missing:
                self.flash(f"checkpoint classes not in classes.txt, ignored: {missing}")

        res = self._model.predict(source=str(img), imgsz=640, conf=self.conf, verbose=False)[0]
        new = []
        for xyxy, cls in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist()):
            mapped = self._model_map.get(int(cls))
            if mapped is None:
                continue
            new.append([mapped, xyxy[0], xyxy[1], xyxy[2], xyxy[3]])
        self.push_undo_boxes()
        self.canvas.boxes = new
        self.canvas.selection = set()
        self.on_boxes_changed()
        self.canvas.update()
        self.flash(f"auto-labeled {len(new)} box(es) at conf {self.conf} - fix and press Space")

    def show_stats(self) -> None:
        counts = {i: 0 for i in range(len(self.classes))}
        labeled = empty = 0
        for img in self.files:
            cs = classes_in(self.label_path(img))
            self.cls_cache[img.name] = cs
            if self.label_path(img).exists():
                labeled += 1
                if not cs:
                    empty += 1
            for line in (self.label_path(img).read_text().splitlines()
                         if self.label_path(img).exists() else []):
                parts = line.split()
                if parts:
                    try:
                        counts[int(float(parts[0]))] = counts.get(int(float(parts[0])), 0) + 1
                    except ValueError:
                        pass
        rows = "\n".join(f"  {i}  {n:<12} {counts.get(i, 0)}" for i, n in enumerate(self.classes))
        QMessageBox.information(self, "dataset stats",
                                f"images       {len(self.files)}\n"
                                f"label files  {labeled}\n"
                                f"empty labels {empty}\n"
                                f"reviewed     {len(self.reviewed)}\n\nboxes per class\n{rows}")

    def toggle_help(self) -> None:
        self.canvas.show_help = not self.canvas.show_help
        self.canvas.update()

    def prompt_goto(self) -> None:
        if not self.view:
            return
        n, ok = QInputDialog.getInt(self, "go to frame", f"index 1..{len(self.view)}",
                                    self.vpos + 1, 1, len(self.view))
        if ok:
            self.go_to(n - 1)

    # -- keys

    def handle_key(self, ev) -> None:
        key = ev.key()
        mods = ev.modifiers()
        ctrl = bool(mods & Qt.ControlModifier)

        if ctrl and key == Qt.Key_Z:
            self.undo_last()
        elif ctrl and key == Qt.Key_C:
            self.copy_boxes()
        elif ctrl and key == Qt.Key_V:
            self.paste_boxes()
        elif ctrl and key == Qt.Key_A:
            self.canvas.selection = set(range(len(self.canvas.boxes)))
            self.canvas.update()
            self.on_selection_changed()
        elif ctrl and key == Qt.Key_S:
            self.commit(mark=True)
            self.flash("saved")
        elif key in (Qt.Key_D, Qt.Key_Right):
            self.go(1)
        elif key in (Qt.Key_A, Qt.Key_Left):
            self.go(-1)
        elif key == Qt.Key_Space:
            self.commit(mark=True)
            self.go(1)
        elif key == Qt.Key_N:
            self.next_unreviewed()
        elif key == Qt.Key_G:
            self.prompt_goto()
        elif key == Qt.Key_S:
            self.commit(mark=True)
            self.flash("saved")
        elif key == Qt.Key_F:
            self.autolabel()
        elif key == Qt.Key_L:
            self.canvas.show_labels = not self.canvas.show_labels
            self.canvas.update()
        elif key == Qt.Key_H:
            self.toggle_help()
        elif key == Qt.Key_Tab:
            if self.canvas.boxes:
                cur = max(self.canvas.selection) if self.canvas.selection else -1
                self.canvas.selection = {(cur + 1) % len(self.canvas.boxes)}
                self.canvas.update()
                self.on_selection_changed()
        elif key == Qt.Key_Escape:
            self.canvas.selection = set()
            self.canvas.show_help = False
            self.canvas.update()
            self.on_selection_changed()
        elif key == Qt.Key_Backspace:
            self.delete_selected()
        elif key == Qt.Key_Delete:
            self.delete_image()
        elif key in (Qt.Key_Plus, Qt.Key_Equal):
            self.canvas.zoom_at(1.25, self.canvas.rect().center())
        elif key == Qt.Key_Minus:
            self.canvas.zoom_at(1 / 1.25, self.canvas.rect().center())
        elif key == Qt.Key_0:
            self.canvas.fit()
        elif Qt.Key_1 <= key <= Qt.Key_9:
            self.set_class(key - Qt.Key_1)
        else:
            return
        ev.accept()

    def keyPressEvent(self, ev):
        self.handle_key(ev)

    def closeEvent(self, ev):
        self.commit()
        self.save_state()
        super().closeEvent(ev)


# -------------------------------------------------------------------------- main


def main() -> None:
    ap = argparse.ArgumentParser(description="YOLO cone labeler",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("root", type=Path,
                    help="dataset root (holding images/ and labels/), or an images directory")
    ap.add_argument("--labels", type=Path, default=None, help="override the labels directory")
    ap.add_argument("--classes", nargs="+", default=None, help="override classes.txt")
    ap.add_argument("--model", type=Path, default=Path(DEFAULT_MODEL),
                    help="checkpoint used by the F auto-label key")
    ap.add_argument("--conf", type=float, default=0.25, help="auto-label confidence threshold")
    args = ap.parse_args()

    root = args.root
    if (root / "images").is_dir():
        images = root / "images"
    elif root.is_dir():
        images, root = root, root.parent
    else:
        sys.exit(f"no such directory: {root}")
    labels = args.labels or (root / "labels")

    app = QApplication(sys.argv)
    win = Labeler(root, images, labels, load_classes(root, args.classes),
                  args.model, args.conf)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
