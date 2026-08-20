#!/usr/bin/env python3
"""Browser front end for the cone labeler, for teammates off the local network.

Same dataset layout and same YOLO files as cone_labeler.py -- this only changes
who can reach it. Several people label at once: the server records who reviewed
each frame and hands out held frames to nobody else.

    python3 src/perception/scripts/cone_labeler_web.py datasets/0801_cones

Standard library only, so nothing needs installing on the Jetson. It binds to
127.0.0.1 by default: the only way in is a tunnel you start yourself (see
docs/remote_labeling.md), never a port sitting open on the campus network. Every
request must carry a token; one is generated at startup and printed with the URL
to hand out. --host 0.0.0.0 is accepted but warns, because that does put the
service on the local network.

Interoperates with the desktop labeler: both read and write .labeler_state.json,
so a frame reviewed in either tool counts as reviewed in the other, and
remap_labels.py keeps skipping it.
"""

from __future__ import annotations

import argparse
import gzip
import json
import mimetypes
import os
import re
import secrets
import shutil
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlencode, urlparse

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")
DEFAULT_CLASSES = ["BLUE", "ORANGE_BIG", "ORANGE", "UNDEFINED", "YELLOW"]
HOLD_TTL = 180.0          # a frame someone opened counts as theirs this long
MAX_BODY = 4 << 20
WEBUI = Path(__file__).resolve().parent / "webui"

_SAFE_USER = re.compile(r"[^A-Za-z0-9 _.-]")


def load_classes(root: Path) -> list[str]:
    path = root / "classes.txt"
    if path.exists():
        names = [ln.strip() for ln in path.read_text().splitlines() if ln.strip()]
        if names:
            return names
    return list(DEFAULT_CLASSES)


def classes_of_text(text: str) -> tuple[int, list[int]]:
    seen, count = set(), 0
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        try:
            seen.add(int(float(parts[0])))
        except ValueError:
            continue
        count += 1
    return count, sorted(seen)


class Dataset:
    """Files, labels and review state, guarded by one lock across all clients."""

    def __init__(self, root: Path, images: Path, labels: Path):
        self.root = root
        self.images = images
        self.labels = labels
        self.trash = root / "_trash"
        self.classes = load_classes(root)
        self.state_path = root / ".labeler_state.json"
        self.lock = threading.RLock()

        self.files: list[str] = sorted(p.name for p in images.iterdir()
                                       if p.suffix.lower() in IMAGE_EXTS)
        if not self.files:
            sys.exit(f"no images in {images}")
        self.pos = {n: i for i, n in enumerate(self.files)}

        self.reviewed: dict[str, str] = {}
        self.extra_state: dict = {}
        if self.state_path.exists():
            try:
                st = json.loads(self.state_path.read_text())
                by = st.get("by", {})
                self.reviewed = {n: by.get(n, "") for n in st.get("reviewed", [])}
                self.extra_state = {k: v for k, v in st.items()
                                    if k not in ("reviewed", "by", "classes")}
            except (json.JSONDecodeError, OSError):
                pass

        self.meta: dict[str, tuple[int, list[int]]] = {}
        for name in self.files:
            path = self.label_path(name)
            self.meta[name] = classes_of_text(path.read_text()) if path.exists() else (0, [])

        self.holds: dict[str, tuple[str, float]] = {}
        self.version = 0
        self.changed: dict[str, int] = {}

    # -- paths

    def label_path(self, name: str) -> Path:
        return self.labels / (Path(name).stem + ".txt")

    def known(self, name: str) -> bool:
        return name in self.pos

    # -- state

    def _touch(self, name: str) -> None:
        self.version += 1
        self.changed[name] = self.version

    def save_state(self) -> None:
        payload = dict(self.extra_state)
        payload["reviewed"] = sorted(self.reviewed)
        payload["by"] = {n: u for n, u in self.reviewed.items() if u}
        payload["classes"] = self.classes
        tmp = self.state_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(payload))
        os.replace(tmp, self.state_path)

    def entry(self, name: str) -> dict:
        count, cls = self.meta.get(name, (0, []))
        out = {"n": name, "b": count, "c": cls}
        if name in self.reviewed:
            out["r"] = self.reviewed[name] or True
        held = self.holds.get(name)
        if held and time.time() - held[1] < HOLD_TTL:
            out["h"] = held[0]
        return out

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "classes": self.classes,
                "root": str(self.root),
                "version": self.version,
                "files": [self.entry(n) for n in self.files],
            }

    def changes_since(self, since: int) -> dict:
        with self.lock:
            names = [n for n, v in self.changed.items() if v > since]
            out = []
            for n in names:
                out.append(self.entry(n) if self.known(n) else {"n": n, "gone": True})
            now = time.time()
            holds = {n: u for n, (u, ts) in self.holds.items()
                     if now - ts < HOLD_TTL and self.known(n)}
            return {"version": self.version, "files": out, "holds": holds}

    # -- label io

    def read_labels(self, name: str) -> str:
        path = self.label_path(name)
        return path.read_text() if path.exists() else ""

    def write_labels(self, name: str, boxes: list, user: str, reviewed: bool) -> dict:
        lines = []
        for b in boxes:
            try:
                cls = int(b[0])
                cx, cy, bw, bh = (float(v) for v in b[1:5])
            except (TypeError, ValueError, IndexError):
                continue
            if not 0 <= cls < len(self.classes) or bw <= 0 or bh <= 0:
                continue
            cx, cy = min(max(cx, 0.0), 1.0), min(max(cy, 0.0), 1.0)
            bw, bh = min(bw, 1.0), min(bh, 1.0)
            lines.append(f"{cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
        with self.lock:
            if not self.known(name):
                return {"ok": False, "error": "unknown frame"}
            self.label_path(name).write_text("\n".join(lines) + ("\n" if lines else ""))
            self.meta[name] = classes_of_text("\n".join(lines))
            if reviewed:
                self.reviewed[name] = user
                self.save_state()
            self._touch(name)
            return {"ok": True, "entry": self.entry(name)}

    def delete(self, name: str, user: str) -> dict:
        with self.lock:
            if not self.known(name):
                return {"ok": False, "error": "unknown frame"}
            img = self.images / name
            lbl = self.label_path(name)
            (self.trash / "images").mkdir(parents=True, exist_ok=True)
            (self.trash / "labels").mkdir(parents=True, exist_ok=True)
            try:
                shutil.move(str(img), str(self.trash / "images" / name))
                if lbl.exists():
                    shutil.move(str(lbl), str(self.trash / "labels" / lbl.name))
            except OSError as exc:
                return {"ok": False, "error": str(exc)}
            idx = self.pos.pop(name)
            self.files.pop(idx)
            for n in self.files[idx:]:
                self.pos[n] -= 1
            self.meta.pop(name, None)
            self.reviewed.pop(name, None)
            self.holds.pop(name, None)
            self.save_state()
            self._touch(name)
            return {"ok": True, "deleted": name, "by": user}

    def restore(self, name: str) -> dict:
        with self.lock:
            if self.known(name):
                return {"ok": False, "error": "already present"}
            src = self.trash / "images" / name
            if not src.exists():
                return {"ok": False, "error": "not in _trash"}
            try:
                shutil.move(str(src), str(self.images / name))
                tl = self.trash / "labels" / (Path(name).stem + ".txt")
                if tl.exists():
                    shutil.move(str(tl), str(self.label_path(name)))
            except OSError as exc:
                return {"ok": False, "error": str(exc)}
            self.files.append(name)
            self.files.sort()
            self.pos = {n: i for i, n in enumerate(self.files)}
            path = self.label_path(name)
            self.meta[name] = classes_of_text(path.read_text()) if path.exists() else (0, [])
            self._touch(name)
            return {"ok": True, "entry": self.entry(name), "index": self.pos[name]}

    # -- multi-user

    def hold(self, name: str, user: str) -> None:
        with self.lock:
            if self.known(name):
                self.holds[name] = (user, time.time())

    def claim(self, user: str, after: int = -1) -> dict:
        """Next unreviewed frame nobody else is sitting on."""
        with self.lock:
            now = time.time()
            order = list(range(after + 1, len(self.files))) + list(range(0, after + 1))
            for i in order:
                name = self.files[i]
                if name in self.reviewed:
                    continue
                held = self.holds.get(name)
                if held and held[0] != user and now - held[1] < HOLD_TTL:
                    continue
                self.holds[name] = (user, now)
                return {"ok": True, "name": name, "index": i}
            return {"ok": False, "error": "no unreviewed frame is free"}

    def stats(self) -> dict:
        with self.lock:
            per_class = {i: 0 for i in range(len(self.classes))}
            empty = 0
            for name in self.files:
                count, _ = self.meta.get(name, (0, []))
                if count == 0:
                    empty += 1
            for name in self.files:
                path = self.label_path(name)
                if not path.exists():
                    continue
                for line in path.read_text().splitlines():
                    parts = line.split()
                    if parts:
                        try:
                            c = int(float(parts[0]))
                        except ValueError:
                            continue
                        per_class[c] = per_class.get(c, 0) + 1
            by_user: dict[str, int] = {}
            for user in self.reviewed.values():
                by_user[user or "(unknown)"] = by_user.get(user or "(unknown)", 0) + 1
            return {"images": len(self.files), "reviewed": len(self.reviewed),
                    "empty": empty, "classes": self.classes,
                    "per_class": [per_class.get(i, 0) for i in range(len(self.classes))],
                    "by_user": by_user}


class Handler(BaseHTTPRequestHandler):
    server_version = "ConeLabeler"
    protocol_version = "HTTP/1.1"
    dataset: Dataset
    token: str

    def log_message(self, fmt, *args):  # quieter than the default access log
        if not self.path.startswith(("/api/image/", "/ui/")):
            sys.stderr.write("%s %s\n" % (self.address_string(), fmt % args))

    # -- plumbing

    def _send(self, code: int, body: bytes, ctype: str, headers: dict | None = None) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, obj, code: int = 200) -> None:
        body = json.dumps(obj).encode()
        headers = {"Cache-Control": "no-store"}
        if len(body) > 2048 and "gzip" in self.headers.get("Accept-Encoding", ""):
            body = gzip.compress(body, 5)
            headers["Content-Encoding"] = "gzip"
        self._send(code, body, "application/json", headers)

    def _body(self) -> dict:
        try:
            n = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            return {}
        if n <= 0 or n > MAX_BODY:
            return {}
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except (json.JSONDecodeError, UnicodeDecodeError):
            return {}

    def _cookies(self) -> dict:
        out = {}
        for part in self.headers.get("Cookie", "").split(";"):
            if "=" in part:
                k, v = part.split("=", 1)
                out[k.strip()] = v.strip()
        return out

    def _authorized(self, query: dict) -> bool:
        given = (query.get("t", [None])[0] or self._cookies().get("clt")
                 or self.headers.get("X-Labeler-Token") or "")
        return secrets.compare_digest(given, self.token)

    def _user(self, query: dict, body: dict | None = None) -> str:
        raw = (body or {}).get("user") or query.get("user", [""])[0] or ""
        return _SAFE_USER.sub("", unquote(raw))[:32] or "anon"

    # -- routes

    def do_GET(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        path = url.path

        if path == "/" and query.get("t"):
            # Trade the link token for a cookie so it stops riding in URLs. Any
            # other parameter (?user=) survives the redirect.
            if not self._authorized(query):
                return self._send(403, b"forbidden\n", "text/plain")
            rest = urlencode([(k, v) for k, vs in query.items() if k != "t" for v in vs])
            return self._send(303, b"", "text/plain", {
                "Location": "/?" + rest if rest else "/",
                "Set-Cookie": f"clt={self.token}; Path=/; HttpOnly; SameSite=Lax; Max-Age=2592000",
            })

        if not self._authorized(query):
            return self._send(403, b"forbidden: append ?t=<token> to the URL once\n", "text/plain")

        if path in ("/", "/index.html"):
            return self._file(WEBUI / "index.html", "text/html; charset=utf-8", cache=False)
        if path.startswith("/ui/"):
            rel = path[4:]
            if "/" in rel or ".." in rel:
                return self._send(404, b"not found\n", "text/plain")
            target = WEBUI / rel
            if not target.is_file():
                return self._send(404, b"not found\n", "text/plain")
            ctype = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
            return self._file(target, ctype, cache=False)

        if path == "/api/session":
            snap = self.dataset.snapshot()
            snap["user"] = self._user(query)
            return self._json(snap)
        if path == "/api/changes":
            try:
                since = int(query.get("v", ["0"])[0])
            except ValueError:
                since = 0
            user = self._user(query)
            name = query.get("hold", [""])[0]
            if name:
                self.dataset.hold(name, user)
            return self._json(self.dataset.changes_since(since))
        if path == "/api/stats":
            return self._json(self.dataset.stats())
        if path.startswith("/api/labels/"):
            name = unquote(path[len("/api/labels/"):])
            if not self.dataset.known(name):
                return self._json({"ok": False, "error": "unknown frame"}, 404)
            return self._json({"ok": True, "text": self.dataset.read_labels(name)})
        if path.startswith("/api/image/"):
            name = unquote(path[len("/api/image/"):])
            if not self.dataset.known(name):
                return self._send(404, b"not found\n", "text/plain")
            return self._file(self.dataset.images / name,
                              mimetypes.guess_type(name)[0] or "image/jpeg", cache=True)

        return self._send(404, b"not found\n", "text/plain")

    def do_POST(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        if not self._authorized(query):
            return self._send(403, b"forbidden\n", "text/plain")
        body = self._body()
        user = self._user(query, body)
        path = url.path

        if path == "/api/save":
            name = body.get("name", "")
            if not self.dataset.known(name):
                return self._json({"ok": False, "error": "unknown frame"}, 404)
            return self._json(self.dataset.write_labels(
                name, body.get("boxes", []), user, bool(body.get("reviewed"))))
        if path == "/api/delete":
            return self._json(self.dataset.delete(body.get("name", ""), user))
        if path == "/api/restore":
            return self._json(self.dataset.restore(body.get("name", "")))
        if path == "/api/claim":
            try:
                after = int(body.get("after", -1))
            except (TypeError, ValueError):
                after = -1
            return self._json(self.dataset.claim(user, after))
        return self._send(404, b"not found\n", "text/plain")

    def do_HEAD(self):
        self.do_GET()

    def _file(self, path: Path, ctype: str, cache: bool) -> None:
        try:
            data = path.read_bytes()
        except OSError:
            return self._send(404, b"not found\n", "text/plain")
        headers = {"Cache-Control": "private, max-age=86400" if cache else "no-store"}
        self._send(200, data, ctype, headers)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path, help="dataset root holding images/ and labels/")
    ap.add_argument("--labels", type=Path, default=None)
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--host", default="127.0.0.1",
                    help="keep the default unless you mean to expose the service directly")
    ap.add_argument("--token", default=None, help="shared access token (generated if omitted)")
    args = ap.parse_args()

    root = args.root
    images = root / "images" if (root / "images").is_dir() else root
    if images == root:
        root = root.parent
    labels = args.labels or (root / "labels")
    labels.mkdir(parents=True, exist_ok=True)
    if not (WEBUI / "index.html").exists():
        sys.exit(f"missing web assets at {WEBUI}")

    dataset = Dataset(root, images, labels)
    Handler.dataset = dataset
    Handler.token = args.token or secrets.token_urlsafe(18)

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.daemon_threads = True

    print(f"dataset  {root}  ({len(dataset.files)} images, {len(dataset.reviewed)} reviewed)")
    print(f"classes  {', '.join(dataset.classes)}")
    print(f"serving  http://{args.host}:{args.port}/?t={Handler.token}")
    if args.host != "127.0.0.1":
        print("WARNING: bound to a non-loopback address -- this service is now reachable\n"
              "         from the local network without going through your tunnel.")
    print("\nHand out the URL above (token included). Stop with Ctrl+C.")

    def on_term(_sig, _frame):
        # serve_labeler.sh stops us with SIGTERM; take the same exit as Ctrl+C so
        # the shutdown save runs. State is written on every edit regardless.
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        with dataset.lock:
            dataset.save_state()
        httpd.server_close()


if __name__ == "__main__":
    main()
