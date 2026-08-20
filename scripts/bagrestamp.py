#!/usr/bin/env python3
"""bagrestamp.py -- copy a rosbag2 (sqlite3), keep chosen topics, re-time by header.stamp.

    bagrestamp <bag_dir> [-o <out_dir>] [--sensors | --topics REGEX] [--max-skew S]

rosbag2 stores one time per message: the instant the recorder RECEIVED it.
rqt_bag's timeline and `ros2 bag play` scheduling both use that column, so two
frames of one stereo grab (identical header.stamp) show up 10-20 ms apart, and
a slow publisher looks like a slow sensor. There is no "use header time" option
in Humble, so this rewrites the column instead.

What it does -- the source bag is only ever opened for reading:
  * builds a fresh <out>/<name>_N.db3 per source file with the same schema,
    containing only the topics that pass the filter (default: all).
  * every message whose type starts with a std_msgs/Header (or a bare
    builtin_interfaces/Time `stamp`) gets timestamp := header.stamp, read
    straight out of the CDR blob -- no deserialisation for the fast path.
    tf2_msgs/TFMessage uses transforms[0].header.stamp.
  * messages with no stamp, a zero stamp, or a stamp further than --max-skew
    seconds from the receive time keep their receive time. The last rule is
    what leaves /tf_static (latched, stamped at launch) where it is instead of
    stretching the timeline back to boot.
  * metadata.yaml (and the db's metadata table) are rewritten with the kept
    topics, their counts, and the recomputed start/duration.

--sensors is the preset for the driving pipeline: exactly what bag.sh records
of the camera and what bagplay.sh replays --
    /sensors/lidar/points
    /sensors/zed/{left,right}/color/rect/{image/compressed,camera_info}
    /sbg/{ekf_nav,ekf_euler,ekf_rot_accel_body,imu_data,gps_pos,gps_vel,gps_hdt,status,utc_time}
Everything else (raw/theora images, rgb duplicate, /localization/*,
/vehicle/*, /tf, bridge status) is regenerated live by bagplay anyway.
"""
import argparse
import glob
import os
import re
import shutil
import sqlite3
import struct
import sys

import yaml

MAX_SKEW_DEFAULT = 5.0
SENSORS_RE = (r"^/sensors/lidar/points$"
              r"|^/sensors/zed/(left|right)/color/rect/(image/compressed|camera_info)$"
              r"|^/sbg/(ekf_nav|ekf_euler|ekf_rot_accel_body|imu_data|gps_pos|gps_vel|gps_hdt|status|utc_time)$")
BATCH = 256


def load_type(name):
    """Return the Python message class for 'pkg/msg/Type', or None if unknown."""
    try:
        from rosidl_runtime_py.utilities import get_message
        return get_message(name)
    except Exception:  # noqa: BLE001 -- unknown / unsourced package
        return None


def stamp_reader(msg_cls):
    """Return f(blob) -> stamp_ns | None for this type, or None if it has no stamp."""
    if msg_cls is None:
        return None
    fields = msg_cls.get_fields_and_field_types()
    first = next(iter(fields.items()), (None, None))
    # fast path: Header or Time as the very first field sits at CDR offset 4
    if first[1] in ("std_msgs/Header", "builtin_interfaces/Time"):
        def read_first(blob):
            if len(blob) < 12:
                return None
            end = "<" if blob[1] == 1 else ">"
            sec, nsec = struct.unpack_from(end + "iI", blob, 4)
            return sec * 1_000_000_000 + nsec
        return read_first
    # slow path: deserialise and look for header / stamp / transforms[0].header
    if not any(t in ("std_msgs/Header", "builtin_interfaces/Time")
               or "geometry_msgs/TransformStamped" in t for t in fields.values()):
        return None
    from rclpy.serialization import deserialize_message

    def read_deep(blob):
        m = deserialize_message(blob, msg_cls)
        st = None
        if hasattr(m, "header"):
            st = m.header.stamp
        elif hasattr(m, "stamp"):
            st = m.stamp
        elif hasattr(m, "transforms") and m.transforms:
            st = m.transforms[0].header.stamp
        if st is None:
            return None
        return st.sec * 1_000_000_000 + st.nanosec
    return read_deep


def rebuild_db(src_path, dst_path, keep, max_skew_ns, report):
    """Write dst_path from src_path with only `keep(topic)` topics, restamped.

    Returns (min_ts, max_ts, {topic: count}) for this file.
    """
    src = sqlite3.connect(f"file:{src_path}?mode=ro", uri=True)
    dst = sqlite3.connect(dst_path)
    dst.execute("PRAGMA journal_mode=OFF")
    dst.execute("PRAGMA synchronous=OFF")
    for (sql,) in src.execute("SELECT sql FROM sqlite_master WHERE sql IS NOT NULL AND name!='sqlite_sequence'"):
        dst.execute(sql)
    if src.execute("SELECT 1 FROM sqlite_master WHERE name='schema'").fetchone():
        dst.executemany("INSERT INTO schema VALUES (?,?)", src.execute("SELECT * FROM schema"))

    topics = src.execute("SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics").fetchall()
    kept = [t for t in topics if keep(t[1])]
    dst.executemany("INSERT INTO topics VALUES (?,?,?,?,?)", kept)

    counts = {}
    for tid, name, typ, _, _ in kept:
        reader = stamp_reader(load_type(typ))
        stat = report.setdefault(name, {"n": 0, "restamped": 0, "zero": 0, "skew": 0, "nostamp": 0})
        cur = src.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY id", (tid,))
        n = 0
        while True:
            rows = cur.fetchmany(BATCH)
            if not rows:
                break
            out = []
            for recv, blob in rows:
                hdr = reader(blob) if reader else None
                if hdr is None:
                    stat["nostamp"] += 1
                    ts = recv
                elif hdr == 0:
                    stat["zero"] += 1
                    ts = recv
                elif abs(hdr - recv) > max_skew_ns:
                    stat["skew"] += 1
                    ts = recv
                else:
                    stat["restamped"] += 1
                    ts = hdr
                out.append((tid, ts, blob))
            dst.executemany("INSERT INTO messages (topic_id, timestamp, data) VALUES (?,?,?)", out)
            n += len(out)
        stat["n"] += n
        counts[name] = n
        dst.commit()
    lo, hi = dst.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages").fetchone()
    src.close()
    return dst, lo, hi, counts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="rosbag2 directory (sqlite3 storage)")
    ap.add_argument("-o", "--out", help="output directory (default: <bag>_hdr)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--sensors", action="store_true", help="keep only the driving-pipeline sensor topics (see above)")
    g.add_argument("--topics", metavar="REGEX", help="keep only topics whose name matches REGEX (re.search)")
    ap.add_argument("--max-skew", type=float, default=MAX_SKEW_DEFAULT,
                    help="keep receive time when |header - receive| exceeds this many seconds "
                         f"(default {MAX_SKEW_DEFAULT})")
    ap.add_argument("--force", action="store_true", help="overwrite the output directory if it exists")
    a = ap.parse_args()

    src_dir = os.path.abspath(a.bag.rstrip("/"))
    meta_src = os.path.join(src_dir, "metadata.yaml")
    if not os.path.isfile(meta_src):
        sys.exit(f"bagrestamp: {src_dir} has no metadata.yaml")
    dst_dir = os.path.abspath(a.out) if a.out else src_dir + "_hdr"
    if os.path.exists(dst_dir):
        if not a.force:
            sys.exit(f"bagrestamp: {dst_dir} exists (use --force to replace)")
        shutil.rmtree(dst_dir)
    if a.sensors:
        pat = re.compile(SENSORS_RE)
    elif a.topics:
        pat = re.compile(a.topics)
    else:
        pat = None
    keep = (lambda name: bool(pat.search(name))) if pat else (lambda name: True)

    dbs = sorted(glob.glob(os.path.join(src_dir, "*.db3")))
    if not dbs:
        sys.exit("bagrestamp: no *.db3 in bag (only sqlite3 storage is supported)")
    print(f"bagrestamp: {src_dir}\n         -> {dst_dir}" + (f"\n  topics: {pat.pattern}" if pat else ""))
    os.makedirs(dst_dir)

    with open(meta_src) as f:
        meta = yaml.safe_load(f)
    info = meta["rosbag2_bagfile_information"]

    report = {}
    per_file = {}
    total_counts = {}
    max_skew_ns = int(a.max_skew * 1e9)
    conns = []
    for db in dbs:
        base = os.path.basename(db)
        print(f"  {base} ...", flush=True)
        dst_conn, lo, hi, counts = rebuild_db(db, os.path.join(dst_dir, base), keep, max_skew_ns, report)
        conns.append(dst_conn)
        per_file[base] = (lo, hi)
        for k, v in counts.items():
            total_counts[k] = total_counts.get(k, 0) + v

    # metadata: kept topics with new counts, bag-level and per-file start/duration
    info["topics_with_message_count"] = [
        dict(t, message_count=total_counts.get(t["topic_metadata"]["name"], 0))
        for t in info["topics_with_message_count"] if keep(t["topic_metadata"]["name"])]
    info["message_count"] = sum(total_counts.values())
    los = [lo for lo, hi in per_file.values() if lo is not None]
    his = [hi for lo, hi in per_file.values() if hi is not None]
    if los:
        info["starting_time"]["nanoseconds_since_epoch"] = min(los)
        info["duration"]["nanoseconds"] = max(his) - min(los)
    for fentry in info.get("files", []):
        lo_hi = per_file.get(os.path.basename(fentry.get("path", "")))
        if lo_hi and lo_hi[0] is not None:
            fentry["starting_time"]["nanoseconds_since_epoch"] = lo_hi[0]
            fentry["duration"]["nanoseconds"] = lo_hi[1] - lo_hi[0]
    meta_text = yaml.safe_dump(meta, sort_keys=False)
    with open(os.path.join(dst_dir, "metadata.yaml"), "w") as f:
        f.write(meta_text)
    for c in conns:  # the db's own metadata copy, if this rosbag2 version keeps one
        if c.execute("SELECT 1 FROM sqlite_master WHERE name='metadata'").fetchone():
            c.execute("INSERT INTO metadata (metadata_version, metadata) VALUES (?,?)",
                      (info.get("version", 5), meta_text))
        c.commit()
        c.close()

    w = max(len(n) for n in report) if report else 10
    print(f"\n{'topic':{w}s} {'msgs':>6s} {'restamped':>9s} {'kept:zero':>9s} {'kept:skew':>9s} {'no-stamp':>8s}")
    for name in sorted(report):
        s = report[name]
        print(f"{name:{w}s} {s['n']:6d} {s['restamped']:9d} {s['zero']:9d} {s['skew']:9d} {s['nostamp']:8d}")
    print("\nkept:skew = header more than %.1f s from receive time (e.g. latched /tf_static)" % a.max_skew)


if __name__ == "__main__":
    main()
