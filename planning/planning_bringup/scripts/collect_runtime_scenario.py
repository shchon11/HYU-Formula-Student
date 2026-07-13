#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
#
# How to run:
#   python3 collect_runtime_scenario.py --scenario NAME --evidence DIR --duration-sec 3

from __future__ import annotations

import argparse, json, re, shutil, subprocess, sys, time, uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Final

Json = None | bool | int | float | str | list["Json"] | dict[str, "Json"]
TF_ECHO_TIMEOUT_SEC: Final = 5.0
TOPIC_LIST_TIMEOUT_SEC: Final = 5.0
REQUIRED_TOPIC_POLL_SEC: Final = 1.0


@dataclass(frozen=True, slots=True)
class NamedTopic:
    topic: str; output: str


@dataclass(frozen=True, slots=True)
class TfEcho:
    target: str; source: str; output: str


@dataclass(frozen=True, slots=True)
class TfOwner:
    topic: str; expected: str; output: str


@dataclass(frozen=True, slots=True)
class Request:
    scenario: str; evidence: Path; duration_sec: float; require_topics: tuple[NamedTopic, ...]
    history_topics: tuple[NamedTopic, ...]; info_topics: tuple[NamedTopic, ...]; echo_topics: tuple[NamedTopic, ...]
    tf_echoes: tuple[TfEcho, ...]; tf_owners: tuple[TfOwner, ...]; forbid_topics: tuple[NamedTopic, ...]
    merge_reasons: tuple[Path, ...]; expect_conflicts: tuple[str, ...]
    min_laps: int | None; summary: Path
    assertions: tuple[str, ...]; classify_baseline: bool
    expect_classification: str; allow_classifications: tuple[str, ...]


def clean(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "-", value.strip("/").replace("/", "__")).strip("-") or "root"


def topic_arg(value: str, prefix: str, suffix: str) -> NamedTopic:
    topic, sep, output = value.partition(":")
    return NamedTopic(topic=topic, output=output if sep else f"{prefix}{clean(topic)}{suffix}")


def tf_echo_arg(values: list[str]) -> TfEcho:
    raw = ":".join(values)
    parts = raw.split(":")
    if len(parts) not in (2, 3) or not parts[0] or not parts[1]:
        raise argparse.ArgumentTypeError("--tf-echo expects target:source[:output] or target source")
    return TfEcho(parts[0], parts[1], parts[2] if len(parts) == 3 else f"tf-echo-{clean(parts[0])}-{clean(parts[1])}.txt")


def tf_owner_arg(value: str) -> TfOwner:
    parts = value.split(":")
    if len(parts) == 1:
        return TfOwner("/tf", parts[0], "tf-owner.txt")
    if len(parts) == 2:
        topic = parts[0] if parts[0].startswith("/") else "/tf"
        expected = parts[1] if parts[0].startswith("/") else parts[0]
        return TfOwner(topic, expected, parts[1] if parts[0].startswith("/") else "tf-owner.txt")
    topic = parts[0] if parts[0].startswith("/") else "/tf"
    expected = parts[1] if parts[0].startswith("/") else parts[0]
    return TfOwner(topic, expected, parts[2])


def parse_args(argv: list[str]) -> Request:
    parser = argparse.ArgumentParser(description="Collect bounded ROS runtime evidence.")
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--duration-sec", required=True, type=float)
    parser.add_argument("--require-topic", action="append", default=[])
    parser.add_argument("--history-topic", action="append", default=[])
    parser.add_argument("--info-topic", action="append", default=[])
    parser.add_argument("--echo-once", action="append", default=[])
    parser.add_argument("--tf-echo", nargs="+", action="append", default=[])
    parser.add_argument("--tf-owner", action="append", default=[])
    parser.add_argument("--forbid-topic", action="append", default=[])
    parser.add_argument("--merge-reasons", action="append", default=[])
    parser.add_argument("--expect-conflict", action="append", default=[])
    parser.add_argument("--min-laps", type=int)
    parser.add_argument("--summary", default="summary.json")
    parser.add_argument("--assert", dest="assertions", action="append", default=[])
    parser.add_argument("--classify-baseline", action="store_true")
    parser.add_argument("--expect-classification", default="")
    parser.add_argument("--allow-classification", action="append", default=[])
    args = parser.parse_args(argv)
    if args.duration_sec <= 0.0: parser.error("--duration-sec must be > 0")
    evidence = Path(args.evidence)
    summary_arg = Path(args.summary)
    return Request(
        scenario=args.scenario,
        evidence=evidence,
        duration_sec=args.duration_sec,
        require_topics=tuple(topic_arg(v, "", ".required.txt") for v in args.require_topic),
        history_topics=tuple(topic_arg(v, "history-", ".txt") for v in args.history_topic),
        info_topics=tuple(topic_arg(v, "topic-info-", ".txt") for v in args.info_topic),
        echo_topics=tuple(topic_arg(v, "topic-echo-", ".txt") for v in args.echo_once),
        tf_echoes=tuple(tf_echo_arg(v) for v in args.tf_echo),
        tf_owners=tuple(tf_owner_arg(v) for v in args.tf_owner),
        forbid_topics=tuple(topic_arg(v, "forbid-", ".txt") for v in args.forbid_topic),
        merge_reasons=tuple(Path(v) for v in args.merge_reasons),
        expect_conflicts=tuple(args.expect_conflict),
        min_laps=args.min_laps,
        summary=summary_arg if summary_arg.is_absolute() else evidence / summary_arg,
        assertions=tuple(args.assertions),
        classify_baseline=args.classify_baseline,
        expect_classification=args.expect_classification,
        allow_classifications=tuple(args.allow_classification),
    )


class Runner:
    def __init__(self, deadline: float) -> None:
        self.ros2 = shutil.which("ros2")
        self.deadline = deadline

    def run(self, args: list[str], output: Path, timeout_cap_sec: float | None = None) -> tuple[int, bool]:
        remaining = self.deadline - time.monotonic()
        if remaining <= 0.0:
            output.write_text("TIMEOUT before command start\n", encoding="utf-8")
            return 124, True
        timeout_sec = min(remaining, timeout_cap_sec) if timeout_cap_sec is not None else remaining
        if self.ros2 is None:
            output.write_text("COMMAND NOT FOUND: ros2\n", encoding="utf-8")
            return 127, False
        try:
            with output.open("w", encoding="utf-8") as handle:
                done = subprocess.run([self.ros2, *args], stdout=handle, stderr=subprocess.STDOUT, text=True, timeout=timeout_sec, check=False)
            return done.returncode, False
        except subprocess.TimeoutExpired:
            with output.open("a", encoding="utf-8") as handle:
                handle.write(f"\nTIMEOUT after {timeout_sec:.3f}s: {' '.join(args)}\n")
            return 124, True


def read_topics(path: Path) -> set[str]:
    return {line.strip() for line in path.read_text(encoding="utf-8", errors="replace").splitlines() if line.strip()} if path.exists() else set()


def wait_for_required_topics(runner: Runner, required_topics: tuple[NamedTopic, ...], output: Path) -> tuple[int, bool, set[str]]:
    code = 1
    timed = False
    topics: set[str] = set()
    required = {item.topic for item in required_topics}
    while time.monotonic() < runner.deadline:
        code, timed = runner.run(["topic", "list"], output, TOPIC_LIST_TIMEOUT_SEC)
        topics = read_topics(output)
        if code == 0 and required.issubset(topics):
            return code, timed, topics
        if not required:
            return code, timed, topics
        time.sleep(min(REQUIRED_TOPIC_POLL_SEC, max(0.0, runner.deadline - time.monotonic())))
    return code, timed, topics


def publisher_count(path: Path) -> int:
    match = re.search(r"Publisher count:\s*(\d+)", path.read_text(encoding="utf-8", errors="replace")) if path.exists() else None
    return int(match.group(1)) if match else 0


def tf_echo_has_transform(path: Path) -> bool:
    text = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
    return "- Translation:" in text and "- Rotation:" in text


def merge_reasons(req: Request, artifacts: list[str]) -> tuple[list[str], list[str]]:
    merged: list[str] = []
    missing: list[str] = []
    output = req.evidence / "path-reasons.log"
    with output.open("w", encoding="utf-8") as out:
        for path in req.merge_reasons:
            source = path if path.is_absolute() or path.exists() else req.evidence / path
            if source.exists():
                out.write(f"### {source}\n{source.read_text(encoding='utf-8', errors='replace')}\n")
                merged.append(str(source))
            else:
                missing.append(str(source))
        if not merged:
            out.write("NO REASON LOGS MERGED\n")
    artifacts.append(str(output))
    return merged, missing


def collect(req: Request) -> dict[str, Json]:
    req.evidence.mkdir(parents=True, exist_ok=True)
    runner = Runner(time.monotonic() + req.duration_sec)
    artifacts: list[str] = []
    failures: list[str] = []
    timeouts: list[str] = []
    topic_list = req.evidence / "topic-list.txt"
    code, timed, topics = wait_for_required_topics(runner, req.require_topics, topic_list)
    artifacts.append(str(topic_list))
    if code:
        failures.append("topic-list")
    if timed:
        timeouts.append("topic-list")
    for item in req.require_topics:
        info = req.evidence / item.output
        echo = req.evidence / f"{Path(item.output).stem}.echo.txt"
        for command, path in ((["topic", "info", item.topic], info), (["topic", "echo", "--once", item.topic], echo)):
            code, timed = runner.run(command, path)
            artifacts.append(str(path))
            failures.extend([item.topic] if code else [])
            timeouts.extend([item.topic] if timed else [])
    for item in req.history_topics:
        info = req.evidence / f"{Path(item.output).stem}.info.txt"
        hist = req.evidence / item.output
        for command, path in ((["topic", "info", "-v", item.topic], info), (["topic", "echo", "--once", item.topic], hist)):
            code, timed = runner.run(command, path)
            artifacts.append(str(path))
            failures.extend([item.topic] if code else [])
            timeouts.extend([item.topic] if timed else [])
    for item in (*req.info_topics, *req.echo_topics):
        command = ["topic", "info", "-v", item.topic] if item in req.info_topics else ["topic", "echo", "--once", item.topic]
        path = req.evidence / item.output
        code, timed = runner.run(command, path)
        artifacts.append(str(path))
        failures.extend([item.topic] if code else [])
        timeouts.extend([item.topic] if timed else [])
    failed_tf: list[str] = []
    for item in req.tf_echoes:
        path = req.evidence / item.output
        code, timed = runner.run(["run", "tf2_ros", "tf2_echo", item.target, item.source], path, TF_ECHO_TIMEOUT_SEC)
        artifacts.append(str(path))
        has_transform = tf_echo_has_transform(path)
        failed_tf.extend([f"{item.target}:{item.source}"] if code and not has_transform else [])
        timeouts.extend([item.output] if timed and not has_transform else [])
    tf_owner_failed: list[str] = []
    tf_owner_observed: list[str] = []
    for item in req.tf_owners:
        path = req.evidence / item.output
        code, timed = runner.run(["topic", "info", "-v", item.topic], path)
        artifacts.append(str(path))
        text = path.read_text(encoding="utf-8", errors="replace").lower() if path.exists() else ""
        tf_owner_observed.extend(re.findall(r"node name:\s*([^\s]+)", text))
        tf_owner_failed.extend([item.expected] if code or item.expected.lower() not in text else [])
        timeouts.extend([item.output] if timed else [])
    forbidden_live: list[str] = []
    forbidden_absent: list[str] = []
    for item in req.forbid_topics:
        path = req.evidence / item.output
        code, timed = runner.run(["topic", "info", "-v", item.topic], path)
        artifacts.append(str(path))
        forbidden_absent.extend([item.topic] if code != 0 else [])
        forbidden_live.extend([item.topic] if code == 0 and publisher_count(path) > 0 else [])
        timeouts.extend([item.topic] if timed else [])
    merged, missing_reason_paths = merge_reasons(req, artifacts)
    missing_topics = [item.topic for item in req.require_topics if item.topic not in topics]
    manifest: dict[str, Json] = {
        "run_id": uuid.uuid4().hex,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "scenario": req.scenario,
        "duration_sec": req.duration_sec,
        "ros2_available": runner.ros2 is not None,
        "artifacts": artifacts,
        "required_topics": [v.topic for v in req.require_topics],
        "missing_topics": missing_topics,
        "missing_required_topic": bool(missing_topics),
        "history_topics": [v.topic for v in req.history_topics],
        "info_topics": [v.topic for v in req.info_topics],
        "echo_once_topics": [v.topic for v in req.echo_topics],
        "tf_echoes": [f"{v.target}:{v.source}" for v in req.tf_echoes],
        "failed_tf_echoes": failed_tf,
        "tf_owner_requests": [v.expected for v in req.tf_owners],
        "tf_owner_observed": tf_owner_observed,
        "tf_owner_failed": tf_owner_failed,
        "forbidden_topics_live": forbidden_live,
        "forbidden_topics_absent": forbidden_absent,
        "expect_conflicts": list(req.expect_conflicts),
        "min_laps": req.min_laps,
        "merged_reason_logs": merged,
        "missing_reason_paths": missing_reason_paths,
        "command_failures": len(failures),
        "failed_requests": failures,
        "runtime_timeout": bool(timeouts),
        "timeout_requests": timeouts,
        "classify_baseline": req.classify_baseline,
        "expect_classification": req.expect_classification,
        "allow_classifications": list(req.allow_classifications),
    }
    with (req.evidence / "runtime-manifest.json").open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return manifest


def summarize(req: Request) -> int:
    command = [sys.executable, str(Path(__file__).with_name("summarize_runtime_scenario.py")), "--scenario", req.scenario, "--evidence", str(req.evidence), "--write", str(req.summary)]
    for assertion in req.assertions:
        command.extend(["--assert", assertion])
    return subprocess.run(command, check=False).returncode


def main(argv: list[str]) -> int:
    req = parse_args(argv)
    manifest = collect(req)
    summary_code = summarize(req)
    if req.classify_baseline or req.expect_classification or req.allow_classifications:
        return summary_code
    strict_fail = manifest["missing_required_topic"] or manifest["failed_tf_echoes"] or manifest["tf_owner_failed"] or manifest["forbidden_topics_live"] or manifest["runtime_timeout"] or manifest["missing_reason_paths"] or manifest["command_failures"]
    return summary_code if summary_code else int(bool(strict_fail))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
