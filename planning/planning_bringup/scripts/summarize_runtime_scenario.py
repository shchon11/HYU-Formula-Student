#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
#
# How to run:
#   python3 summarize_runtime_scenario.py --scenario NAME --evidence DIR --write OUT.json

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Final, TypeAlias

Json: TypeAlias = None | bool | int | float | str | list["Json"] | dict[str, "Json"]
BASELINE_FILES = ("git-status.txt", "map-sha256.txt", "package-versions.tsv", "colcon-list.txt", "launch-defaults-planning-bringup.txt", "launch-defaults-graph-slam.txt", "launch-defaults-global-planner-slam.txt", "topic-contracts-interfaces.txt", "topic-contracts-static-rg.txt", "test-surface-files.txt", "test-registration-surface.txt")
TOKENS = ("false_early_convergence", "invalid_stop_after_first_lap", "no_path", "invalid", "stale", "missing", "failed", "waiting")
BASELINE_ALLOWED_CLASSIFICATIONS: Final = frozenset((
    "topology_conflict",
    "false_early_convergence",
    "invalid_stop_after_first_lap",
    "no_path",
    "handoff_invalid_path",
    "min_laps_unmet",
    "missing_required_topic",
))


@dataclass(frozen=True, slots=True)
class Request:
    scenario: str
    evidence: Path
    write: Path
    assertions: tuple[str, ...]
    allow_missing_runtime: bool


def parse_args(argv: list[str]) -> Request:
    parser = argparse.ArgumentParser(description="Summarize a recorded FSK runtime scenario evidence directory.")
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--write", required=True)
    parser.add_argument("--assert", dest="assertions", action="append", default=[])
    parser.add_argument("--allow-missing-runtime", action="store_true")
    args = parser.parse_args(argv)
    return Request(args.scenario, Path(args.evidence), Path(args.write), tuple(args.assertions), args.allow_missing_runtime)


def manifest(evidence: Path) -> dict[str, Json]:
    path = evidence / "runtime-manifest.json"
    if not path.exists():
        return {}
    loaded = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise AssertionError(f"runtime manifest is not a JSON object: {path}")
    return loaded


def resolve_artifact_path(evidence: Path, raw: str) -> Path:
    path = Path(raw)
    if path.is_absolute():
        return path
    candidates = [Path.cwd() / path, evidence / path]
    candidates.extend(parent / path for parent in evidence.parents)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def manifest_artifact_paths(evidence: Path, data: dict[str, Json]) -> list[Path]:
    raw = data.get("artifacts", [])
    if not isinstance(raw, list):
        return []
    return [resolve_artifact_path(evidence, item) for item in raw if isinstance(item, str)]


def artifact_paths(evidence: Path, data: dict[str, Json]) -> list[Path]:
    return [path for path in manifest_artifact_paths(evidence, data) if path.is_file()]


def texts(paths: list[Path]) -> list[str]:
    return [path.read_text(encoding="utf-8", errors="replace") for path in paths]


def key_values(text: str) -> dict[str, str]:
    found: dict[str, str] = {}
    for key, value in re.findall(r"([A-Za-z0-9_.-]+)\s*[=:]\s*([A-Za-z0-9_.+-]+)", text):
        found[key] = value
    return found


def number_from(name: str, data: dict[str, Json], all_text: str) -> float | None:
    value = data.get(name)
    if isinstance(value, int | float):
        return float(value)
    match = re.search(rf"{re.escape(name)}\s*[=:]\s*([0-9]+(?:\.[0-9]+)?)", all_text)
    return float(match.group(1)) if match else None


def max_lap(all_text: str) -> int | None:
    values = [int(v) for v in re.findall(r"(?:lap_count|lap)\s*[=:]\s*(\d+)", all_text)]
    return max(values) if values else None


def lap_history_count(paths: list[Path]) -> int | None:
    values: list[int] = []
    for path in paths:
        if path.name != "lap-history.log":
            continue
        values.extend(
            int(value)
            for value in re.findall(r"data:\s*(\d+)", path.read_text(encoding="utf-8", errors="replace"))
        )
    return max(values) if values else None


def publisher_count(text: str) -> int:
    match = re.search(r"Publisher count:\s*(\d+)", text)
    return int(match.group(1)) if match else 0


def publisher_nodes(text: str) -> list[str]:
    publishers = text.split("Subscription count:", 1)[0]
    return re.findall(r"Node name:\s*([^\s]+)", publishers, flags=re.IGNORECASE)


def duplicate_publishers(paths: list[Path]) -> tuple[list[str], list[str]]:
    duplicate_topics: list[str] = []
    observed_conflicts: set[str] = set()
    for path in paths:
        if not (path.name.endswith("-publishers.txt") or path.name.startswith("topic-info-")):
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        nodes = publisher_nodes(text)
        repeated_nodes = {node for node in nodes if nodes.count(node) > 1}
        has_duplicate_count = publisher_count(text) > 1 and path.name == "graph-slam-status-publishers.txt"
        if repeated_nodes or has_duplicate_count:
            duplicate_topics.append(path.name)
        if "graph_slam" in repeated_nodes or (path.name == "graph-slam-status-publishers.txt" and publisher_count(text) > 1):
            observed_conflicts.add("duplicate_graph_slam")
        if path.name == "tf-publishers.txt" and "graph_slam" in repeated_nodes:
            observed_conflicts.add("tf_owner_conflict")
    return duplicate_topics, sorted(observed_conflicts)


def owners(all_text: str, data: dict[str, Json]) -> list[str]:
    observed = data.get("tf_owner_observed", [])
    if isinstance(observed, list) and observed:
        return [str(item) for item in observed]
    return re.findall(r"Node name:\s*([^\s]+)", all_text, flags=re.IGNORECASE)


def classify(data: dict[str, Json], all_text: str, duplicate_writers: bool, min_laps_unmet: bool) -> str:
    explicit = data.get("failure_branch")
    if isinstance(explicit, str) and explicit:
        return explicit
    lowered = all_text.lower()
    if duplicate_writers or data.get("forbidden_topics_live"):
        return "topology_conflict"
    if "false_early_convergence" in lowered or "false early convergence" in lowered:
        return "false_early_convergence"
    if "invalid_stop_after_first_lap" in lowered or "invalid stop after first lap" in lowered:
        return "invalid_stop_after_first_lap"
    if "no_path" in lowered or "no path" in lowered:
        return "no_path"
    if "invalid" in lowered:
        return "handoff_invalid_path"
    if min_laps_unmet:
        return "min_laps_unmet"
    if data.get("missing_required_topic"):
        return "missing_required_topic"
    return "expected_localization" if data else ""


def check_assert(actual: Json, expr: str) -> tuple[bool, str, str, str]:
    for op in ("<=", ">=", "=", "<", ">"):
        if op in expr:
            key, expected = (part.strip() for part in expr.split(op, 1))
            got = actual.get(key) if isinstance(actual, dict) else None
            got_text = "true" if got is True else "false" if got is False else "null" if got is None else str(got)
            if op == "=":
                return got_text.strip().lower() == expected.strip().lower(), key, op, expected
            try:
                left = float(got_text)
                right = float(expected)
            except ValueError:
                return False, key, op, expected
            return (left <= right if op == "<=" else left >= right if op == ">=" else left < right if op == "<" else left > right), key, op, expected
    raise AssertionError(f"bad assertion shape: {expr!r}")


def summarize(req: Request) -> dict[str, Json]:
    if not req.evidence.is_dir():
        raise FileNotFoundError(f"evidence directory missing: {req.evidence}")
    data = manifest(req.evidence)
    paths = artifact_paths(req.evidence, data)
    manifest_paths = manifest_artifact_paths(req.evidence, data)
    all_text = "\n".join(texts(paths))
    kv = key_values(all_text)
    missing_baseline = [name for name in BASELINE_FILES if not (req.evidence / name).is_file()]
    empty_baseline = [name for name in BASELINE_FILES if (req.evidence / name).is_file() and (req.evidence / name).stat().st_size == 0]
    missing_artifacts = [str(path) for path in manifest_paths if not path.is_file()]
    duplicate_topics, observed_conflicts = duplicate_publishers(paths)
    min_laps = data.get("min_laps")
    observed_laps = max_lap(all_text)
    if observed_laps is None:
        observed_laps = lap_history_count(paths)
    min_laps_unmet = isinstance(min_laps, int) and (observed_laps is None or observed_laps < min_laps)
    conflict_expected = bool(data.get("expect_conflicts", []))
    conflict_observed = bool(duplicate_topics or observed_conflicts or data.get("forbidden_topics_live") or "conflict" in all_text.lower())
    reason_text = "\n".join(path.read_text(encoding="utf-8", errors="replace") for path in paths if path.name == "path-reasons.log")
    reason_tokens = [token for token in TOKENS if token in reason_text.lower()]
    baseline_capture_complete = not missing_baseline and not empty_baseline and bool(data) and bool(manifest_paths) and not missing_artifacts
    summary: dict[str, Json] = {
        "scenario": req.scenario,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "evidence_dir": str(req.evidence),
        "summary_path": str(req.write),
        "baseline_capture_complete": baseline_capture_complete,
        "missing_baseline_files": missing_baseline,
        "empty_baseline_files": empty_baseline,
        "missing_runtime_artifacts": missing_artifacts,
        "runtime_manifest_present": bool(data),
        "runtime_artifacts_present": bool(paths),
        "allow_missing_runtime": req.allow_missing_runtime,
        "run_id": data.get("run_id", ""),
        "run_started_at": data.get("started_at", ""),
        "missing_required_topic": data.get("missing_required_topic", False) is True,
        "missing_history_topic": bool(data.get("failed_requests", [])),
        "failed_info_topic": bool([v for v in data.get("failed_requests", []) if isinstance(v, str) and v.startswith("/")]),
        "failed_echo_once_topic": bool(data.get("failed_requests", [])),
        "failed_tf_echo": bool(data.get("failed_tf_echoes", [])),
        "tf_owner": data.get("tf_owner_requests", []),
        "tf_owner_observed": owners(all_text, data),
        "tf_owner_failed": bool(data.get("tf_owner_failed", [])),
        "forbidden_topic_live": bool(data.get("forbidden_topics_live", [])),
        "forbidden_topics_live": data.get("forbidden_topics_live", []),
        "forbidden_topics_absent": data.get("forbidden_topics_absent", []),
        "duplicate_writers": bool(duplicate_topics),
        "duplicate_writer_topics": duplicate_topics,
        "observed_conflicts": observed_conflicts,
        "duplicate_graph_slam": "duplicate_graph_slam" in observed_conflicts,
        "tf_owner_conflict": "tf_owner_conflict" in observed_conflicts,
        "conflict_expected": conflict_expected,
        "conflict_observed": conflict_observed,
        "expect_conflict_unmet": conflict_expected and not conflict_observed,
        "lap_count": observed_laps,
        "min_laps": min_laps,
        "min_laps_unmet": False if req.allow_missing_runtime and not paths else min_laps_unmet,
        "lap_duration_sec": number_from("lap_duration_sec", data, all_text),
        "path_duration_sec": number_from("path_duration_sec", data, all_text),
        "false_early_convergence": "false_early_convergence" in all_text.lower() or "false early convergence" in all_text.lower(),
        "invalid_stop_after_first_lap": "invalid_stop_after_first_lap" in all_text.lower() or "invalid stop after first lap" in all_text.lower(),
        "reason_log_present": bool(reason_text),
        "reason_log_line_count": len(reason_text.splitlines()),
        "reason_tokens": reason_tokens,
        "runtime_timeout": data.get("runtime_timeout", False) is True,
    }
    for key, value in kv.items():
        summary.setdefault(key, value)
    branch = classify(data, all_text, bool(duplicate_topics), bool(summary["min_laps_unmet"]))
    summary["failure_branch"] = branch
    summary["primary_failure_branch"] = branch
    for key, value in data.items():
        summary[f"runtime_{key}"] = value
    expected_class = data.get("expect_classification", "")
    allowed = data.get("allow_classifications", [])
    baseline_mode = data.get("classify_baseline") is True or bool(expected_class) or bool(allowed)
    allowed_classes = set(BASELINE_ALLOWED_CLASSIFICATIONS) if baseline_mode else set()
    if isinstance(expected_class, str) and expected_class:
        allowed_classes.add(expected_class)
    if isinstance(allowed, list):
        allowed_classes.update(str(item) for item in allowed)
    classification_failed = isinstance(expected_class, str) and expected_class and branch != expected_class and branch not in allowed_classes
    results: list[Json] = []
    failed = classification_failed
    for assertion in req.assertions:
        passed, key, op, expected = check_assert(summary, assertion)
        failed = failed or not passed
        results.append({"key": key, "op": op, "expected": expected, "actual": summary.get(key), "passed": passed})
    summary["assertions"] = results
    summary["assertions_passed"] = not any(v.get("passed") is False for v in results if isinstance(v, dict))
    baseline_classification_accepted = baseline_mode and baseline_capture_complete and branch in allowed_classes and summary["assertions_passed"] is True
    summary["baseline_classification_mode"] = baseline_mode
    summary["baseline_allowed_classifications"] = sorted(allowed_classes)
    summary["baseline_classification_accepted"] = baseline_classification_accepted
    strict_ok = not failed and (req.allow_missing_runtime or bool(data) or bool(paths)) and not summary["missing_required_topic"] and not summary["failed_tf_echo"] and not summary["tf_owner_failed"] and not summary["forbidden_topic_live"] and not summary["expect_conflict_unmet"] and not summary["min_laps_unmet"] and not summary["runtime_timeout"]
    summary["summary_ok"] = baseline_classification_accepted or strict_ok
    return summary


def main(argv: list[str]) -> int:
    try:
        req = parse_args(argv)
        result = summarize(req)
        req.write.parent.mkdir(parents=True, exist_ok=True)
        req.write.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 0 if result["summary_ok"] is True else 1
    except (AssertionError, FileNotFoundError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
