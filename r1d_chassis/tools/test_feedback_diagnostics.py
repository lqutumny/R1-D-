#!/usr/bin/env python3
"""运行阶段 A 的正常联动与升降反馈中断矩阵，并核对结构化报告。"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
TOPICS = {
    "height": "rt/height_state",
    "velocity": "rt/state_vel",
    "alarm": "rt/height_alarm",
}
CASES = (
    {"name": "normal", "topics": (), "expected_code": 0},
    {"name": "stow_height_250ms", "topics": ("height",), "expected_code": 2},
    {"name": "stow_velocity_250ms", "topics": ("velocity",), "expected_code": 2},
    {"name": "stow_alarm_250ms", "topics": ("alarm",), "expected_code": 2},
    {"name": "stow_height_velocity_250ms", "topics": ("height", "velocity"),
     "expected_code": 2},
)


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def validate(case, code, report, sim):
    problems = []
    if code != case["expected_code"]:
        problems.append(f"exit={code}, expected={case['expected_code']}")
    health = report.get("feedback_health") or {}
    health_topics = health.get("topics") or {}
    for topic in TOPICS.values():
        item = health_topics.get(topic) or {}
        if item.get("frame_count", 0) <= 0:
            problems.append(f"{topic} frame_count missing")
        intervals = item.get("interval_ms") or {}
        if any(intervals.get(key) is None for key in ("p50", "p95", "p99", "max")):
            problems.append(f"{topic} interval percentiles missing")
        if item.get("max_data_age_ms") is None:
            problems.append(f"{topic} max_data_age_ms missing")

    events = report.get("stale_events") or []
    expected_topics = {TOPICS[name] for name in case["topics"]}
    actual_topics = {event.get("topic") for event in events}
    if not case["topics"]:
        if report.get("complete") is not True or report.get("fault") is not None:
            problems.append("normal run did not complete cleanly")
        if events:
            problems.append(f"normal run contains stale events: {events}")
        if report.get("model_fk_tcp_coverage_m", 0.0) < 0.95:
            problems.append("normal coverage below 0.95 m")
    else:
        if report.get("complete") is not False:
            problems.append("injected run unexpectedly complete")
        if actual_topics != expected_topics:
            problems.append(f"stale topics={sorted(actual_topics)}, expected={sorted(expected_topics)}")
        fault = report.get("fault") or ""
        for topic in expected_topics:
            if topic not in fault:
                problems.append(f"fault text missing {topic}")
        for event in events:
            if event.get("control_phase") != "after-scan":
                problems.append(f"wrong control phase: {event}")
            age = event.get("data_age_ms")
            if age is None or not 200.0 < age <= 300.0:
                problems.append(f"stale age outside 201-300 ms: {event}")
        injection = sim.get("feedback_drop") or {}
        if not injection.get("triggered"):
            problems.append("simulated feedback drop did not trigger")
        if set(injection.get("requested_topics") or ()) != set(case["topics"]):
            problems.append("simulated feedback drop topics mismatch")
        dropped = injection.get("dropped_frames") or {}
        for name in case["topics"]:
            if dropped.get(name, 0) <= 0:
                problems.append(f"no frames dropped for {name}")
    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        help="输出目录；默认使用带时间戳的新目录，拒绝复用非空目录")
    parser.add_argument("--case", action="append", choices=[case["name"] for case in CASES],
                        help="只运行指定场景；可重复，默认运行完整矩阵")
    args = parser.parse_args()
    selected = [case for case in CASES if not args.case or case["name"] in args.case]
    output = (args.output or ROOT / "out" /
              f"stage_a_feedback_{time.strftime('%Y%m%d_%H%M%S')}").resolve()
    if output.exists() and any(output.iterdir()):
        parser.error(f"输出目录非空，拒绝混用旧报告：{output}")
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    results = []
    for index, case in enumerate(selected):
        case_output = output / case["name"]
        case_output.mkdir(parents=True, exist_ok=True)
        env = os.environ.copy()
        env.update(NO_VIEWER="1", DOMAIN=str(220 + index), OUTPUT=str(case_output),
                   INITIAL_HEIGHT_MM="650")
        if case["topics"]:
            env.update(DROP_TOPICS=" ".join(case["topics"]), DROP_PHASE="after-scan",
                       DROP_DURATION_MS="250")
        print(f"[{index + 1}/{len(selected)}] {case['name']}", flush=True)
        run = subprocess.run(["bash", "run_sync_sim.sh"], cwd=ROOT, env=env,
                             text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             check=False)
        (case_output / "launcher.log").write_text(run.stdout, encoding="utf-8")
        report_path = case_output / "sync_report.json"
        sim_path = case_output / "sim.json"
        problems = []
        if not report_path.exists() or not sim_path.exists():
            problems.append("sync_report.json or sim.json missing")
            report, sim = {}, {}
        else:
            report, sim = load(report_path), load(sim_path)
            problems.extend(validate(case, run.returncode, report, sim))
        item = {
            "case": case["name"],
            "passed": not problems,
            "exit_code": run.returncode,
            "problems": problems,
            "complete": report.get("complete"),
            "fault": report.get("fault"),
            "model_fk_tcp_coverage_m": report.get("model_fk_tcp_coverage_m"),
            "feedback_health": report.get("feedback_health"),
            "stale_events": report.get("stale_events"),
            "feedback_drop": sim.get("feedback_drop"),
            "output": str(case_output),
        }
        results.append(item)
        print("  PASS" if item["passed"] else f"  FAIL: {'; '.join(problems)}", flush=True)

    summary = {
        "stage": "A DDS feedback diagnostics and fault injection",
        "timeout_ms_unchanged": 200,
        "total": len(results),
        "passed": sum(item["passed"] for item in results),
        "seconds": round(time.monotonic() - started, 3),
        "results": results,
    }
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"通过 {summary['passed']}/{summary['total']}，报告 {output / 'summary.json'}")
    return 0 if summary["passed"] == summary["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
