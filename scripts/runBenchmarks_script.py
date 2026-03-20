#!/usr/bin/env python3
import os
import re
import sys
import json
import shutil
import argparse
import subprocess
import csv
from datetime import datetime
from statistics import mean, stdev
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any, Tuple


# ── Data types ─────────────────────────────────────────────────────────────────

@dataclass
class RunResult:
    time_s: float
    fps: float


@dataclass
class CaseResults:
    name: str
    scene: str
    results: List[RunResult] = field(default_factory=list)
    failures: List[str]      = field(default_factory=list)

    @property
    def n_success(self) -> int:
        return len(self.results)

    @property
    def n_failures(self) -> int:
        return len(self.failures)

    def stats(self) -> Optional[Dict[str, float]]:
        if not self.results:
            return None
        times = [r.time_s for r in self.results]
        fpss  = [r.fps    for r in self.results]
        return {
            'mean_time':   mean(times),
            'stddev_time': stdev(times) if len(times) > 1 else 0.0,
            'min_time':    min(times),
            'max_time':    max(times),
            'mean_fps':    mean(fpss),
        }


# ── Config ─────────────────────────────────────────────────────────────────────

DEFAULTS: Dict[str, Any] = {
    'sofa_exe': 'runSofa',
    'warmup':   1,
    'output':   'log.benchmark',
}

# Must be explicitly set in config or via CLI — no silent defaults.
REQUIRED_KEYS = ('iterations', 'n_tests', 'timeout')

# All keys overrideable via CLI — must match build_parser arguments exactly.
OVERRIDE_KEYS = (*DEFAULTS.keys(), *REQUIRED_KEYS)


def load_config(config_path: str, overrides: Dict[str, Any]) -> Dict[str, Any]:
    with open(config_path) as f:
        config = json.load(f)
    for key, val in DEFAULTS.items():
        config.setdefault(key, val)
    for key, val in overrides.items():
        if val is not None:
            config[key] = val
    return config


def validate_config(config: Dict[str, Any]) -> List[str]:
    errors = []

    for key in REQUIRED_KEYS:
        if key not in config:
            errors.append(f"'{key}' is required but not set in config or CLI")

    sofa_exe = config.get('sofa_exe')
    if not sofa_exe:
        errors.append("'sofa_exe' is required but not set")
    elif not shutil.which(sofa_exe):
        errors.append(f"sofa_exe not found: '{sofa_exe}'")

    cases = config.get('cases')
    if not cases:
        errors.append("'cases' list is missing or empty")
        return errors

    for i, case in enumerate(cases):
        tag = f"Case '{case['name']}'" if 'name' in case else f"Case {i}"
        if 'name' not in case:
            errors.append(f"Case {i}: missing 'name'")
        if 'scene' not in case:
            errors.append(f"{tag}: missing 'scene'")
        elif not os.path.isfile(case['scene']):
            errors.append(f"{tag}: scene file not found: '{case['scene']}'")

    for key in REQUIRED_KEYS:
        val = config.get(key)
        if val is not None and (not isinstance(val, int) or val < 1):
            errors.append(f"'{key}' must be a positive integer, got: {val!r}")
    warmup = config.get('warmup')
    if warmup is not None and (not isinstance(warmup, int) or warmup < 0):
        errors.append(f"'warmup' must be a non-negative integer, got: {warmup!r}")

    return errors


# ── Runner ─────────────────────────────────────────────────────────────────────

_TIMING_RE  = re.compile(r'(\d+(?:\.\d+)?)\s+s\s*\(\s*(\d+(?:\.\d+)?)\s*FPS\s*\)', re.IGNORECASE)
_NUMBER_RE  = re.compile(r'\d+\.\d+')


def _parse_timing_line(line: str) -> Optional[RunResult]:
    m = _TIMING_RE.search(line)
    if m:
        return RunResult(float(m.group(1)), float(m.group(2)))
    # Fallback: last two decimal numbers in the line
    numbers = _NUMBER_RE.findall(line)
    if len(numbers) >= 2:
        return RunResult(float(numbers[-2]), float(numbers[-1]))
    return None


def run_single(
    sofa_exe: str, scene: str, iterations: int, timeout: int
) -> Tuple[Optional[RunResult], Optional[str]]:
    """Returns (RunResult, None) on success, (None, error_message) on failure. Never raises."""
    try:
        proc = subprocess.run(
            [sofa_exe, '-g', 'batch', '-n', str(iterations), scene],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return None, f'Timeout after {timeout}s'
    except Exception as e:
        return None, f'Failed to launch: {e}'

    for line in proc.stdout.splitlines():
        if 'iterations done in' in line:
            result = _parse_timing_line(line)
            if result:
                return result, None
            return None, f'Found timing line but could not parse it: {line!r}'

    error_detail = f'exit={proc.returncode}, no timing line in output'
    last_stderr = proc.stderr.strip().rsplit('\n', 1)[-1][:200]
    if last_stderr:
        error_detail += f' | stderr: {last_stderr}'
    return None, error_detail


def run_case(config: Dict[str, Any], case: Dict[str, str]) -> CaseResults:
    sofa_exe   = config['sofa_exe']
    iterations = config['iterations']
    timeout    = config['timeout']
    n_tests    = config['n_tests']
    warmup     = config['warmup']
    scene      = case['scene']
    name       = case['name']

    cr = CaseResults(name=name, scene=scene)

    for i in range(warmup + n_tests):
        is_warmup = i < warmup
        label = f'warmup {i + 1}/{warmup}' if is_warmup else f'test {i - warmup + 1}/{n_tests}'
        print(f'  [{name}] {label} ... ', end='', flush=True)

        result, error = run_single(sofa_exe, scene, iterations, timeout)

        if result is not None:
            suffix = ' [warmup, discarded]' if is_warmup else ''
            print(f'{result.time_s:.3f}s  ({result.fps:.1f} FPS){suffix}')
            if not is_warmup:
                cr.results.append(result)
        else:
            suffix = ' [warmup]' if is_warmup else ''
            print(f'FAILED{suffix}: {error}')
            if not is_warmup:
                cr.failures.append(error)

    return cr


# ── Reporting ──────────────────────────────────────────────────────────────────

def print_table(case_results: List[CaseResults], all_stats: List[Optional[Dict]]) -> None:
    col = 24
    header = (
        f'{"Case":<{col}} {"Ok":>4} {"Fail":>4}'
        f' {"Mean (s)":>9} {"Std":>7} {"Min":>7} {"Max":>7} {"FPS":>7}'
    )
    width = len(header)
    print()
    print('=' * width)
    print(header)
    print('-' * width)
    for cr, s in zip(case_results, all_stats):
        name = cr.name[:col]
        if s:
            print(
                f'{name:<{col}} {cr.n_success:>4} {cr.n_failures:>4}'
                f' {s["mean_time"]:>9.3f} {s["stddev_time"]:>7.3f}'
                f' {s["min_time"]:>7.3f} {s["max_time"]:>7.3f}'
                f' {s["mean_fps"]:>7.1f}'
            )
        else:
            print(f'{name:<{col}} {0:>4} {cr.n_failures:>4}  ALL FAILED')
    print('=' * width)


def write_csv(output_prefix: str, case_results: List[CaseResults], all_stats: List[Optional[Dict]]) -> str:
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = f'{output_prefix}.{timestamp}.csv'
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'name', 'scene', 'n_success', 'n_failures',
            'mean_time_s', 'stddev_time_s', 'min_time_s', 'max_time_s', 'mean_fps',
        ])
        for cr, s in zip(case_results, all_stats):
            if s:
                writer.writerow([
                    cr.name, cr.scene, cr.n_success, cr.n_failures,
                    s['mean_time'], s['stddev_time'], s['min_time'], s['max_time'], s['mean_fps'],
                ])
            else:
                writer.writerow([cr.name, cr.scene, 0, cr.n_failures, '', '', '', '', ''])
    return path


# ── CLI ────────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description='Benchmark SOFA scenes',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument('-config',     required=True,          help='Path to JSON config file')
    p.add_argument('-sofa_exe',   default=None,           help='Override: path to runSofa executable')
    p.add_argument('-iterations', default=None, type=int, help='Override: ODE iterations per run')
    p.add_argument('-n_tests',    default=None, type=int, help='Override: timed test runs per case')
    p.add_argument('-warmup',     default=None, type=int, help='Override: warmup runs (discarded from stats)')
    p.add_argument('-timeout',    default=None, type=int, help='Override: per-run timeout in seconds')
    p.add_argument('-output',     default=None,           help='Override: output CSV filename prefix')
    return p


def main() -> None:
    args = build_parser().parse_args()

    overrides = {k: getattr(args, k) for k in OVERRIDE_KEYS}

    try:
        config = load_config(args.config, overrides)
    except FileNotFoundError:
        print(f'Error: config file not found: {args.config}', file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f'Error: invalid JSON in config: {e}', file=sys.stderr)
        sys.exit(1)

    errors = validate_config(config)
    if errors:
        print('Config errors:', file=sys.stderr)
        for e in errors:
            print(f'  - {e}', file=sys.stderr)
        sys.exit(1)

    print(
        f'Benchmark: {len(config["cases"])} case(s), '
        f'{config["n_tests"]} tests + {config["warmup"]} warmup, '
        f'{config["iterations"]} iterations, '
        f'timeout={config["timeout"]}s'
    )

    case_results: List[CaseResults] = []
    for case in config['cases']:
        print(f'\nCase: {case["name"]}  ->  {case["scene"]}')
        case_results.append(run_case(config, case))

    all_stats = [cr.stats() for cr in case_results]
    print_table(case_results, all_stats)

    csv_path = write_csv(config['output'], case_results, all_stats)
    print(f'\nResults written to: {csv_path}')


if __name__ == '__main__':
    main()
