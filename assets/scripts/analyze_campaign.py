#!/usr/bin/env python3
"""Merge campaign CSVs and summarize metrics without third-party dependencies."""
import argparse
import csv
import statistics
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--aggregate', required=True, type=Path)
    parser.add_argument('--paired', action='store_true')
    args = parser.parse_args()
    rows = []
    fields = None
    for path in args.inputs:
        with path.open(newline='') as source:
            reader = csv.DictReader(source)
            if fields is not None and reader.fieldnames != fields:
                parser.error(f'CSV columns differ: {path}')
            fields = reader.fieldnames
            rows.extend(reader)
    if not rows:
        parser.error('No campaign rows')
    rows.sort(key=lambda row: (int(row['run']), row['controller']))
    keys = [(row['run'], row['controller']) for row in rows]
    if len(set(keys)) != len(keys):
        parser.error('Duplicate run/controller rows')
    if args.paired:
        cases = {}
        for row in rows:
            cases.setdefault(row['run'], []).append(row)
        for run, pair in cases.items():
            if len(pair) != 2 or {row['controller'] for row in pair} != {'bdot', 'estimated-rate'}:
                parser.error(f'Incomplete controller pair: {run}')
            for name in ('seed', 'initial_rate_deg_s', 'initial_orbit_phase_deg',
                         'field_model', 'reference_field_model', 'sensor_profile',
                         'epoch', 'duration_s', 'time_step_s', 'master_seed',
                         'gain_A_m2_s_per_T', 'bdot_filter_s', 'estimator_enabled',
                         'goal_rate_deg_s', 'goal_dwell_s', 'final_window_s',
                         'final_window_limit_deg_s'):
                if name not in fields:
                    continue
                if pair[0][name] != pair[1][name]:
                    parser.error(f'Scenario differs within pair {run}: {name}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as target:
        writer = csv.DictWriter(target, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = []
    for controller in sorted({row['controller'] for row in rows}):
        selected = [row for row in rows if row['controller'] == controller]
        times = [float(row['detumble_time_s']) for row in selected
                 if float(row['detumble_time_s']) >= 0]
        summary.append({
            'controller': controller, 'runs': len(selected),
            'successes': sum(row['success'] == '1' for row in selected),
            'mean_detumble_time_s': statistics.mean(times) if times else -1,
            'median_detumble_time_s': statistics.median(times) if times else -1,
            'p95_detumble_time_s': statistics.quantiles(times, n=100, method='inclusive')[94]
                                  if len(times) > 1 else (times[0] if times else -1),
            'maximum_detumble_time_s': max(times, default=-1),
            'worst_final_rate_deg_s': max(float(row['final_rate_deg_s']) for row in selected),
            'mean_dipole_effort_A2_m4_s': statistics.mean(float(row['dipole_effort_A2_m4_s']) for row in selected),
            'converged_estimators': sum(float(row['estimator_convergence_s']) >= 0 for row in selected),
            'handoffs': sum(float(row['handoff_s']) >= 0 for row in selected),
            'false_low_rate_events': sum(int(row['false_low_rate_events']) for row in selected),
        })
    args.aggregate.parent.mkdir(parents=True, exist_ok=True)
    with args.aggregate.open('w', newline='') as target:
        writer = csv.DictWriter(target, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    for row in summary:
        print(f"{row['controller']}: {row['successes']}/{row['runs']} passed; "
              f"mean detumble {row['mean_detumble_time_s']:.1f} s; "
              f"false LOW_RATE entries {row['false_low_rate_events']}")
    return 0 if all(row['success'] == '1' for row in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
