#!/usr/bin/env python3
"""Decode embedded ROS1 definitions; no hardware, ROS master or source-bag writes."""
import argparse
import json
from collections import Counter
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from robotic_fish_acoustic.core import CLASSES, Estimator, Model, Status


def records(path):
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    with Reader(path) as reader:
        store = get_typestore(Stores.ROS1_NOETIC)
        connections = [c for c in reader.connections if c.topic == '/robotic_fish/adc/samples']
        if not connections:
            raise ValueError('bag has no ADC arrays: ' + str(path))
        for connection in connections:
            definition = connection.msgdef
            store.register(get_types_from_msg(definition if isinstance(definition, str) else definition.data, connection.msgtype))
        for connection, stamp, raw in reader.messages(connections=connections):
            yield stamp / 1e9, store.deserialize_ros1(raw, connection.msgtype)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', required=True, type=Path, help='New output directory; never overwrite')
    parser.add_argument('--model')
    parser.add_argument('--window-s', type=float, default=.5)
    args = parser.parse_args()
    paths = sorted(p for p in args.input.glob('*.bag') if not p.name.startswith('._')) if args.input.is_dir() else [args.input]
    if not paths:
        parser.error('no bags found')
    model = Model(args.model) if args.model else None
    args.output.mkdir(parents=True, exist_ok=False)
    summaries = []
    for path in paths:
        engine = Estimator(model, args.window_s)
        counts, flags = Counter(), Counter()
        elapsed = 0.
        with (args.output / (path.stem + '.jsonl')).open('x') as output:
            for now, message in records(path):
                start = time.perf_counter()
                result = engine.push(message, now)
                elapsed += time.perf_counter() - start
                counts['groups'] += 1
                counts['updated'] += int(result['updated'])
                counts['feature_rows'] += int('features' in result)
                for flag in Status:
                    if result['status'] & flag:
                        flags[flag.name] += 1
                p = result['probabilities']
                if p is not None:
                    assert all(0 <= v <= 1 for v in p) and abs(sum(p) - 1) < 1e-12
                result.update(record_time=now, probability_left=p[2]+p[3] if p else None,
                              probability_right=p[0]+p[1] if p else None)
                output.write(json.dumps(result, allow_nan=False) + '\n')
        summary = dict(bag=str(path), counts=dict(counts), flags=dict(flags),
                       mean_compute_ms=1000 * elapsed / max(counts['groups'], 1),
                       model_id=model.id if model else 'unavailable', classes=CLASSES)
        summaries.append(summary)
        print(json.dumps(summary))
    (args.output / 'summary.json').write_text(json.dumps(summaries, indent=2) + '\n')


if __name__ == '__main__':
    main()
