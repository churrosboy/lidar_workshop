#!/usr/bin/env python3
"""Plot a measured scan saved by verify_topics.py."""
import argparse
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/gl5-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('input')
parser.add_argument('--output', default='artifacts/gl5_scan.png')
args = parser.parse_args()
scan = json.loads(Path(args.input).read_text())
points = [(r * math.cos(scan['angle_min'] + i * scan['angle_increment']),
           r * math.sin(scan['angle_min'] + i * scan['angle_increment']))
          for i, r in enumerate(scan['ranges_m']) if r is not None and r > 0]
if not points:
    raise SystemExit('No finite measured points to plot')
fig, ax = plt.subplots(figsize=(9, 8), constrained_layout=True)
x, y = zip(*points)
ax.scatter(x, y, s=3, color='#007f70', label=f'{len(points)} measured returns')
ax.scatter([0], [0], color='#c74236', marker='^', s=70, label='GL5')
ax.arrow(0, 0, 0.5, 0, color='#c74236', width=0.01, length_includes_head=True)
ax.set(xlabel='X / forward (m)', ylabel='Y / left (m)', title='SOSLAB GL5 — measured ROS 2 scan')
ax.set_aspect('equal', adjustable='datalim')
ax.grid(alpha=0.25)
ax.legend()
Path(args.output).parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.output, dpi=160)
print(args.output)
