"""Normalize existing collection simulation output for recovery-stage runs."""
import argparse
import csv
import json
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--source',type=Path,required=True,help='collection simulator output directory')
p.add_argument('--output',type=Path,default=Path('collection_stage_data'))
a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=True)
with (a.source/'mobility_per_run.csv').open(newline='',encoding='utf-8-sig') as src:
    rows=list(csv.DictReader(src))
with (a.output/'collection_per_run.csv').open('w',newline='',encoding='utf-8') as dst:
    w=csv.writer(dst)
    w.writerow(['trajectory','speed_mps','throughput_mbps','replicate','data_reports_received','online_terminals'])
    for r in rows:
        w.writerow([r['trajectory'],r['speed_mps'],r['throughput_mbps'],r['replicate'],
                    r['complete_reports_received'],r['online_terminals']])
for name in ['experiment_config.json','mobility_final_summary.csv','mobility_collection_curve.csv']:
    (a.output/name).write_bytes((a.source/name).read_bytes())
print(a.output.resolve())
