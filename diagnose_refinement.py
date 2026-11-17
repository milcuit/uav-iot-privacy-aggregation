"""Check whether failed rounds include terminals with insufficient solo contact."""
import csv,json,random
from types import SimpleNamespace
import numpy as np
from audit_numerics import ROOT,OUT,scenario,contacts,exact_intervals,save
a=SimpleNamespace(**json.loads((ROOT/'collection_stage_data/experiment_config.json').read_text()))
with (OUT/'per_run.csv').open() as f:
    rows=[r for r in csv.DictReader(f) if r['method']=='analytic_contacts' and r['trajectory']=='random_waypoint' and float(r['speed_mps'])==15]
results=[]
for rep in range(a.repetitions):
    nodes,online,points=scenario(a,rep,'random_waypoint')
    active,_,_=contacts(a,nodes,online,points,15.)
    candidates=np.flatnonzero(active.any(axis=0))
    rng=random.Random(a.seed+rep*1009+991338)
    selected=sorted(rng.sample(candidates.tolist(),60))
    intervals=exact_intervals(nodes,points,selected,a.coverage_radius_m,15.,a.route_distance_m)
    duration=np.zeros(60)
    for start,end,j in intervals:duration[j]+=end-start
    for g in [.25,1.]:
        result=next(r for r in rows if int(r['replicate'])==rep and float(r['throughput_mbps'])==g)
        short=int(np.count_nonzero(duration*g*1e6/8<13824-1e-7))
        results.append(dict(replicate=rep,throughput_mbps=g,success=int(result['success']),insufficient_even_with_exclusive_service=short,min_selected_contact_s=float(duration.min())))
save(OUT/'short_contact_diagnosis.csv',results)
for g in [.25,1.]:
    group=[r for r in results if r['throughput_mbps']==g]
    failed=[r for r in group if not r['success']]
    print(g,'failed',len(failed),'failed_with_solo_impossible',sum(r['insufficient_even_with_exclusive_service']>0 for r in failed))
