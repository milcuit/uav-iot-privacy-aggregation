"""Check the time accounting used by the recovery simulation."""
import csv, json, random, math, time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from mobility_two_stage import scenario, contacts
from mobility_collection_experiment import wilson_interval

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'numerical_refinement'

def grid_intervals(active, dt):
    edges = np.r_[0., np.cumsum(dt)]
    intervals = []
    for j in range(active.shape[1]):
        change = np.diff(np.r_[False, active[:,j], False].astype(int))
        intervals.extend((float(edges[s]), float(edges[e]), j)
                         for s,e in zip(np.flatnonzero(change==1), np.flatnonzero(change==-1)))
    return intervals

def exact_intervals(nodes, points, selected, radius, speed, length):
    intervals=[]
    distance=0.
    for start,end in zip(points[:-1],points[1:]):
        start=np.asarray(start); delta=np.asarray(end)-start
        segment=float(np.linalg.norm(delta))
        if segment<=1e-12: continue
        unit=delta/segment
        for j,i in enumerate(selected):
            offset=nodes[i]-start
            projection=float(offset@unit)
            perpendicular=max(0.,float(offset@offset)-projection**2)
            if perpendicular>=radius**2: continue
            half=math.sqrt(radius**2-perpendicular)
            left=max(0.,projection-half); right=min(segment,projection+half)
            if right>left:
                intervals.append(((length-distance-right)/speed,(length-distance-left)/speed,j))
        distance+=segment
    return intervals

def complete(intervals, goodput, n=60):
    # Events are actual contact boundaries; completion frees capacity immediately.
    events={}
    for start,end,j in intervals:
        events.setdefault(start,[]).append((j,1))
        events.setdefault(end,[]).append((j,-1))
    remaining=np.full(n,2*6912.,dtype=float)
    levels=np.zeros(n,dtype=int)
    previous=0.
    bytes_s=goodput*1e6/8
    for instant,changes in sorted(events.items()):
        available=max(0.,instant-previous)
        while available>1e-12:
            ids=np.flatnonzero((levels>0)&(remaining>1e-7))
            if not len(ids): break
            rate=bytes_s/len(ids)
            used=min(available,float(remaining[ids].min())/rate)
            remaining[ids]=np.maximum(0.,remaining[ids]-used*rate)
            available-=used
        for j,change in changes: levels[j]+=change
        previous=instant
    return int(np.count_nonzero(remaining<=1e-7))

def save(path, rows):
    with path.open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader();w.writerows(rows)

def checks():
    # Two jobs overlap, one ends early; released service must finish the other.
    g=13824*8/1e6
    assert complete([(0,1,0)],g,n=1)==1
    assert complete([(0,.5,0),(1,1.5,0)],g,n=1)==1
    assert complete([(0,.1,0)],g,n=1)==0
    assert complete([(0,2,0),(1,2,1)],g,n=2)==2
    assert complete([(0,1,0),(0,1,1)],g,n=2)==0
    intervals=exact_intervals(np.array([[5.,0.]]),[(0.,0.),(10.,0.)],[0],2.,1.,10.)
    assert np.allclose(intervals,[(3.,7.,0)])

def main():
    checks();OUT.mkdir(exist_ok=True)
    config=json.loads((ROOT/'collection_stage_data/experiment_config.json').read_text())
    a=SimpleNamespace(**config)
    rates=[float(g) for g in a.throughputs_mbps.split(',')]
    speeds=[float(v) for v in a.speeds.split(',')]
    with (ROOT/'mobility_protocol_recovery_results/per_run.csv').open() as f: old=list(csv.DictReader(f))
    old={(r['trajectory'],float(r['speed_mps']),float(r['throughput_mbps']),int(r['replicate'])):r for r in old}
    rows=[]; begun=time.time()
    for trajectory in ['fixed_patrol','random_waypoint']:
        for speed in speeds:
            for rep in range(a.repetitions):
                nodes,online,points=scenario(a,rep,trajectory)
                a.dt_s=.5
                active,cap,dt=contacts(a,nodes,online,points,speed)
                candidates=np.flatnonzero(active.any(axis=0))
                rng=random.Random(a.seed+rep*1009+991337+(0 if trajectory=='fixed_patrol' else 1))
                selected=sorted(rng.sample(candidates.tolist(),60)) if len(candidates)>=60 else []
                assert len(candidates)==int(old[trajectory,speed,rates[0],rep]['online_tms_identified_during_collection'])
                variants={}
                if selected:
                    assert cap[:,selected][active[:,selected]].min()>=max(rates)*1e6
                    variants['recycle_dt_0.5']=grid_intervals(active[:,selected],dt)
                    a.dt_s=.1
                    fine,_,fine_dt=contacts(a,nodes,online,points,speed)
                    variants['recycle_dt_0.1']=grid_intervals(fine[:,selected],fine_dt)
                    variants['analytic_contacts']=exact_intervals(nodes,points,selected,a.coverage_radius_m,speed,a.route_distance_m)
                for g in rates:
                    original=int(old[trajectory,speed,g,rep]['selected_responses_complete'])
                    outcomes={'original_dt_0.5':original}
                    outcomes.update({name:complete(intervals,g) for name,intervals in variants.items()})
                    if not selected: outcomes.update({name:0 for name in ['recycle_dt_0.5','recycle_dt_0.1','analytic_contacts']})
                    assert outcomes['recycle_dt_0.5']>=original
                    for method,count in outcomes.items():
                        rows.append(dict(method=method,trajectory=trajectory,speed_mps=speed,throughput_mbps=g,replicate=rep,responses_complete=count,success=int(count==60)))
            print(trajectory,speed,'elapsed',round(time.time()-begun,1),flush=True)
    save(OUT/'per_run.csv',rows)
    groups={}
    for r in rows: groups.setdefault((r['method'],r['trajectory'],r['speed_mps'],r['throughput_mbps']),[]).append(r)
    summary=[]
    for (method,tr,v,g),group in groups.items():
        successes=sum(r['success'] for r in group);lo,hi=wilson_interval(successes,len(group))
        summary.append(dict(method=method,trajectory=tr,speed_mps=v,throughput_mbps=g,successes=successes,trials=len(group),completion_rate=successes/len(group),mean_responses=np.mean([r['responses_complete'] for r in group]),ci95_low=lo,ci95_high=hi))
    save(OUT/'summary.csv',summary)
    comparison=[]
    for tr in ['fixed_patrol','random_waypoint']:
        for v in speeds:
            for g in rates:
                entries={r['method']:r for r in summary if r['trajectory']==tr and r['speed_mps']==v and r['throughput_mbps']==g}
                record=dict(trajectory=tr,speed_mps=v,throughput_mbps=g)
                record.update({m:e['completion_rate'] for m,e in entries.items()})
                record['analytic_change_pp']=100*(entries['analytic_contacts']['completion_rate']-entries['original_dt_0.5']['completion_rate'])
                comparison.append(record)
    save(OUT/'comparison.csv',comparison)
    (OUT/'config.json').write_text(json.dumps(dict(original_config=config,selected_set='Frozen to original 0.5-s discovery, same terminal identities for all methods',service='Equal-share work-conserving half-duplex unicast; immediate reuse of capacity after a terminal completes',exact_contacts='Analytic line-segment / horizontal coverage-disk intersections; no time discretization',unchanged='Original collection data, routes, seeds, availability, selected t, message sizes, link threshold, speed, goodput, flight budget',self_checks='passed'),indent=2))
    print('SAVED',OUT,flush=True)

if __name__=='__main__': main()
