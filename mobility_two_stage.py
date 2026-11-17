"""Simulate the recovery traffic for a preselected set of terminals."""
import csv
import json
import math
import random
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from mobility_collection_experiment import position_at, wilson_interval

ROOT = Path(__file__).resolve().parent

def scenario(a, replicate, trajectory):
    rng = random.Random(a.seed + replicate*1009)
    nodes = np.array([(rng.uniform(0,a.field_m),rng.uniform(0,a.field_m)) for _ in range(a.n)])
    rng = random.Random(a.seed + replicate*1009 + 77777)
    online = np.array([rng.random()<a.availability for _ in range(a.n)])
    points = a.fixed_route
    if trajectory == 'random_waypoint':
        rng = random.Random(a.seed + replicate*1009 + 991337)
        points = [(0.,0.)]
        distance = 0.
        while distance < a.route_distance_m:
            nxt = (rng.uniform(0,a.field_m),rng.uniform(0,a.field_m))
            length = math.dist(points[-1],nxt)
            if distance+length > a.route_distance_m:
                f = (a.route_distance_m-distance)/length
                nxt = tuple(p+f*(q-p) for p,q in zip(points[-1],nxt))
                length = a.route_distance_m-distance
            distance += length
            points.append(nxt)
    return nodes, online, points

def contacts(a,nodes,online,points,speed):
    duration = a.route_distance_m/speed
    t = np.arange(0,duration,a.dt_s)
    dt = np.minimum(a.dt_s,duration-t)
    positions = np.array([position_at(points,speed*(x+d/2)) for x,d in zip(t,dt)])
    horizontal = np.linalg.norm(positions[:,None,:]-nodes[None,:,:],axis=2)
    # Same mean-loss/SNR model as the original script; gains and power unchanged.
    d3 = np.maximum(np.hypot(horizontal,a.altitude_m),1.)
    elevation = np.degrees(np.arctan2(a.altitude_m,np.maximum(horizontal,1e-9)))
    los = 1/(1+a.urban_a*np.exp(-a.urban_b*(elevation-a.urban_a)))
    loss = 20*np.log10(4*np.pi*a.fc_hz*d3/299792458.)+los*a.eta_los_db+(1-los)*a.eta_nlos_db
    noise = -174+10*np.log10(a.bandwidth_hz)+a.noise_figure_db
    snr = a.tx_power_dbm+a.tx_gain_dbi+a.rx_gain_dbi-loss-noise
    active = (snr>=a.min_snr_db)&online[None,:]
    cap = np.minimum(a.per_link_cap_bps,a.bandwidth_hz*np.log2(1+10**(snr/10)))
    return active[::-1], cap[::-1], dt[::-1]

def recovery(active,cap,dt,goodput,down_bytes,up_bytes):
    down = np.zeros(active.shape[1]); up = np.zeros_like(down)
    for eligible,capacity,seconds in zip(active,cap,dt):
        pending = eligible & (up < up_bytes-1e-8)
        count = np.count_nonzero(pending)
        if not count: continue
        # One shared half-duplex unicast payload budget across both directions.
        # A terminal first downloads its request, then uploads its response.
        rate = np.minimum(capacity,goodput*1e6/count)/8
        budget = rate*seconds
        download = np.minimum(np.maximum(down_bytes-down,0),budget)
        down[pending] += download[pending]
        upload = np.minimum(np.maximum(up_bytes-up,0),budget-download)
        up[pending] += upload[pending]
    return int(np.count_nonzero(up>=up_bytes-1e-8))

def main():
    source = ROOT/'collection_stage_data'
    out = ROOT/'mobility_protocol_recovery_results'; out.mkdir(exist_ok=True)
    config = json.loads((source/'experiment_config.json').read_text())
    a = SimpleNamespace(**config)
    a.threshold = int(config.get('threshold', 60))
    down_bytes = int(config.get('polynomial_bytes', 6912))
    up_bytes = int(config.get('response_bytes', 6912))
    with (source/'collection_per_run.csv').open(newline='') as f:
        old = list(csv.DictReader(f))
    lookup = {(r['trajectory'],float(r['speed_mps']),float(r['throughput_mbps']),int(r['replicate'])):int(r['data_reports_received']) for r in old}
    rates = [float(x) for x in a.throughputs_mbps.split(',')]
    speeds = [float(x) for x in a.speeds.split(',')]
    rows=[]; summary=[]
    for trajectory in ['fixed_patrol','random_waypoint']:
        for speed in speeds:
            counts={g:[] for g in rates}
            for rep in range(a.repetitions):
                nodes,online,points=scenario(a,rep,trajectory)
                active,cap,dt=contacts(a,nodes,online,points,speed)
                # The same route is flown in reverse for recovery. Any online TM
                # contacted during collection is therefore in CC's known set.
                collection_contacts=active[::-1]
                candidates=np.flatnonzero(np.any(collection_contacts,axis=0))
                selection_rng=random.Random(a.seed+rep*1009+991337+(0 if trajectory=='fixed_patrol' else 1))
                if len(candidates)>=a.threshold:
                    selected=selection_rng.sample(candidates.tolist(),a.threshold)
                    selected_mask=np.zeros(a.n,dtype=bool); selected_mask[selected]=True
                else:
                    selected=[]
                    selected_mask=np.zeros(a.n,dtype=bool)
                selected_active=active[:,selected_mask]
                selected_cap=cap[:,selected_mask]
                for g in rates:
                    reports=lookup[trajectory,speed,g,rep]
                    if len(selected)==a.threshold:
                        count=recovery(selected_active,selected_cap,dt,g,down_bytes,up_bytes)
                    else:
                        count=0
                    counts[g].append(count)
                    rows.append([trajectory,speed,g,rep,reports,len(candidates),len(selected),count,int(count==a.threshold),2*a.route_distance_m/speed])
            for g,values in counts.items():
                successes=sum(v==a.threshold for v in values)
                lo,hi=wilson_interval(successes,len(values))
                summary.append([trajectory,speed,g,sum(values)/len(values),successes,successes/len(values),lo,hi,len(values)])
            print(trajectory,speed,'completed',flush=True)
    with (out/'per_run.csv').open('w',newline='') as f:
        w=csv.writer(f)
        w.writerow(['trajectory','speed_mps','throughput_mbps','replicate','data_reports_received','online_tms_identified_during_collection','selected_t','selected_responses_complete','selected_set_complete','total_flight_time_s']);w.writerows(rows)
    with (out/'summary.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['trajectory','speed_mps','throughput_mbps','mean_selected_responses_complete','selected_set_complete_trials','selected_set_completion_rate','ci95_low','ci95_high','repetitions']);w.writerows(summary)
    config.update(threshold=a.threshold,recovery_route='reverse of collection route, starting after final aggregation',recovery_distance_m=a.route_distance_m,total_distance_m=2*a.route_distance_m,polynomial_bytes=down_bytes,downlink_polynomials=1,response_bytes=up_bytes,selection='during ciphertext collection, online TMs that establish an in-range contact form the set known to CC; CC uniformly selects exactly t from this set before recovery; selection and discovery overhead are not charged',metric='all selected t request-response transfers complete before the recovery route ends; transport feasibility, not cryptographic correctness',availability='independent Bernoulli draw at start of each trial; state remains fixed across collection and recovery',service='equal-share half-duplex unicast among selected terminals in contact; symmetric link budget; partial transfers resume; no headers, crypto delays, or leftover-share redistribution within timestep')
    (out/'config.json').write_text(json.dumps(config,indent=2))
    print(out)

if __name__=='__main__':main()
