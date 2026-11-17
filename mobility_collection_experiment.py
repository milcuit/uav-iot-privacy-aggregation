#!/usr/bin/env python3
"""Paired UAV mobility and distance-dependent report-collection simulation."""
from __future__ import annotations
import argparse, csv, json, math, random
from pathlib import Path

C = 299_792_458.0
Z95 = 1.96

def path_loss_db(distance_m, altitude_m, fc_hz, a, b, eta_los_db, eta_nlos_db):
    d3 = max(math.hypot(distance_m, altitude_m), 1.0)
    elev_deg = math.degrees(math.atan2(altitude_m, max(distance_m, 1e-9)))
    p_los = 1.0 / (1.0 + a * math.exp(-b * (elev_deg - a)))
    fspl = 20.0 * math.log10(4.0 * math.pi * fc_hz * d3 / C)
    return fspl + p_los * eta_los_db + (1.0-p_los) * eta_nlos_db

def effective_horizontal_radius(args):
    noise_dbm=-174.0+10.0*math.log10(args.bandwidth_mhz*1e6)+args.noise_figure_db
    tx_dbm=args.tx_power_dbm+args.tx_gain_dbi+args.rx_gain_dbi
    def snr_db(horizontal_m):
        loss=path_loss_db(horizontal_m,args.altitude_m,args.fc_ghz*1e9,
                          args.urban_a,args.urban_b,args.eta_los_db,args.eta_nlos_db)
        return tx_dbm-loss-noise_dbm
    low,high=0.0,max(args.field_m*4.0,1000.0)
    for _ in range(60):
        mid=(low+high)/2.0
        if snr_db(mid)>=args.min_snr_db: low=mid
        else: high=mid
    return low

def make_preplanned_route(field_m, coverage_radius_m):
    """Serpentine patrol with lane spacing bounded by the link coverage swath."""
    lanes=max(1,math.ceil(field_m/(2.0*coverage_radius_m)))
    spacing=field_m/lanes
    ys=[(j+0.5)*spacing for j in range(lanes)]
    points=[(0.0,0.0)]
    for j,y in enumerate(ys):
        row=[(0.0,y),(field_m,y)] if j%2==0 else [(field_m,y),(0.0,y)]
        if points[-1]==row[0]: row=row[1:]
        points.extend(row)
    return points,spacing

def make_random_waypoints(rng, field_m, count):
    return [(rng.uniform(0,field_m),rng.uniform(0,field_m)) for _ in range(count)]

def position_at(points, distance_m):
    left=max(0.0,distance_m)
    for p,q in zip(points,points[1:]):
        length=math.dist(p,q)
        if length <= 1e-12: continue
        if left <= length:
            f=left/length
            return p[0]+f*(q[0]-p[0]), p[1]+f*(q[1]-p[1])
        left-=length
    return points[-1]

def path_length(points):
    return sum(math.dist(p,q) for p,q in zip(points,points[1:]))

def one_run_full_route(args, replicate, trajectory, speed, throughput_mbps):
    rng=random.Random(args.seed + replicate*1009)
    nodes=[(rng.uniform(0,args.field_m),rng.uniform(0,args.field_m)) for _ in range(args.n)]
    online_rng=random.Random(args.seed + replicate*1009 + 77_777)
    online=[online_rng.random() < args.availability for _ in range(args.n)]
    if trajectory == 'fixed_patrol':
        points=args.fixed_route
    else:
        route_rng=random.Random(args.seed + replicate*1009 + 991_337)
        points=[(0.0,0.0)]
        distance=0.0
        while distance < args.route_distance_m:
            nxt=(route_rng.uniform(0,args.field_m),route_rng.uniform(0,args.field_m))
            segment=math.dist(points[-1],nxt)
            if distance+segment > args.route_distance_m:
                fraction=(args.route_distance_m-distance)/segment
                nxt=(points[-1][0]+fraction*(nxt[0]-points[-1][0]),
                     points[-1][1]+fraction*(nxt[1]-points[-1][1]))
                segment=args.route_distance_m-distance
            distance+=segment
            points.append(nxt)

    received=[0.0]*args.n
    done=[False]*args.n
    online_fraction=sum(online)/args.n
    samples=[(0.0,0.0,0.0,online_fraction)]
    noise_dbm=-174.0+10.0*math.log10(args.bandwidth_hz)+args.noise_figure_db
    flight_duration=args.route_distance_m/speed
    steps=round(args.horizon_s/args.dt_s)
    sample_every=max(1,round(args.sample_s/args.dt_s))
    for k in range(steps):
        t0=k*args.dt_s
        t1=min((k+1)*args.dt_s,args.horizon_s)
        service_dt=max(0.0,min(t1,flight_duration)-min(t0,flight_duration))
        if service_dt>0:
            mid_time=t0+service_dt/2.0
            ux,uy=position_at(points,speed*mid_time)
            active=[]
            capacities=[]
            for i,(x,y) in enumerate(nodes):
                if not online[i] or done[i]: continue
                horizontal=math.hypot(ux-x,uy-y)
                pl=path_loss_db(horizontal,args.altitude_m,args.fc_hz,args.urban_a,args.urban_b,args.eta_los_db,args.eta_nlos_db)
                rx_dbm=args.tx_power_dbm+args.tx_gain_dbi+args.rx_gain_dbi-pl
                snr_db=rx_dbm-noise_dbm
                if snr_db < args.min_snr_db: continue
                snr=10.0**(snr_db/10.0)
                capacities.append(min(args.per_link_cap_bps,args.bandwidth_hz*math.log2(1.0+snr)))
                active.append(i)
            if active:
                share=throughput_mbps*1e6/len(active)
                for i,cap in zip(active,capacities):
                    received[i]=min(args.report_bytes,received[i]+min(cap,share)*service_dt/8.0)
                    if received[i]>=args.report_bytes-1e-8: done[i]=True
        if (k+1)%sample_every==0 or k+1==steps:
            samples.append((round(t1,6),sum(done)/args.n,sum(done),online_fraction))
    return samples

def summarize(values):
    mean=sum(values)/len(values)
    if len(values)<2: return mean,0.0
    var=sum((x-mean)**2 for x in values)/(len(values)-1)
    return mean,Z95*math.sqrt(var/len(values))

def wilson_interval(successes, repetitions):
    fraction=successes/repetitions
    z2=Z95**2
    denominator=1.0+z2/repetitions
    center=(fraction+z2/(2.0*repetitions))/denominator
    radius=Z95*math.sqrt(fraction*(1.0-fraction)/repetitions+z2/(4.0*repetitions**2))/denominator
    return max(0.0,center-radius),min(1.0,center+radius)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir',default='mobility_experiment_results')
    p.add_argument('--n',type=int,default=100)
    p.add_argument('--field-m',type=float,default=500.0)
    p.add_argument('--altitude-m',type=float,default=100.0)
    p.add_argument('--speeds',default='5,7.5,10,12.5,15')
    p.add_argument('--throughputs-mbps',default='0.05,0.1,0.25,0.5,1.0')
    p.add_argument('--dt-s',type=float,default=0.5)
    p.add_argument('--sample-s',type=float,default=10.0)
    p.add_argument('--repetitions',type=int,default=200)
    p.add_argument('--seed',type=int,default=20261110)
    p.add_argument('--availability',type=float,default=0.9)
    p.add_argument('--collection-target',type=float,default=0.8,
                   help='operational report-collection target; NOT the cryptographic t/n threshold')
    p.add_argument('--report-bytes',type=int,default=20736)
    p.add_argument('--lane-spacing-m',type=float,default=75.0)
    p.add_argument('--fc-ghz',type=float,default=2.0)
    p.add_argument('--bandwidth-mhz',type=float,default=20.0)
    p.add_argument('--per-link-cap-mbps',type=float,default=6.0)
    p.add_argument('--tx-power-dbm',type=float,default=23.0)
    p.add_argument('--tx-gain-dbi',type=float,default=0.0)
    p.add_argument('--rx-gain-dbi',type=float,default=0.0)
    p.add_argument('--noise-figure-db',type=float,default=7.0)
    p.add_argument('--min-snr-db',type=float,default=20.0)
    p.add_argument('--urban-a',type=float,default=9.61)
    p.add_argument('--urban-b',type=float,default=0.16)
    p.add_argument('--eta-los-db',type=float,default=1.0)
    p.add_argument('--eta-nlos-db',type=float,default=20.0)
    a=p.parse_args()
    speeds=[float(x) for x in a.speeds.split(',')]
    if a.n<1 or a.repetitions<2 or not speeds or min(speeds)<=0: p.error('n, repetitions, and speeds must be positive; repetitions >= 2')
    if not 0<a.availability<=1 or not 0<a.collection_target<=1: p.error('availability and target must be in (0,1]')
    throughputs=[float(x) for x in a.throughputs_mbps.split(',')]
    if not throughputs or min(throughputs)<=0: p.error('throughput values must be positive')
    if min(a.field_m,a.altitude_m,a.dt_s,a.sample_s,a.bandwidth_mhz,a.report_bytes)<=0: p.error('physical and timing parameters must be positive')
    a.max_speed=max(speeds)
    a.min_speed=min(speeds)
    a.coverage_radius_m=effective_horizontal_radius(a)
    a.fixed_route,a.lane_spacing_m=make_preplanned_route(a.field_m,a.coverage_radius_m)
    a.route_distance_m=path_length(a.fixed_route)
    a.horizon_s=a.route_distance_m/a.min_speed
    a.fc_hz=a.fc_ghz*1e9
    a.bandwidth_hz=a.bandwidth_mhz*1e6
    a.per_link_cap_bps=a.per_link_cap_mbps*1e6
    out=Path(a.output_dir); out.mkdir(parents=True,exist_ok=True)
    with (out/'experiment_config.json').open('w',encoding='utf-8') as f:
        json.dump(vars(a),f,indent=2)
    trajectory_names=['fixed_patrol','random_waypoint']
    # Both route families use the same starting point and total flight distance.
    raw={}
    for tr in trajectory_names:
        for speed in speeds:
            for throughput in throughputs:
                runs=[one_run_full_route(a,r,tr,speed,throughput) for r in range(a.repetitions)]
                raw[(tr,speed,throughput)]=runs
    path=out/'mobility_collection_curve.csv'
    with path.open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f); w.writerow(['trajectory','speed_mps','throughput_mbps','time_s','collection_mean','ci95_halfwidth','mean_reports','mean_online_fraction','repetitions','target_fraction'])
        for tr in trajectory_names:
            for speed in speeds:
                for throughput in throughputs:
                    runs=raw[(tr,speed,throughput)]
                    for j,(t,_,_,_) in enumerate(runs[0]):
                        vals=[run[j][1] for run in runs]
                        m,ci=summarize(vals)
                        mr=sum(run[j][2] for run in runs)/len(runs)
                        on=sum(run[j][3] for run in runs)/len(runs)
                        w.writerow([tr,speed,throughput,t,f'{m:.8f}',f'{ci:.8f}',f'{mr:.4f}',f'{on:.6f}',a.repetitions,a.collection_target])
    # Compact final summary suitable for manuscript table.
    with (out/'mobility_final_summary.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f); w.writerow(['trajectory','speed_mps','throughput_mbps','final_collection_mean','ci95_halfwidth','target_fraction','target_met_fraction_of_runs','mean_time_to_target_s','n','repetitions'])
        for tr in trajectory_names:
            for speed in speeds:
                for throughput in throughputs:
                    runs=raw[(tr,speed,throughput)]; finals=[r[-1][1] for r in runs]
                    mean,ci=summarize(finals)
                    met=sum(v>=a.collection_target for v in finals)/len(finals)
                    times=[]
                    for run in runs:
                        times.append(next((row[0] for row in run if row[1]>=a.collection_target),float('nan')))
                    finite=[t for t in times if math.isfinite(t)]
                    w.writerow([tr,speed,throughput,f'{mean:.6f}',f'{ci:.6f}',a.collection_target,f'{met:.4f}',f'{sum(finite)/len(finite):.3f}' if finite else 'NA',a.n,len(runs)])
    with (out/'mobility_per_run.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f)
        w.writerow(['trajectory','speed_mps','throughput_mbps','replicate','complete_reports_received','online_terminals'])
        for (tr,speed,throughput),runs in raw.items():
            for replicate,run in enumerate(runs):
                count=int(run[-1][2])
                w.writerow([tr,speed,throughput,replicate,count,round(run[-1][3]*a.n)])
    print(f'Wrote {path} and mobility_final_summary.csv')
    print(f'Collection target {a.collection_target:.0%} is an operational target, not t/n recovery threshold.')
    print('This stage reports complete encrypted data reports only; it does not evaluate threshold recovery.')
    print(f'Link-derived coverage radius: {a.coverage_radius_m:.1f} m; lane spacing: {a.lane_spacing_m:.1f} m; route distance: {a.route_distance_m:.1f} m.')
    print(f'Common plot horizon: {a.horizon_s:.1f} s (slowest speed completes route). Throughput sweep: {throughputs} Mbit/s.')
    print('Model: Al-Hourani mean urban air-to-ground path loss; equal-share single channel; no fast fading/shadowing or measured PHY.')
if __name__=='__main__': main()
