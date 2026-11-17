"""Plot the numerical audit using the original figure's style."""
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).resolve().parent/'numerical_refinement'
with (root/'summary.csv').open() as f: rows=list(csv.DictReader(f))
plt.rcParams.update({'font.family':'serif','font.serif':['Times New Roman'],'mathtext.fontset':'stix','font.size':10,'axes.linewidth':.8})
speeds=sorted({float(r['speed_mps']) for r in rows})
rates=sorted({float(r['throughput_mbps']) for r in rows})
def panel(ax,tr,method):
    data={(float(r['speed_mps']),float(r['throughput_mbps'])):100*float(r['completion_rate']) for r in rows if r['trajectory']==tr and r['method']==method}
    z=np.array([[data[v,g] for v in speeds] for g in rates])
    im=ax.imshow(z,origin='lower',aspect='auto',cmap='jet',vmin=0,vmax=100)
    ax.set_xticks(range(len(speeds)),[f'{v:g}' for v in speeds])
    ax.set_yticks(range(len(rates)),[f'{g:g}' for g in rates])
    for y in range(len(rates)):
        for x in range(len(speeds)):
            value=z[y,x]
            label=f'{value:.1f}%' if value%1 else f'{value:.0f}%'
            ax.text(x,y,label,ha='center',va='center',fontsize=8,color='white' if value<25 or value>90 else 'black')
    ax.set_xticks(np.arange(-.5,len(speeds)),minor=True)
    ax.set_yticks(np.arange(-.5,len(rates)),minor=True)
    ax.grid(which='minor',color='white',linewidth=1.2)
    ax.tick_params(which='minor',bottom=False,left=False)
    ax.tick_params(labelsize=8)
    return im
fig,axes=plt.subplots(1,2,figsize=(7.16,3),sharey=True)
for ax,tr in zip(axes,['fixed_patrol','random_waypoint']):
    im=panel(ax,tr,'analytic_contacts')
    ax.set_title('Preplanned patrol' if tr=='fixed_patrol' else 'Random-waypoint route',fontsize=9)
    ax.set_xlabel('UAV speed (m/s)')
axes[0].set_ylabel('Shared goodput (Mbit/s)')
fig.subplots_adjust(left=.09,right=.85,bottom=.18,top=.90,wspace=.14)
fig.colorbar(im,cax=fig.add_axes([.88,.18,.018,.72])).set_label('Selected-set communication completion (%)',fontsize=9)
fig.savefig(root/'recovery_transport_heatmap_refined.png',dpi=600,bbox_inches='tight')
plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(7.16,3),sharey=True)
for ax,method,title in zip(axes,['original_dt_0.5','analytic_contacts'],['Random waypoint: original','Random waypoint: numerical refinement']):
    im=panel(ax,'random_waypoint',method);ax.set_title(title,fontsize=9);ax.set_xlabel('UAV speed (m/s)')
axes[0].set_ylabel('Shared goodput (Mbit/s)')
fig.subplots_adjust(left=.09,right=.85,bottom=.18,top=.90,wspace=.14)
fig.colorbar(im,cax=fig.add_axes([.88,.18,.018,.72])).set_label('Selected-set communication completion (%)',fontsize=9)
fig.savefig(root/'random_route_before_after.png',dpi=600,bbox_inches='tight')
print(root)
