"""
IoT-Based Automotive Range Detection System
Python Simulation — Phase 2 Evidence
Kalman Filter Sensor Fusion + Dynamic Safe Distance

Authors: Ajinkya Bhagwat, Manan Kabra, Rishita Modi
Institution: NMIT Bengaluru | VTU | Academic Year 2026-27

HOW TO RUN:
1. Open terminal / command prompt
2. Type: pip install numpy matplotlib
3. Type: python kalman_simulation_FINAL.py
4. Graph saved as: simulation_results.png in same folder
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import warnings
warnings.filterwarnings("ignore")

np.random.seed(42)

N    = 300
dt   = 0.2
time = np.arange(N) * dt

# Ground Truth
true_dist = np.zeros(N)
for i in range(N):
    t = time[i]
    if t < 30:
        true_dist[i] = 500 - 12*t + 0.05*t**2
    else:
        true_dist[i] = max(40, 140 + 4*(t-30))
true_dist = np.clip(true_dist, 20, 600)

# Sensor readings
hcsr04_noise = np.where(true_dist > 300, np.random.normal(0,18,N), np.random.normal(0,6,N))
hcsr04 = true_dist + hcsr04_noise
lidar  = true_dist + np.random.normal(0, 2.5, N)
spike_idx = np.random.choice(N, size=12, replace=False)
hcsr04[spike_idx] += np.random.choice([-1,1],12) * np.random.uniform(30,80,12)
hcsr04 = np.clip(hcsr04, 2, 600)

# Kalman Filter
def kalman_filter_dual(z1, z2, Q=0.01, R=0.5):
    n = len(z1); x_est = np.zeros(n); p_est = np.zeros(n)
    x = z1[0]; p = 1.0
    for i in range(n):
        p = p + Q
        K1 = p/(p+R);     x = x+K1*(z1[i]-x);          p=(1-K1)*p
        K2 = p/(p+R*0.25); z_avg=(z1[i]+z2[i])/2; x=x+K2*(z_avg-x); p=(1-K2)*p
        x_est[i]=x; p_est[i]=p
    return x_est, p_est

fused, uncertainty = kalman_filter_dual(hcsr04, lidar)

# Safe distance formula (IS 11556)
def safe_distance(speed_kmh):
    v = speed_kmh / 3.6
    return max(20, (v*0.7 + v*v/14.0)*100)

speed_profile = np.zeros(N)
for i in range(N):
    t = time[i]
    if   t < 20: speed_profile[i] = 3*t
    elif t < 40: speed_profile[i] = 60-(t-20)
    else:        speed_profile[i] = max(0, 40-2*(t-40))

safe_dist = np.array([safe_distance(s) for s in speed_profile])

# Alert levels
alert = np.zeros(N, dtype=int)
for i in range(N):
    if   fused[i] < safe_dist[i]*0.5: alert[i] = 2
    elif fused[i] < safe_dist[i]:     alert[i] = 1

# Metrics
mae_hcsr  = np.mean(np.abs(hcsr04-true_dist))
mae_lidar = np.mean(np.abs(lidar-true_dist))
mae_fused = np.mean(np.abs(fused-true_dist))
rmse_hcsr  = np.sqrt(np.mean((hcsr04-true_dist)**2))
rmse_lidar = np.sqrt(np.mean((lidar-true_dist)**2))
rmse_fused = np.sqrt(np.mean((fused-true_dist)**2))
static_alerts = np.sum((hcsr04<150)&(true_dist>=150))
dynamic_false = np.sum((fused<safe_dist)&(true_dist>=safe_dist))
false_reduction = (static_alerts-dynamic_false)/max(static_alerts,1)*100

# Plot
BG = '#0d0d1a'; GRID = '#1e1e35'; TEXT = '#e0e0f0'
C  = {'true':'#00f5d4','hcsr':'#f72585','lidar':'#4cc9f0','fused':'#ffe566',
      'safe':'#ff6b35','warn':'#ffd166','danger':'#ef233c'}

def style(ax, title):
    ax.set_facecolor(BG); ax.set_title(title, color=TEXT, fontsize=11, fontweight='bold', pad=8)
    ax.tick_params(colors=TEXT, labelsize=8); ax.xaxis.label.set_color(TEXT); ax.yaxis.label.set_color(TEXT)
    for sp in ax.spines.values(): sp.set_color('#2a2a4a')
    ax.grid(True, color=GRID, linewidth=0.5, alpha=0.7)

fig = plt.figure(figsize=(18,14)); fig.patch.set_facecolor(BG)
gs  = gridspec.GridSpec(3, 2, figure=fig, hspace=0.42, wspace=0.32)

# Graph 1
ax1 = fig.add_subplot(gs[0,:])
ax1.fill_between(time, fused-2*np.sqrt(uncertainty+1e-6), fused+2*np.sqrt(uncertainty+1e-6), alpha=0.15, color=C['fused'], label='Kalman +-2sigma')
ax1.plot(time, true_dist, color=C['true'],  lw=2.0, label='Ground Truth', zorder=5)
ax1.plot(time, hcsr04,    color=C['hcsr'],  lw=0.7, alpha=0.65, label=f'HC-SR04 (MAE={mae_hcsr:.1f} cm)')
ax1.plot(time, lidar,     color=C['lidar'], lw=0.7, alpha=0.65, label=f'TF-Luna LiDAR (MAE={mae_lidar:.1f} cm)')
ax1.plot(time, fused,     color=C['fused'], lw=2.2, label=f'Kalman Fused (MAE={mae_fused:.1f} cm)', zorder=6)
ax1.set_xlabel('Time (s)'); ax1.set_ylabel('Distance (cm)'); ax1.set_xlim(0,time[-1])
ax1.legend(loc='upper right', facecolor='#14142a', edgecolor='#2a2a4a', labelcolor=TEXT, fontsize=8.5)
style(ax1, 'Graph 1: Kalman Filter Sensor Fusion — HC-SR04 + TF-Luna LiDAR  (Q=0.01, R=0.5)')

# Graph 2
ax2 = fig.add_subplot(gs[1,0])
ax2.fill_between(time,0,650,where=(alert==2),alpha=0.18,color=C['danger'],label='DANGER zone')
ax2.fill_between(time,0,650,where=(alert==1),alpha=0.15,color=C['warn'],  label='WARNING zone')
ax2.fill_between(time,0,650,where=(alert==0),alpha=0.08,color='#00ff88',  label='SAFE zone')
ax2.plot(time,safe_dist,color=C['safe'],lw=2.0,ls='--',label='Dynamic Safe Threshold')
ax2.plot(time,fused,color=C['fused'],lw=1.8,label='Fused Distance')
ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Distance (cm)'); ax2.set_xlim(0,time[-1]); ax2.set_ylim(0,650)
ax2.legend(loc='upper right', facecolor='#14142a', edgecolor='#2a2a4a', labelcolor=TEXT, fontsize=7.5)
style(ax2, 'Graph 2: Alert Zones vs Dynamic Safe Distance Threshold')

# Graph 3
ax3 = fig.add_subplot(gs[1,1])
speeds = np.linspace(0,120,300); sd_curve = [safe_distance(s) for s in speeds]
ax3.fill_between(speeds,0,sd_curve,alpha=0.15,color=C['safe'])
ax3.plot(speeds,sd_curve,color=C['safe'],lw=2.5)
for spd,lbl in [(30,'30 km/h\n79 cm'),(60,'60 km/h\n232 cm'),(80,'80 km/h\n388 cm'),(100,'100 km/h\n587 cm')]:
    sd = safe_distance(spd)
    ax3.plot(spd,sd,'o',color=C['safe'],ms=7)
    ax3.annotate(lbl,xy=(spd,sd),xytext=(spd+4,sd+40),fontsize=7.5,color=TEXT,arrowprops=dict(arrowstyle='->',color=C['safe'],lw=1.2))
ax3.set_xlabel('Vehicle Speed (km/h)'); ax3.set_ylabel('Safe Following Distance (cm)')
ax3.set_xlim(0,120); ax3.set_ylim(0,max(sd_curve)*1.1)
style(ax3, 'Graph 3: d_safe = v x tr + v2/(2a)   [tr=0.7s, a=7.0 m/s2]')

# Graph 4
ax4 = fig.add_subplot(gs[2,0])
x = np.arange(3); maes=[mae_hcsr,mae_lidar,mae_fused]; rmses=[rmse_hcsr,rmse_lidar,rmse_fused]; cols=[C['hcsr'],C['lidar'],C['fused']]
b1=ax4.bar(x-0.2,maes,0.35,color=cols,alpha=0.85,label='MAE (cm)')
b2=ax4.bar(x+0.2,rmses,0.35,color=cols,alpha=0.45,label='RMSE (cm)',hatch='//')
for bar,val in zip(list(b1)+list(b2),maes+rmses):
    ax4.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.3, f'{val:.1f}', ha='center', va='bottom', fontsize=9, color=TEXT, fontweight='bold')
ax4.set_xticks(x); ax4.set_xticklabels(['HC-SR04\nAlone','TF-Luna\nAlone','Kalman\nFused'])
ax4.set_ylabel('Error (cm)'); ax4.set_ylim(0,max(rmses)*1.35)
ax4.legend(facecolor='#14142a',edgecolor='#2a2a4a',labelcolor=TEXT,fontsize=8)
style(ax4, 'Graph 4: Accuracy Comparison — MAE & RMSE')

# Graph 5
ax5 = fig.add_subplot(gs[2,1])
cats=['Static Threshold\n(150 cm fixed)','Dynamic Threshold\n(GPS-adaptive)']; vals=[static_alerts,dynamic_false]; bcols=[C['danger'],'#00cc88']
bars=ax5.bar(cats,vals,color=bcols,alpha=0.85,width=0.45)
for bar,val in zip(bars,vals):
    ax5.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.05, str(val), ha='center', va='bottom', fontsize=14, color=TEXT, fontweight='bold')
ax5.text(0.5,0.6,f'Down {false_reduction:.0f}% reduction\nin false alerts',ha='center',fontsize=11,color='#00ff88',fontweight='bold',transform=ax5.transAxes)
ax5.set_ylabel('False Alert Count'); ax5.set_ylim(0,max(vals)*1.5)
style(ax5, 'Graph 5: False Alert Reduction — Static vs Dynamic Threshold')

fig.suptitle('IoT-Based Automotive Range Detection System — Phase 2 Simulation Results\nAjinkya Bhagwat - Manan Kabra - Rishita Modi  |  NMIT Bengaluru - VTU', color=TEXT, fontsize=13, fontweight='bold', y=0.98)
plt.savefig('simulation_results.png', dpi=180, bbox_inches='tight', facecolor=fig.get_facecolor())
plt.close()

print("=" * 58)
print("  IoT RANGE DETECTION SIMULATION SUMMARY")
print("=" * 58)
print(f"  Samples        : {N} @ 5 Hz ({N*dt:.0f} seconds)")
print(f"  HC-SR04 MAE    : {mae_hcsr:.2f} cm   RMSE: {rmse_hcsr:.2f} cm")
print(f"  TF-Luna MAE    : {mae_lidar:.2f} cm   RMSE: {rmse_lidar:.2f} cm")
print(f"  Kalman MAE     : {mae_fused:.2f} cm   RMSE: {rmse_fused:.2f} cm")
print(f"  False Alerts   : Static={static_alerts}  Dynamic={dynamic_false}  Reduction={false_reduction:.1f}%")
print(f"  SAFE={np.sum(alert==0)}  WARNING={np.sum(alert==1)}  DANGER={np.sum(alert==2)}")
print("=" * 58)
print("  Graph saved as: simulation_results.png")
print("=" * 58)
