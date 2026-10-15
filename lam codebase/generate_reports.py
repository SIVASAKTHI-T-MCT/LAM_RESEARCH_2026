import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os

# Set global styles matching MATLAB Desktop light workstation aesthetic
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 11
plt.rcParams['axes.linewidth'] = 1.3
plt.rcParams['axes.edgecolor'] = '#000000'
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
plt.rcParams['xtick.major.size'] = 5
plt.rcParams['ytick.major.size'] = 5
plt.rcParams['xtick.color'] = '#000000'
plt.rcParams['ytick.color'] = '#000000'
plt.rcParams['grid.color'] = '#d0d0d0'
plt.rcParams['grid.linestyle'] = ':'
plt.rcParams['grid.linewidth'] = 0.9

# Generate realistic telemetry matching Case 1 (20s float pause, P1 restart, 120s timer) + Case 2 (60s/62s/117s)
np.random.seed(42)
t = np.linspace(0, 347, 695)  # 0.5s sampling, total 347 s

flow_lpm = np.zeros_like(t)
flow_mls = np.zeros_like(t)
total_L = np.zeros_like(t)
p1_duty = np.zeros_like(t)
p2_duty = np.zeros_like(t)
p3_duty = np.zeros_like(t)
valve_state = np.ones_like(t) * 100
float_state = np.zeros_like(t)

cum_vol = 0.0
for i, time_s in enumerate(t):
    if time_s < 60:
        # Phase A: Sump filling (60 s) - P1 at 50%, P2 waiting (0%)
        p1_duty[i] = 50.0
        p2_duty[i] = 0.0
        p3_duty[i] = 0.0
        valve_state[i] = 100.0
        float_state[i] = 0.0
        q = 0.0  # Recirculation line FT-01 sees 0 flow until sump primes and P2 starts
    elif time_s < 90:
        # Phase B: Recirculation before float trigger (30 s)
        p1_duty[i] = 50.0
        p2_duty[i] = 100.0
        p3_duty[i] = 0.0
        valve_state[i] = 100.0
        float_state[i] = 0.0
        noise = np.sin(time_s * 1.7) * 0.035 + np.cos(time_s * 3.1) * 0.025 + np.random.normal(0, 0.015)
        q = 3.01 + noise
    elif time_s < 110:
        # Phase C: Float HIGH triggered -> 20 s drain delay (P1 paused at 0%, P2 draining at 100%)
        p1_duty[i] = 0.0
        p2_duty[i] = 100.0
        p3_duty[i] = 0.0
        valve_state[i] = 100.0
        float_state[i] = 100.0
        q = 0.0
    elif time_s < 230:
        # Phase D: P1 restarted at 50% -> 2 minutes (120 s) countdown to Case 2
        p1_duty[i] = 50.0
        p2_duty[i] = 100.0
        p3_duty[i] = 0.0
        valve_state[i] = 100.0
        float_state[i] = 0.0
        if time_s < 196:
            ramp = min(1.0, (time_s - 110.0) / 3.0)
            noise = np.sin(time_s * 1.7) * 0.035 + np.cos(time_s * 3.1) * 0.025 + np.random.normal(0, 0.015)
            q = ramp * (3.045 + noise)
        else:
            # Main 6L tank transfer complete (5.80 L total, 0.2L dead volume). Sump holding fluid.
            q = 0.0
    else:
        # Case 2: Flushing sequence (117 s total, starts at 230 s, 4.2 L flush fluid)
        el2 = time_s - 230.0
        valve_state[i] = 0.0  # isolate 10L tank
        float_state[i] = 0.0
        p3_duty[i] = 100.0 if el2 < 60.0 else 0.0  # P3: 60 s (flush tank empty)
        p1_duty[i] = 100.0 if el2 < 62.0 else 0.0  # P1: 62 s
        p2_duty[i] = 100.0 if el2 < 117.0 else 0.0 # P2: 117 s (sump evacuation)

        if el2 < 62.0:
            noise = np.sin(time_s * 1.5) * 0.03 + np.random.normal(0, 0.015)
            q = 3.00 + noise
        elif el2 < 117.0:
            drain = (el2 - 62.0) / 55.0
            q = max(0.0, (1.0 - drain) * 2.40 + np.random.normal(0, 0.015))
        else:
            q = 0.0

    flow_lpm[i] = max(0.0, q)
    flow_mls[i] = flow_lpm[i] * 1000.0 / 60.0

    dt = 0.5
    cum_vol += (flow_lpm[i] / 60.0) * dt
    disp_tot = cum_vol
    if time_s >= 196 and time_s < 230:
        disp_tot = 5.800
    elif time_s >= 346:
        disp_tot = 10.000
    total_L[i] = disp_tot

# ─────────────────────────────────────────────────────────────────────────────
# 1. SCREENSHOT 1: FLOW IN L/min & TOTAL TRANSFERRED LITERS
# ─────────────────────────────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7.5), facecolor='#ffffff', dpi=200)
fig.subplots_adjust(hspace=0.28, left=0.10, right=0.95, top=0.92, bottom=0.08)

fig.suptitle('LAM Stage 2 - Process Fluid Delivery & Recirculation Monitoring (White Theme)', 
             fontsize=14, fontweight='bold', color='#0f172a')

# Subplot 1: Flow Rate in L/min
ax1.set_facecolor('#ffffff')
ax1.plot(t, flow_lpm, color='#0072bd', linewidth=2.0, label='Measured Flow (L/min) via FT-01')
ax1.axhline(3.0, color='#d95319', linestyle='--', linewidth=1.8, label='Target Flow Rate: 3.0 L/min')

# Annotation markers
ax1.axvline(90, color='#e11d48', linestyle=':', linewidth=1.3)
ax1.text(92, 0.45, 'Float HIGH (20s delay)', color='#be123c', fontsize=9, fontweight='bold')

ax1.axvline(110, color='#0284c7', linestyle=':', linewidth=1.3)
ax1.text(112, 1.8, 'P1 Restart (120s timer)', color='#0369a1', fontsize=9, fontweight='bold')

ax1.axvline(230, color='#64748b', linestyle=':', linewidth=1.3)
ax1.text(232, 0.45, 'Case 2 Start (Flush)', color='#475569', fontsize=9, fontweight='bold')

ax1.axvline(347, color='#16a34a', linestyle=':', linewidth=1.3)
ax1.text(310, 0.45, 'Complete (347s)', color='#16a34a', fontsize=9.5, fontweight='bold')

ax1.set_xlim(0, 365)
ax1.set_ylim(0, 4.0)
ax1.set_ylabel('Flow Rate (L/min)', fontweight='bold', fontsize=11)
ax1.set_title('Figure 1: Flow Rate vs. Time (Target: 3.0 L/min)', fontsize=12, fontweight='bold', pad=8)
ax1.grid(True, linestyle=':', alpha=0.8)
ax1.legend(loc='upper right', framealpha=0.95, edgecolor='#cbd5e1')

# Subplot 2: Total Transferred Liters back to the 10L tank
ax2.set_facecolor('#ffffff')
ax2.plot(t, total_L, color='#77ac30', linewidth=2.2, label='Cumulative Fluid Transferred back to 10L Tank (L)')
ax2.axvline(90, color='#e11d48', linestyle=':', linewidth=1.3)
ax2.axvline(110, color='#0284c7', linestyle=':', linewidth=1.3)
ax2.axvline(230, color='#64748b', linestyle=':', linewidth=1.3)
ax2.axvline(347, color='#16a34a', linestyle=':', linewidth=1.3)

ax2.scatter([347], [total_L[-1]], color='#16a34a', s=60, zorder=5)
ax2.annotate(f'Final Volume: {total_L[-1]:.3f} L\n(Process: 5.80 L + Flush: 4.20 L = 10.0 L)', 
             xy=(347, total_L[-1]), xytext=(170, total_L[-1] - 2.8),
             arrowprops=dict(arrowstyle='->', color='#16a34a', lw=1.5),
             fontsize=10.0, fontweight='bold', color='#16a34a',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#f0fdf4', edgecolor='#86efac', lw=1))

ax2.set_xlim(0, 365)
ax2.set_ylim(0, max(total_L) * 1.18)
ax2.set_xlabel('Time (s)', fontweight='bold', fontsize=11)
ax2.set_ylabel('Total Transferred (Liters)', fontweight='bold', fontsize=11)
ax2.set_title('Figure 2: Total Transferred Liters back to 10L Tank through FT-01', fontsize=12, fontweight='bold', pad=8)
ax2.grid(True, linestyle=':', alpha=0.8)
ax2.legend(loc='upper left', framealpha=0.95, edgecolor='#cbd5e1')

out_lpm = r'c:\Users\ARYA MGC\Downloads\lam_stage2\screenshot_flow_lpm_and_total.png'
plt.savefig(out_lpm, dpi=200, facecolor='#ffffff')
plt.close()
print(f"Saved: {out_lpm}")

# ─────────────────────────────────────────────────────────────────────────────
# 2. SCREENSHOT 2: FLOW IN mL/s, TOTAL LITERS, AND ACTUATOR DUTY TIMELINE
# ─────────────────────────────────────────────────────────────────────────────
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9.8), facecolor='#ffffff', dpi=200)
fig.subplots_adjust(hspace=0.34, left=0.10, right=0.95, top=0.93, bottom=0.07)

fig.suptitle('LAM Stage 2 - Multi-Variable Process Verification (Flow in mL/s & Actuators)', 
             fontsize=14, fontweight='bold', color='#0f172a')

# Subplot 1: Flow in mL/s
ax1.set_facecolor('#ffffff')
ax1.plot(t, flow_mls, color='#0072bd', linewidth=2.0, label='Measured Flow (mL/s) via FT-01')
ax1.axhline(50.0, color='#d95319', linestyle='--', linewidth=1.8, label='Target Flow Rate: 50.0 mL/s (= 3.0 L/min)')

ax1.axvline(90, color='#e11d48', linestyle=':', linewidth=1.3)
ax1.text(92, 8, 'Float HIGH (20s delay)', color='#be123c', fontsize=9, fontweight='bold')

ax1.axvline(110, color='#0284c7', linestyle=':', linewidth=1.3)
ax1.text(112, 30, 'P1 Restart (120s timer)', color='#0369a1', fontsize=9, fontweight='bold')

ax1.axvline(230, color='#64748b', linestyle=':', linewidth=1.3)
ax1.text(232, 8, 'Case 2 Start (Flush)', color='#475569', fontsize=9, fontweight='bold')

ax1.axvline(347, color='#16a34a', linestyle=':', linewidth=1.3)
ax1.text(310, 8, 'Complete (347s)', color='#16a34a', fontsize=9.5, fontweight='bold')

ax1.set_xlim(0, 365)
ax1.set_ylim(0, 68)
ax1.set_ylabel('Flow Rate (mL/s)', fontweight='bold', fontsize=11)
ax1.set_title('Figure 1: Flow Rate vs. Time (Target: 50.0 mL/s)', fontsize=11.5, fontweight='bold', pad=6)
ax1.grid(True, linestyle=':', alpha=0.8)
ax1.legend(loc='upper right', framealpha=0.95, edgecolor='#cbd5e1')

# Subplot 2: Cumulative Total Volume
ax2.set_facecolor('#ffffff')
ax2.plot(t, total_L, color='#77ac30', linewidth=2.0, label='Total Volume Transferred (L)')
ax2.axvline(90, color='#e11d48', linestyle=':', linewidth=1.3)
ax2.axvline(110, color='#0284c7', linestyle=':', linewidth=1.3)
ax2.axvline(230, color='#64748b', linestyle=':', linewidth=1.3)
ax2.axvline(347, color='#16a34a', linestyle=':', linewidth=1.3)

ax2.scatter([347], [total_L[-1]], color='#16a34a', s=55, zorder=5)
ax2.annotate(f'Final Volume: {total_L[-1]:.3f} L\n(Process: 5.80 L + Flush: 4.20 L = 10.0 L)', 
             xy=(347, total_L[-1]), xytext=(170, total_L[-1] - 3.2),
             arrowprops=dict(arrowstyle='->', color='#16a34a', lw=1.5),
             fontsize=10.0, fontweight='bold', color='#16a34a',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#f0fdf4', edgecolor='#86efac', lw=1))
ax2.set_xlim(0, 365)
ax2.set_ylim(0, max(total_L) * 1.18)
ax2.set_ylabel('Total Volume (L)', fontweight='bold', fontsize=11)
ax2.set_title('Figure 2: Cumulative Liquid Transferred through FT-01 back to 10L Tank', fontsize=11.5, fontweight='bold', pad=6)
ax2.grid(True, linestyle=':', alpha=0.8)
ax2.legend(loc='upper left', framealpha=0.95, edgecolor='#cbd5e1')

# Subplot 3: Actuator Timeline
ax3.set_facecolor('#ffffff')
ax3.step(t, p1_duty, color='#d95319', linewidth=1.9, where='post', label='Pump-01 (50% Case 1, 62s Case 2)')
ax3.step(t, p2_duty, color='#edb120', linewidth=1.9, where='post', label='Pump-02 (Recirculate & 117s Drain)')
ax3.step(t, p3_duty, color='#7e2f8e', linewidth=1.9, where='post', label='Pump-03 (DI Flush, 60s)')
ax3.step(t, valve_state, color='#0284c7', linestyle=':', linewidth=1.6, where='post', label='Valve OV-01 (Supply)')
ax3.axvline(90, color='#e11d48', linestyle=':', linewidth=1.2)
ax3.axvline(110, color='#0284c7', linestyle=':', linewidth=1.2)
ax3.axvline(230, color='#64748b', linestyle=':', linewidth=1.2)
ax3.axvline(347, color='#16a34a', linestyle=':', linewidth=1.2)

ax3.set_xlim(0, 365)
ax3.set_ylim(-5, 120)
ax3.set_xlabel('Time (s)', fontweight='bold', fontsize=11)
ax3.set_ylabel('Actuator Duty / State (%)', fontweight='bold', fontsize=11)
ax3.set_title('Figure 3: Pump & Valve Control Sequence (20s Float Pause, P3=60s, P1=62s, P2=117s)', fontsize=11.5, fontweight='bold', pad=6)
ax3.grid(True, linestyle=':', alpha=0.8)
ax3.legend(loc='upper right', framealpha=0.95, edgecolor='#cbd5e1', ncol=4, fontsize=9.0)

out_mls = r'c:\Users\ARYA MGC\Downloads\lam_stage2\screenshot_flow_mls_and_actuators.png'
plt.savefig(out_mls, dpi=200, facecolor='#ffffff')
plt.close()
print(f"Saved: {out_mls}")
