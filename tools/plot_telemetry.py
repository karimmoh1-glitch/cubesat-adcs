#!/usr/bin/env python3
"""Render telemetry_performance.png — 3-panel knock-recovery plot at 300 DPI.

    .venv/bin/python tools/plot_telemetry.py [telemetry/knock_heavy_neg.csv] [out.png]

Reads the CSV written by `make test`. Time is re-zeroed at knock onset, and
the settle time is recomputed from the data with the harness's criterion
(|err| < 0.10 deg and |rate| < 0.05 deg/s, holding to the end of the log).
"""
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
SRC = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "telemetry" / "knock_heavy_neg.csv"
DST = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "telemetry_performance.png"

KNOCK_T = 3.0             # knock onset in the harness scenario [s]
WINDOW = (0.0, 3.0)       # plotted span after the knock [s]
ANGLE_TOL = 0.10          # settle band [deg]
RATE_TOL = 0.05           # settle band [deg/s]
I_ZONE = 0.15             # integral separation threshold [deg]
WORST_CASE = 1.32         # worst settle over all 125 Monte Carlo runs (telemetry/summary.csv)
REQUIREMENT = 1.5
PWM_MAX = 4000
TORQUE_MAX = 20.0         # [mN m]

BLUE, ORANGE, INK, MUTED, GREEN = "#1f6fd1", "#e8661f", "#1b1f27", "#6b7280", "#1a9e6f"
LABEL_BG = dict(boxstyle="square,pad=0.25", fc="white", ec="none")

# ---- Load ----------------------------------------------------------------------
with SRC.open() as f:
    rows = list(csv.DictReader(f))
col = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
t = col["t_s"] - KNOCK_T
err, rate = col["err_deg"], col["rate_true_dps"]
pwm, torque_cmd, i_term = col["pwm"], col["torque_cmd_mNm"], col["i_mNm"]
err_est = -col["theta_est_deg"]          # the error the flight PID actually sees

after = t >= 0
viol = after & ((np.abs(err) > ANGLE_TOL) | (np.abs(rate) > RATE_TOL))
settle = t[viol].max() if viol.any() else 0.0

sel = (t >= WINDOW[0] - 0.15) & (t <= WINDOW[1])
t, err, pwm, torque_cmd, i_term, err_est = (a[sel] for a in (t, err, pwm, torque_cmd, i_term, err_est))
saturated = np.abs(torque_cmd) >= TORQUE_MAX - 1e-3
in_zone = np.abs(err_est) < I_ZONE


def spans(mask):
    """Contiguous True runs of a boolean mask as (t_start, t_end) pairs."""
    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return [(t[a], t[min(b, len(t) - 1)]) for a, b in zip(edges[::2], edges[1::2])]


# ---- Style ---------------------------------------------------------------------
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica Neue", "Helvetica", "Arial", "DejaVu Sans"],
    "font.size": 10,
    "axes.edgecolor": "#c9ccd3",
    "axes.labelcolor": INK,
    "axes.titlesize": 11.5,
    "axes.titleweight": "bold",
    "axes.titlelocation": "left",
    "axes.titlepad": 8,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.color": "#e7e9ee",
    "grid.linewidth": 0.8,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "legend.frameon": False,
    "legend.fontsize": 8.8,
})

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9.2), sharex=True,
                                    gridspec_kw={"height_ratios": [1.25, 1, 1], "hspace": 0.32})
fig.subplots_adjust(left=0.085, right=0.975, top=0.885, bottom=0.07)
fig.text(0.085, 0.955, "Reaction-wheel ADCS — heavy-knock recovery", fontsize=16, weight="bold", color=INK)
fig.text(0.085, 0.925,
         "−6.0 mN·m·s impulse (6.9 °/s kick) at t = 0 · 1 kHz PID · plant with ±5 % motor-parameter mismatch",
         fontsize=10, color=MUTED)

for ax in (ax1, ax2, ax3):
    for a, b in spans(saturated):
        ax.axvspan(a, b, color=ORANGE, alpha=0.07, lw=0)
    ax.axvline(0, color=INK, lw=0.8, alpha=0.5)

# ---- Panel 1: attitude error ------------------------------------------------------
ax1.axhspan(-ANGLE_TOL, ANGLE_TOL, color=GREEN, alpha=0.14, lw=0, label="±0.1° settle threshold")
ax1.plot(t, err, color=BLUE, lw=1.8, label="attitude error (truth)")
ax1.axvline(settle, color=BLUE, lw=1.2, ls="--")
ax1.axvline(WORST_CASE, color=INK, lw=1.4)
ax1.axvline(REQUIREMENT, color="#b42318", lw=1.2, ls=":")
ymax = err.max()
ax1.annotate(f"this run settled\n{settle:.2f} s", xy=(settle, ymax * 0.62), xytext=(-8, 0),
             textcoords="offset points", ha="right", va="center", fontsize=8.8, color=BLUE, bbox=LABEL_BG)
ax1.annotate(f"Worst-Case Convergence\n{WORST_CASE:.2f} s (all 125 runs)", xy=(WORST_CASE, ymax * 0.62),
             xytext=(7, 0), textcoords="offset points", ha="left", va="center", fontsize=8.8,
             color=INK, weight="bold", bbox=LABEL_BG, zorder=5)
ax1.annotate("requirement\n1.5 s", xy=(REQUIREMENT, ymax * 0.30), xytext=(7, 0), textcoords="offset points",
             ha="left", va="center", fontsize=8.8, color="#b42318")
i_pk = int(np.argmax(np.abs(err)))
ax1.annotate(f"peak {err[i_pk]:.2f}°", xy=(t[i_pk], err[i_pk]), xytext=(14, -4),
             textcoords="offset points", fontsize=8.8, color=MUTED)
ax1.set_ylim(-0.2, ymax * 1.12)
ax1.set_ylabel("Attitude error [deg]")
ax1.set_title("1 · Attitude error")
ax1.legend(loc="upper right")

# ---- Panel 2: PWM command ------------------------------------------------------
ax2.axhline(PWM_MAX, color=MUTED, lw=1, ls="--")
ax2.axhline(-PWM_MAX, color=MUTED, lw=1, ls="--")
ax2.text(WINDOW[1], -PWM_MAX, "−4000 PWM rail  ", ha="right", va="bottom", fontsize=8.3, color=MUTED)
ax2.text(WINDOW[1], PWM_MAX, "+4000 PWM rail  ", ha="right", va="top", fontsize=8.3, color=MUTED)
ax2.step(t, pwm, where="post", color=ORANGE, lw=1.3, label="PWM command [counts]")
pk = int(np.argmin(pwm))
ax2.annotate("torque-limited\n(2 A current clamp, 20 mN·m)", xy=(t[pk], pwm[pk]), xytext=(28, -6),
             textcoords="offset points", fontsize=8.8, color=ORANGE, va="center",
             arrowprops=dict(arrowstyle="-", color=ORANGE, lw=0.8))
ax2.set_ylim(-PWM_MAX * 1.12, PWM_MAX * 1.12)
ax2.set_ylabel("PWM [counts]")
ax2.set_title("2 · Actuator command (shaded: wheel at max torque)")

# ---- Panel 3: integrator ---------------------------------------------------------
for a, b in spans(in_zone):
    ax3.axvspan(a, b, ymin=0, ymax=0.03, color=GREEN, alpha=0.9, lw=0)
ax3.plot(t, i_term, color=GREEN, lw=1.8, label="integral term [mN·m]")
ax3.axhline(0, color=MUTED, lw=0.8)
frozen = (t >= 0) & ~in_zone
if frozen.any():
    f0, f1 = t[frozen].min(), t[frozen].max()
    xa = ax3.get_xaxis_transform()       # x in data, y in axes fraction
    ax3.annotate("", xy=(f0, 0.62), xytext=(f1, 0.62), xycoords=xa, textcoords=xa,
                 arrowprops=dict(arrowstyle="<->", color=INK, lw=0.9, shrinkA=0, shrinkB=0))
    ax3.text((f0 + f1) / 2, 0.66, "|e| > 0.15°: integrator frozen\n(holds pre-knock value)", transform=xa,
             ha="center", va="bottom", fontsize=8.8, color=INK)
    ax3.annotate("enters 0.15° zone:\nintegrator re-engages", xy=(f1, i_term[np.searchsorted(t, f1)]),
                 xytext=(14, 18), textcoords="offset points", fontsize=8.8, color=GREEN,
                 arrowprops=dict(arrowstyle="->", color=GREEN, lw=0.9))
ax3.text(0.99, 0.05, "green bar: |e| < 0.15°, integration enabled", transform=ax3.transAxes,
         ha="right", va="bottom", fontsize=8, color=MUTED)
ax3.set_ylabel("Integral term [mN·m]")
ax3.set_xlabel("Time since knock [s]")
ax3.set_title("3 · Integrator (integral separation)")
ax3.set_xlim(-0.15, WINDOW[1])

fig.text(0.975, 0.012, "controller: flight/pid.hpp · data: telemetry/" + SRC.name,
         ha="right", fontsize=7.5, color=MUTED)

fig.savefig(DST, dpi=300, facecolor="white")
print(f"wrote {DST}  (settle this run: {settle:.3f} s)")
