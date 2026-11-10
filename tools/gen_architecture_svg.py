#!/usr/bin/env python3
"""Generate architecture_diagram.svg — system blueprint of the 1-axis ADCS loop.

Standard library only. Every box, arrow and label is placed on an explicit
grid so text anchors stay aligned at any zoom level.

    python3 tools/gen_architecture_svg.py [output.svg]
"""
import sys
from pathlib import Path
from xml.sax.saxutils import escape

W, H = 1600, 900

BG      = "#0a0c10"
PANEL   = "#11151d"
PANEL_HI = "#151b26"
GRID    = "#161b24"
WHITE   = "#f4f6fa"
MUTED   = "#8b94a7"
DIM     = "#5b6375"
BLUE    = "#21b4ff"
ORANGE  = "#ff8a1e"

SANS = "Inter, 'SF Pro Display', 'Helvetica Neue', Arial, sans-serif"
MONO = "'JetBrains Mono', 'SF Mono', Menlo, Consolas, monospace"

out: list[str] = []


def emit(s: str) -> None:
    out.append(s)


def text(x, y, s, size=14, fill=WHITE, weight=400, anchor="start", family=SANS,
         spacing=0.0, opacity=1.0):
    emit(f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" '
         f'font-weight="{weight}" fill="{fill}" text-anchor="{anchor}" '
         f'letter-spacing="{spacing}" opacity="{opacity}">{escape(s)}</text>')


def box(x, y, w, h, stroke, fill=PANEL, title="", lines=(), glow=False, tag=""):
    """Component box: title centred at the top, detail lines centred below."""
    f = ' filter="url(#glow)"' if glow else ""
    emit(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="{fill}" '
         f'stroke="{stroke}" stroke-width="1.6"{f}/>')
    # accent bar along the top edge
    emit(f'<rect x="{x + 1}" y="{y + 1}" width="{w - 2}" height="4" rx="2" fill="{stroke}" opacity="0.85"/>')
    cx = x + w / 2
    ty = y + 36
    if tag:
        text(cx, y + 26, tag, size=10.5, fill=stroke, weight=600, anchor="middle", family=MONO, spacing=1.5)
        ty += 10
    text(cx, ty, title, size=17, weight=600, anchor="middle", spacing=0.3)
    for i, ln in enumerate(lines):
        text(cx, ty + 26 + i * 21, ln, size=12.5, fill=MUTED, anchor="middle", family=MONO)


def arrow(points, color, label="", label_at=None, label_anchor="middle", dashed=False, width=2.2):
    d = "M " + " L ".join(f"{px} {py}" for px, py in points)
    dash = ' stroke-dasharray="7 6"' if dashed else ""
    marker = "arrow-blue" if color == BLUE else "arrow-orange" if color == ORANGE else "arrow-white"
    emit(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" '
         f'stroke-linecap="round"{dash} marker-end="url(#{marker})" filter="url(#glow-soft)"/>')
    if label:
        knockout_label(label_at, label, color, label_anchor)


def knockout_label(at, label, color, anchor="middle", opacity=1.0):
    """Mono label on a background patch so boundary lines never cross the glyphs."""
    lx, ly = at
    w = 6.95 * len(label) + 10
    x0 = lx - w / 2 if anchor == "middle" else lx - 5 if anchor == "start" else lx - w + 5
    emit(f'<rect x="{x0:.1f}" y="{ly - 12}" width="{w:.1f}" height="17" rx="3" fill="{BG}"/>')
    text(lx, ly, label, size=11.5, fill=color, anchor=anchor, family=MONO, weight=500, opacity=opacity)


def boundary(x, y, w, h, color, path_label, caption):
    emit(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="16" fill="none" stroke="{color}" '
         f'stroke-width="1.4" stroke-dasharray="10 7" opacity="0.75"/>')
    # label notch sits on the top border
    lw = 16 + 10.2 * len(path_label)
    emit(f'<rect x="{x + 24}" y="{y - 14}" width="{lw}" height="28" rx="6" fill="{BG}" stroke="{color}" stroke-width="1.4"/>')
    text(x + 24 + lw / 2, y + 5, path_label, size=15, fill=color, weight=700, anchor="middle", family=MONO)
    # caption gets a background knock-out so the dashed border doesn't strike through it
    cw = 7.6 * len(caption) + 16
    emit(f'<rect x="{x + 24 + lw + 6}" y="{y - 10}" width="{cw}" height="20" fill="{BG}"/>')
    text(x + 24 + lw + 14, y + 4, caption, size=12.5, fill=MUTED, family=MONO)


# ---------------------------------------------------------------------------
emit(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
     f'role="img" aria-labelledby="t d">')
emit('<title id="t">1-axis CubeSat reaction-wheel ADCS architecture</title>')
emit('<desc id="d">Data flow between the sim/ ground-truth harness (plant, mock sensors) and the '
     'freestanding flight/ control software (sensor ingest, complementary filter, inter-frame '
     'smoothing, PID engine with 0.15 degree integral separation, torque-to-PWM mapper).</desc>')
emit("<defs>")
# userSpaceOnUse regions: an objectBoundingBox filter on a perfectly straight
# arrow has zero height and would make the arrow vanish.
emit(f'<filter id="glow" filterUnits="userSpaceOnUse" x="0" y="0" width="{W}" height="{H}">'
     '<feGaussianBlur stdDeviation="5" result="b"/>'
     '<feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter>')
emit(f'<filter id="glow-soft" filterUnits="userSpaceOnUse" x="0" y="0" width="{W}" height="{H}">'
     '<feGaussianBlur stdDeviation="2" result="b"/>'
     '<feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter>')
for name, col in (("arrow-blue", BLUE), ("arrow-orange", ORANGE), ("arrow-white", WHITE)):
    emit(f'<marker id="{name}" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="7" markerHeight="7" '
         f'orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10 z" fill="{col}"/></marker>')
emit(f'<pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse">'
     f'<path d="M 40 0 L 0 0 0 40" fill="none" stroke="{GRID}" stroke-width="1"/></pattern>')
emit("</defs>")

emit(f'<rect width="{W}" height="{H}" fill="{BG}"/>')
emit(f'<rect width="{W}" height="{H}" fill="url(#grid)" opacity="0.55"/>')

# ---- Header -----------------------------------------------------------------
text(40, 58, "CUBESAT ADCS", size=13, fill=BLUE, weight=700, family=MONO, spacing=4)
text(40, 92, "1-Axis Reaction-Wheel Attitude Control — System Architecture", size=28, weight=650)
text(W - 40, 58, "BARE-METAL · 1 kHz · FREESTANDING C++17", size=13, fill=ORANGE, weight=700,
     anchor="end", family=MONO, spacing=2)
text(W - 40, 88, "zero libraries · zero heap · 7.7 ns / tick · 92 B state", size=14, fill=MUTED,
     anchor="end", family=MONO)

# ---- Boundaries -----------------------------------------------------------------
boundary(40, 140, 480, 660, ORANGE, "sim/", "ground-truth harness · float64")
boundary(580, 140, 980, 660, BLUE, "flight/", "MCU firmware · float32 · -ffreestanding")

# ---- sim/ components ------------------------------------------------------------
box(90, 180, 400, 110, ORANGE, title="Mock Gyro (IMU)", tag="SENSOR",
    lines=("int16 · ±500 dps · 65.5 LSB/dps", "0.25 dps bias + white noise"))
box(90, 318, 400, 110, ORANGE, title="Mock Star Tracker", tag="SENSOR",
    lines=("10 Hz absolute fix", "int32 µrad · σ = 50 µrad"))
box(90, 456, 400, 90, ORANGE, title="Hall Tachometer",
    lines=("wheel speed · 1 rpm quantized",))
box(90, 574, 400, 176, ORANGE, fill=PANEL_HI, title="Rigid-Body + Flywheel Plant", tag="PHYSICS · RK4 @ 10 kHz",
    lines=("J_b·ω̇_b = −τ_w + τ_ext",
           "J_w·(ω̇_b + Ω̇) = τ_w − τ_f",
           "τ_w = Kt·clamp((d·V − Ke·Ω)/R, ±2 A)",
           "knock injector → τ_ext impulse"), glow=True)

# truth-state bus inside sim/: plant feeds every sensor
emit(f'<path d="M 90 700 L 66 700 L 66 235" fill="none" stroke="{ORANGE}" stroke-width="1.6" '
     f'stroke-dasharray="4 5" opacity="0.8"/>')
for yy in (235, 373, 501):
    arrow([(66, yy), (88, yy)], ORANGE, width=1.6)
emit(f'<text x="56" y="460" font-family="{MONO}" font-size="11" fill="{ORANGE}" text-anchor="middle" '
     f'transform="rotate(-90 56 460)" letter-spacing="1.5">TRUTH STATE θ, ω_b, Ω</text>')

# ---- flight/ components ---------------------------------------------------------
box(630, 180, 250, 136, BLUE, title="Sensor Ingest", tag="DRIVER ISR",
    lines=("raw LSB → rad/s", "µrad → rad · rpm → rad/s"))
box(920, 180, 280, 136, BLUE, title="Complementary Filter", tag="FUSION",
    lines=("gyro propagate + abs. fix", "online bias observer", "L1 = 0.30 · L2 = 0.05"))
box(1240, 180, 280, 136, BLUE, title="Inter-Frame Smoothing", tag="CORRECTION LAYER",
    lines=("fix innovation spread", "over 100 ticks (0.1 s)", "no 10 Hz staircase"))

# PID engine — the hero block
px, py, pw, ph = 1170, 380, 350, 290
emit(f'<rect x="{px}" y="{py}" width="{pw}" height="{ph}" rx="12" fill="{PANEL_HI}" stroke="{BLUE}" '
     f'stroke-width="2.4" filter="url(#glow)"/>')
emit(f'<rect x="{px + 1}" y="{py + 1}" width="{pw - 2}" height="5" rx="2" fill="{BLUE}"/>')
pcx = px + pw / 2
text(pcx, py + 30, "CONTROL CORE", size=10.5, fill=BLUE, weight=600, anchor="middle", family=MONO, spacing=1.5)
text(pcx, py + 58, "PID Control Engine", size=20, weight=650, anchor="middle")
text(pcx, py + 84, "Kp 4.64 · Ki 6.40 · Kd 0.82", size=13, fill=WHITE, anchor="middle", family=MONO)
# integral-separation callout
emit(f'<rect x="{px + 22}" y="{py + 102}" width="{pw - 44}" height="54" rx="8" fill="{ORANGE}" '
     f'fill-opacity="0.10" stroke="{ORANGE}" stroke-width="1.6"/>')
text(pcx, py + 124, "INTEGRAL SEPARATION", size=12, fill=ORANGE, weight=700, anchor="middle", family=MONO, spacing=1.5)
text(pcx, py + 146, "∫e  active only when |e| < 0.15°", size=13.5, fill=WHITE, weight=500, anchor="middle", family=MONO)
for i, ln in enumerate(("P-error clamp → 0.12 rad/s slew",
                        "D from gyro rate · 20 Hz LPF",
                        "anti-windup · ±20 mN·m limit",
                        "ωn = 8 rad/s · ζ = 0.9")):
    text(pcx, py + 184 + i * 23, ln, size=12.5, fill=MUTED, anchor="middle", family=MONO)

box(680, 560, 420, 160, BLUE, title="Torque → PWM Mapper", tag="ACTUATOR DRIVE",
    lines=("τ_w = −τ_body + friction FF",
           "i = τ_w / Kt · V = iR + Ke·Ω",
           "duty → signed PWM ±4000 counts"))

# ---- Data flow ------------------------------------------------------------------
# gyro -> ingest (crosses boundary)
arrow([(490, 235), (628, 235)], BLUE, "int16 · 1 kHz", (559, 225))
# star tracker -> fusion, routed under the ingest block
arrow([(490, 373), (1060, 373), (1060, 318)], BLUE, "int32 µrad · 10 Hz", (775, 363))
# tach -> mapper
arrow([(490, 501), (890, 501), (890, 558)], BLUE, "int16 rpm", (690, 491))
# pipeline
arrow([(880, 248), (918, 248)], WHITE)
arrow([(1200, 248), (1238, 248)], WHITE)
arrow([(1380, 316), (1380, 378)], WHITE, "θ̂, ω̂ (bias-corrected)", (1392, 352), label_anchor="start")
arrow([(1170, 640), (1102, 640)], WHITE, "τ_body", (1136, 630))
# PWM back into the plant (crosses boundary)
arrow([(680, 662), (492, 662)], ORANGE, "PWM ±4000", (586, 650), width=3)
knockout_label((586, 684), "→ reaction torque", ORANGE, opacity=0.85)

# ---- Footer -----------------------------------------------------------------------
emit(f'<line x1="40" y1="828" x2="{W - 40}" y2="828" stroke="{DIM}" stroke-width="1" opacity="0.6"/>')
stats = [("WORST-CASE SETTLE", "1.32 s", BLUE),
         ("REQUIREMENT", "< 1.5 s", WHITE),
         ("MONTE CARLO", "125 runs", WHITE),
         ("CONTROL / PHYSICS", "1 kHz / 10 kHz", WHITE),
         ("MOMENTUM DRIFT", "1e-14", ORANGE)]
col = (W - 80) / len(stats)
for i, (k, v, c) in enumerate(stats):
    cx = 40 + col * i + col / 2
    text(cx, 852, k, size=11, fill=MUTED, anchor="middle", family=MONO, spacing=1.5)
    text(cx, 880, v, size=19, fill=c, weight=650, anchor="middle", family=MONO)

emit("</svg>")

dest = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent / "architecture_diagram.svg")
dest.write_text("\n".join(out), encoding="utf-8")
print(f"wrote {dest} ({dest.stat().st_size / 1024:.1f} KB)")
