"""Generates doc/apriltags/apriltags_handeye_and_holders.pdf.

Committed alongside the PDF because the tag ids and their physical
sizes are what the calibration and the localization code will assume,
and a reprint that quietly changes either is a wrong measurement, not
a failed one.

Run with the pydm env, which carries opencv and matplotlib:
    ~/micromamba/envs/pydm/bin/python doc/apriltags/generate.py

Sizes come from the D405: fx ~674 px, so a tag spans size_mm * 674 /
distance_mm pixels. tag36h11 needs ~30 px to detect and ~60 px before
its pose is worth trusting, which is why the holder tags are only good
from 70-150 mm and the hand-eye target is 100 mm.
"""

import cv2, numpy as np, matplotlib, os
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle

D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
MM = 1/25.4

def tag_img(i, cells_px=40):
    return cv2.aruco.generateImageMarker(D, i, 8*cells_px)

def place(ax, i, size_mm, x_mm, y_mm, lines, quiet_cells=1.0):
    cell = size_mm/8.0; q = quiet_cells*cell
    ax.add_patch(Rectangle((x_mm-q, y_mm-q), size_mm+2*q, size_mm+2*q,
                           facecolor="white", edgecolor="none", zorder=1))
    ax.imshow(tag_img(i), cmap="gray", vmin=0, vmax=255, zorder=2,
              extent=(x_mm, x_mm+size_mm, y_mm, y_mm+size_mm), interpolation="nearest")
    for k, t in enumerate(lines):
        ax.text(x_mm+size_mm/2, y_mm-q-1.8-k*3.4, t, ha="center", va="top",
                fontsize=6, family="monospace", zorder=3,
                weight="bold" if k == 0 else "normal")
    ax.add_patch(Rectangle((x_mm-q, y_mm-q), size_mm+2*q, size_mm+2*q,
                           facecolor="none", edgecolor="0.75", lw=0.3,
                           linestyle=(0,(2,2)), zorder=3))

def page(pdf, title, note, draw):
    fig = plt.figure(figsize=(210*MM, 297*MM))
    ax = fig.add_axes([0,0,1,1]); ax.set_xlim(0,210); ax.set_ylim(0,297)
    ax.set_aspect("equal"); ax.axis("off")
    ax.text(15, 285, title, fontsize=11, weight="bold", family="monospace")
    ax.text(15, 279, note, fontsize=6.5, family="monospace", color="0.3", va="top")
    y = 20
    ax.plot([15,115],[y,y], color="k", lw=0.8)
    for k in range(11):
        x = 15+k*10
        ax.plot([x,x],[y,y+(3 if k%5 else 5)], color="k", lw=0.8)
    ax.text(15, y-4, "0", fontsize=5, family="monospace")
    ax.text(115, y-4, "100 mm", fontsize=5, family="monospace", ha="right")
    ax.text(120, y, "<- must measure exactly 100 mm.\n   If not, the print was scaled.",
            fontsize=5.5, family="monospace", va="center", color="0.3")
    draw(ax); pdf.savefig(fig); plt.close(fig)

out = "doc/apriltags/apriltags_handeye_and_holders.pdf"
with PdfPages(out) as pdf:
    def p1(ax):
        place(ax, 0, 100.0, 55, 150, ["id = 0", "100.0 mm", "HAND-EYE / BASE"])
        ax.text(15, 128, "\n".join([
            "Hand-eye calibration target. Fix it to the floor or table so it cannot move",
            "while the arm is moved through calibration poses.",
            "",
            "  - The black border is part of the tag: 100.0 mm is the OUTER black square.",
            "  - Keep the white margin around it clear (>= 12.5 mm).",
            "  - Mount it flat. A bulge of 1 mm over 100 mm is already 0.6 deg of pose error.",
            "  - D405 at 300 mm sees this as ~225 px across: plenty for pose.",
        ]), fontsize=6.5, family="monospace", va="top", linespacing=1.6)
    page(pdf, "AprilTag 36h11 - hand-eye target (100 mm)",
         "Print at 100% / actual size. Do NOT use 'fit to page' or 'shrink to fit'.", p1)

    def p2(ax):
        x0, y0, pitch, rowgap = 22, 240, 36, 34
        for k in range(10):
            r, c = divmod(k, 5)
            place(ax, 1+k, 10.0, x0+c*pitch, y0-r*rowgap,
                  [f"id = {1+k}", f"slot {k+1}"])
        place(ax, 11, 10.0, x0, y0-2*rowgap-6, ["id = 11", "MEAS.", "HOLDER"])
        ax.text(15, 150, "\n".join([
            "Rack slot tags (id 1-10) and the measurement holder tag (id 11), 10.0 mm.",
            "Every tag carries a distinct id, so a detection identifies which holder it is.",
            "",
            "  10 mm is usable from 70 to ~150 mm only (96 px down to 45 px on a D405).",
            "  Beyond ~200 mm detection becomes unreliable and pose is unusable.",
            "  That is fine for close-up localization; it is NOT enough for hand-eye,",
            "  which is why the 100 mm tag on page 1 exists.",
            "",
            "  Each cell is 1.25 mm. Print on a 600 dpi laser printer or better;",
            "  inkjet bleed rounds the corners and biases the detected pose.",
            "  Matte paper. Glossy or laminated surfaces blow out under the D405 projector.",
        ]), fontsize=6.5, family="monospace", va="top", linespacing=1.6)
    page(pdf, "AprilTag 36h11 - holder tags (10 mm)",
         "Print at 100% / actual size. Cut along the dashed line to keep the quiet zone.", p2)

    def p3(ax):
        x = 20
        for size, idx in [(10.0,100),(15.0,101),(20.0,102),(25.0,103),(30.0,104)]:
            place(ax, idx, size, x, 200, [f"id = {idx}", f"{size:.0f} mm"])
            x += size + 18
        ax.text(15, 175, "\n".join([
            "Trial strip. Before committing to 10 mm, tape these where a holder tag would go",
            "and check at the distance the wrist camera actually observes from.",
            "",
            "  Expected pixel size on a D405 (fx ~674 px):",
            "",
            "        dist |  10mm   15mm   20mm   25mm   30mm",
            "      -------+-----------------------------------",
            "        70mm |   96     144    193    241    289",
            "       100mm |   67     101    135    169    202",
            "       150mm |   45      67     90    112    135",
            "       200mm |   34      51     67     84    101",
            "       300mm |   22      34     45     56     67",
            "",
            "  Rule of thumb: >= 30 px to detect, >= 60 px before trusting the pose.",
            "  Pick the smallest size that still clears 60 px at your working distance.",
        ]), fontsize=6.5, family="monospace", va="top", linespacing=1.6)
    page(pdf, "AprilTag 36h11 - size trial strip",
         "Print at 100% / actual size. Use this to choose the holder tag size empirically.", p3)
print(f"{out} ({os.path.getsize(out):,} B)")
