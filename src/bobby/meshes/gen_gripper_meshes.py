#!/usr/bin/env python3
"""Generate STL meshes for bobby gripper components from URDF dimensions."""
from build123d import *
import os

MESH_DIR = os.path.dirname(os.path.abspath(__file__))

def save(part, name):
    path = os.path.join(MESH_DIR, name)
    export_stl(part, path, tolerance=0.01, angular_tolerance=0.1)
    print(f"  wrote {path}")

print("Generating gripper meshes — Design A: Classic Industrial...")

# ── link_6: simple cylinder ──────────────────────────────────────────────────
# radius=22mm, height=30mm, centered at origin (URDF origin xyz="0 0 -0.005")
with BuildPart() as p:
    Cylinder(radius=22, height=30)
save(p.part, "link_6.STL")

# ── gripper_flange: clean disc, NO bolt holes (Design A), chamfer top edge ───
# radius=31.5mm, height=10mm (URDF origin xyz="0 0 0.005")
with BuildPart() as p:
    Cylinder(radius=31.5, height=10)
    top_rim = p.edges().filter_by(GeomType.CIRCLE).group_by(Axis.Z)[-1]
    chamfer(top_rim, length=1.2)
save(p.part, "gripper_flange.STL")

# ── gripper_base: 100×30×10mm + T-slot Einlaufkerben an beiden Enden ─────────
# centered at origin (URDF origin xyz="0 0 0.015")
with BuildPart() as p:
    Box(100, 30, 10)
    # Einlaufkerbe links (x=-50 Seite): Box-Ausschnitt 10×12×10
    with Locations((-45, 0, 0)):
        Box(10, 12, 10, mode=Mode.SUBTRACT)
    # Einlaufkerbe rechts (x=+50 Seite)
    with Locations((45, 0, 0)):
        Box(10, 12, 10, mode=Mode.SUBTRACT)
save(p.part, "gripper_base.STL")

# ── finger: 10×20×90mm + symmetrische T-Nut-Führungsnut auf Außenflächen ─────
# centered at origin (URDF origin xyz="0 0 0.045")
# Symmetric → same STL for left and right
with BuildPart() as p:
    Box(10, 20, 90)
    # Führungsnut auf x=-5 Fläche (linker Finger: Außenseite)
    with Locations((-4, 0, 0)):
        Box(2, 6, 88, mode=Mode.SUBTRACT)
    # Führungsnut auf x=+5 Fläche (rechter Finger: Außenseite, symmetrisch)
    with Locations((4, 0, 0)):
        Box(2, 6, 88, mode=Mode.SUBTRACT)
    # Gerade Stirnfläche an Spitze (Design A: keine Fase)
save(p.part, "finger_left.STL")
save(p.part, "finger_right.STL")

print("Done.")
