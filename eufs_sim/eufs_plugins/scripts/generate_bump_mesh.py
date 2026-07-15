#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Generate the unit bump mesh used by the terrain field.

The mesh is a raised-cosine dome of unit footprint radius and unit peak height:

    h(r) = 0.5 * (1 + cos(pi * r)),  r in [0, 1]

TerrainField scales one instance per bump by (R, R, H), so this single asset
covers every bump shape. The profile matters: its slope is zero at both r=0 and
r=1, so a bump meets the surrounding asphalt C1-continuously and the car rolls
onto it without an impulse. A spherical cap would not -- and a shallow cap needs
an absurd sphere (a 2 cm high, 45 cm wide cap is the top of a 5 m sphere), which
is why this is a mesh and not a <sphere>.

`TerrainField::height` evaluates the same analytic profile, so the surface the
LiDAR ray-traces and the surface the car drives on are the same surface, up to
the tessellation error bounded below.

The solid is closed: dome on top, a skirt down to z = -SKIRT, and a bottom disc.
The skirt keeps the underside off the z=0 ground plane so the two do not z-fight.

Regenerate with:
    python3 scripts/generate_bump_mesh.py meshes/bump_dome.stl
"""

import math
import struct
import sys

# Every bump is a separate trimesh collision in the world, so the triangle count
# is paid hundreds of times over. These are sized so the tessellation error is
# far below anything that can observe it: a fraction of a millimetre on a 3 cm
# bump, against a LiDAR with 8 mm of range noise.
SECTORS = 16  # around the dome
RINGS = 8  # centre to rim
SKIRT = 0.5  # buried depth, in units of bump height


def profile(r):
    """Unit raised-cosine dome height at radius r (0 outside the footprint)."""
    if r >= 1.0:
        return 0.0
    return 0.5 * (1.0 + math.cos(math.pi * r))


def vertex(ring, sector):
    r = ring / RINGS
    theta = 2.0 * math.pi * sector / SECTORS
    return (r * math.cos(theta), r * math.sin(theta), profile(r))


def facet_normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    n = math.sqrt(nx * nx + ny * ny + nz * nz)
    if n < 1e-12:
        return (0.0, 0.0, 1.0)
    return (nx / n, ny / n, nz / n)


def build_triangles():
    tris = []

    # Dome: a fan at the apex, quads (split into two triangles) further out.
    for s in range(SECTORS):
        s_next = (s + 1) % SECTORS
        apex = (0.0, 0.0, 1.0)
        tris.append((apex, vertex(1, s), vertex(1, s_next)))

    for ring in range(1, RINGS):
        for s in range(SECTORS):
            s_next = (s + 1) % SECTORS
            v00 = vertex(ring, s)
            v01 = vertex(ring, s_next)
            v10 = vertex(ring + 1, s)
            v11 = vertex(ring + 1, s_next)
            tris.append((v00, v10, v11))
            tris.append((v00, v11, v01))

    # Skirt: vertical wall at the rim, dropping below the ground plane.
    for s in range(SECTORS):
        s_next = (s + 1) % SECTORS
        top = vertex(RINGS, s)
        top_next = vertex(RINGS, s_next)
        bot = (top[0], top[1], -SKIRT)
        bot_next = (top_next[0], top_next[1], -SKIRT)
        tris.append((top, bot, bot_next))
        tris.append((top, bot_next, top_next))

    # Bottom disc, wound so its normal points down.
    centre = (0.0, 0.0, -SKIRT)
    for s in range(SECTORS):
        s_next = (s + 1) % SECTORS
        v = vertex(RINGS, s)
        v_next = vertex(RINGS, s_next)
        tris.append((centre, (v_next[0], v_next[1], -SKIRT), (v[0], v[1], -SKIRT)))

    return tris


def write_binary_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"eufs unit bump dome: h(r) = 0.5*(1+cos(pi*r))".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            f.write(struct.pack("<3f", *facet_normal(a, b, c)))
            for v in (a, b, c):
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))


def max_tessellation_error():
    """Peak height error between the tessellated dome and the analytic profile.

    The chord of each ring segment sags below the true surface; the worst case
    is at the ring midpoint. Reported so the caller knows what mismatch it is
    accepting between what the LiDAR sees and what the car drives on.
    """
    worst = 0.0
    for ring in range(RINGS):
        r0, r1 = ring / RINGS, (ring + 1) / RINGS
        mid = 0.5 * (r0 + r1)
        chord = 0.5 * (profile(r0) + profile(r1))
        worst = max(worst, abs(profile(mid) - chord))
    return worst


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "meshes/bump_dome.stl"
    tris = build_triangles()
    write_binary_stl(out, tris)
    err = max_tessellation_error()
    print(f"wrote {out}: {len(tris)} triangles")
    print(f"radial tessellation error <= {err:.5f} of bump height "
          f"({err * 0.02 * 1000:.3f} mm on a 2 cm bump)")


if __name__ == "__main__":
    main()
