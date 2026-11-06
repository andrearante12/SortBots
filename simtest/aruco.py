"""ArUco markers on the sim's magnet plates. Pure python: Isaac's venv has no cv2 to ask.

A marker is printed on the plate, so finding it finds the magnet's side, and its id says which
package this is: the two things depth geometry cannot tell (nodes/box_geometry.py). The sim
builds it from flat squares rather than a texture: a white backing, and a black square per
dark cell, children of the package so they ride along with it. To a camera that is a print.

DICT_4X4_50, 4x4 data cells inside a one-cell black border. The white quiet zone round the
border is not decoration: on a black plate the detector has no edge to find without it.
"""
from __future__ import annotations

from . import world

MARKER_M = 0.05         # the black border's outer edge: what nodes/package_vision.py measures against
QUIET_M = 0.005         # white round it
BACKING_M = MARKER_M + 2 * QUIET_M
BACKING_PROUD_M = 0.0005    # above the plate
CELLS_PROUD_M = 0.001       # above the plate, and so above the backing, so neither z-fights
CELLS_PER_SIDE = 6

# Data cells of DICT_4X4_50, row by row from the top, "1" white. tests/aruco_test.py holds this
# against cv2.aruco.generateImageMarker (drawMarker before OpenCV 4.7).
DATA = {
    0: ("1011", "0101", "0011", "0010"),
    1: ("0000", "1111", "1001", "1010"),
    2: ("0011", "0011", "0010", "1101"),
    3: ("1001", "1001", "0100", "0110"),
    4: ("0101", "0100", "1001", "1110"),
    5: ("0111", "1001", "1100", "1101"),
    6: ("1001", "1110", "0010", "1110"),
    7: ("1100", "0100", "1111", "0010"),
    8: ("1111", "1110", "1101", "1010"),
    9: ("1100", "1111", "0101", "0110"),
}

NORMALS = {"+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0), "+y": (0.0, 1.0, 0.0),
           "-y": (0.0, -1.0, 0.0), "top": (0.0, 0.0, 1.0)}


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def face_axes(face: str) -> tuple:
    """(right, up, normal) of a face as seen from outside looking at it. Getting `right` the
    wrong way round mirrors the print, and a mirrored marker does not decode."""
    n = NORMALS[face]
    up = (1.0, 0.0, 0.0) if face == "top" else (0.0, 0.0, 1.0)
    return _cross(up, n), up, n


def _square(size, face, u0, v0, u1, v1, proud) -> list:
    """The corners of a rectangle on the plate, package frame, counter-clockwise from outside."""
    psize, centre = world.plate_geometry(size, face)
    right, up, n = face_axes(face)
    lift = world.PLATE_THICKNESS_M / 2 + proud
    return [tuple(centre[i] + n[i] * lift + right[i] * u + up[i] * v for i in range(3))
            for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]


def backing(size, face) -> list:
    h = BACKING_M / 2
    return _square(size, face, -h, -h, h, h, BACKING_PROUD_M)


def dark_cells(size, face, marker: int) -> list:
    """One corner list per black cell of the marker."""
    rows = ("0" * CELLS_PER_SIDE, *(f"0{r}0" for r in DATA[marker]), "0" * CELLS_PER_SIDE)
    s = MARKER_M / CELLS_PER_SIDE
    half = MARKER_M / 2
    out = []
    for r, row in enumerate(rows):
        for c, bit in enumerate(row):
            if bit == "0":
                u, v = -half + c * s, half - r * s
                out.append(_square(size, face, u, v - s, u + s, v, CELLS_PROUD_M))
    return out
