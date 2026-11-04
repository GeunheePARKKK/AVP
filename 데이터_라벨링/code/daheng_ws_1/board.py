"""Checkerboard config + detection helpers shared by capture and calibration."""

import json
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, "board.json")


class Board:
    def __init__(self, cols, rows, square_size_mm):
        self.cols = int(cols)          # inner corners across
        self.rows = int(rows)          # inner corners down
        self.square = float(square_size_mm)

    @property
    def size(self):
        return (self.cols, self.rows)

    @property
    def n_corners(self):
        return self.cols * self.rows

    def object_points(self):
        """3D corner coordinates on the Z=0 board plane, in millimetres."""
        objp = np.zeros((self.rows * self.cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        return objp * self.square

    def __repr__(self):
        return f"Board({self.cols}x{self.rows} inner corners, {self.square:g} mm squares)"


def load_board(path=CONFIG_PATH):
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    return Board(cfg["cols"], cfg["rows"], cfg["square_size_mm"])


_SB_FLAGS = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
_SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)


def to_gray(img):
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def find_corners(img, board, fast=False):
    """Locate the board. Returns Nx1x2 float32 corners, or None.

    ``fast=True`` drops the exhaustive/accuracy refinement -- fine for a live
    overlay, never for the images actually fed to calibrateCamera. Note this is
    still findChessboardCornersSB: the classic detector with CALIB_CB_FAST_CHECK
    measured ~2.5x slower than SB on this camera's frames, not faster.
    """
    gray = to_gray(img)

    if fast:
        ok, corners = cv2.findChessboardCornersSB(gray, board.size, 0)
        return corners if ok else None

    ok, corners = cv2.findChessboardCornersSB(gray, board.size, _SB_FLAGS)
    if ok:
        return corners

    # findChessboardCornersSB is stricter; fall back to the classic detector so a
    # partially glared or steeply tilted view is not thrown away.
    ok, corners = cv2.findChessboardCorners(
        gray, board.size, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    win = max(5, min(11, int(min(gray.shape) / 200)))
    return cv2.cornerSubPix(gray, corners, (win, win), (-1, -1), _SUBPIX_CRIT)


def sharpness(gray, corners):
    """Variance of Laplacian inside the board's bounding box -- blur rejection."""
    pts = corners.reshape(-1, 2)
    x0, y0 = np.floor(pts.min(axis=0)).astype(int)
    x1, y1 = np.ceil(pts.max(axis=0)).astype(int)
    h, w = gray.shape[:2]
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w, x1), min(h, y1)
    if x1 - x0 < 10 or y1 - y0 < 10:
        return 0.0
    return float(cv2.Laplacian(gray[y0:y1, x0:x1], cv2.CV_64F).var())
