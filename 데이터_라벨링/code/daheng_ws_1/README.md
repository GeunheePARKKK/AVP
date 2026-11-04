# Daheng MER2-240-159U3C — checkerboard intrinsic calibration

Camera: Daheng Imaging MER2-240-159U3C (USB3 Vision, sn `FHH26070138`), 2048×1200 colour.
Board: **8 × 9 inner corners, 125 mm squares** (a 9 × 10-square board) — see `board.json`.

## Quick start

```powershell
python dhcam.py                          # sanity check: opens the camera, grabs a frame
python detect_board_size.py --write      # confirm the board's inner-corner count
python capture_board.py                  # live capture, auto-grabs ~20 distinct views
python calibrate.py                      # solve intrinsics, write results/
```

## 0. Confirm the board — `detect_board_size.py`

**Run this once for any new board.** Getting the inner-corner count wrong is the
single most expensive mistake here, and it is silent: OpenCV will happily lock onto
a smaller sub-grid of a larger board, land on a *different* sub-grid in each view,
and calibration then fits nonsense. The first run on this camera was configured as
9 × 6 against a board that is actually 10 × 9, which produced `fx` 6477 / `fy` 8550
and 24.5 px RMS before the real size was found.

Count **inner corners** (where four squares meet), not squares — an 11 × 10-square
board has 10 × 9 inner corners.

```powershell
python detect_board_size.py --write            # live preview, writes board.json
python detect_board_size.py captures\x.png     # or analyse an existing image
```

Live mode opens a preview window with a framing guide; press `SPACE` when the
**whole** board, white border included, is inside the box. A cut-off board reports a
smaller grid than it really has. After analysis it draws the detected grid so you can
confirm it covers the entire board rather than part of it.

It takes about 5 s. `CALIB_CB_LARGER` proposes candidate sizes — it is not
trustworthy alone (on one frame it returned 5×9, 9×9, 9×5 and 10×9 depending on the
seed size and flags), so every candidate is confirmed with an ordinary fixed-size
detection, and sizes above the largest candidate are probed too. The search runs at
1024 px: measured ~170 ms per size tried versus ~400 ms at full resolution, with
*better* true-size recall and no false positives at either scale.

It cannot infer `square_size_mm` — measure one square with a ruler.

### Never put tape on the checker pattern

Flattening the board is right, but the tape has to stay off the squares. Measured on
a real capture by compositing tape over known corner positions:

| what is covered | result |
|---|---|
| nothing | detected |
| tape on the white margin, or on border squares outside the outermost inner corners | detected |
| a thin strip ≤10 px crossing the pattern | detected |
| **1 inner corner** | **detected in 5 of 10 placements — unreliable** |
| **2 or more inner corners** | **failed in every configuration tried** |

Detection is all-or-nothing: a covered corner does not cost you that corner, it costs
the entire view. Tape the **white margin** or the **back** of the board, or mount it
to a rigid panel with tape only around the outside edge.

`snap.py` tells you which case you are in.

## Board specification

Current target: **8 × 9 inner corners (9 × 10 squares), 125 mm squares**, printed on a
single sheet and bonded to a rigid panel.

| squares | pattern | sheet (one square of quiet zone) | paper | working distance |
|---|---|---|---|---|
| 30 mm | 330 × 300 mm | 390 × 360 mm | A2, one sheet | ~1.2 m |
| 40 mm | 440 × 400 mm | 520 × 480 mm | A1, one sheet | ~1.6 m |
| 60 mm | 660 × 600 mm | 780 × 720 mm | A0 / plotter | ~2.4 m |
| **125 mm (current)** | 1125 × 1250 mm | 1375 × 1500 mm | 60″ (1524 mm) plotter roll | **~5.0 m** |

The first three rows are for the earlier 10 × 9 board; the last is the current
9 × 10-square one. Working distance is set so the sheet spans about **half the frame
height**. Height is the binding dimension — these boards are nearly square while the
sensor is 1.71, so the board runs out of vertical room first. Sizing by frame *width*
gives distances that are physically impossible: at 5.0 m the current sheet is 549 px
wide, only 27 % of the 2048 px frame.

**The 125 mm board needs about 6 m of clear floor and cannot be printed on A0.** At
1375 × 1500 mm it fits a 60″ roll with the 1375 mm dimension across the web, and
nothing narrower — a 36″ or 42″ plotter forces tiling, which is the exact
non-planarity that held the second attempt at 1.99 px. Confirm the print shop's roll
width before ordering. A 1.4 × 1.5 m rigid panel is also a two-person object to tilt
20–40° by hand; plan a stand rather than hand-holding it.

### Why half the frame height

Measured on synthetic views rendered from a known K and distortion, repeated over four
angle sets:

| frame fill | views | RMS | mean \|fx err\| | mean \|cx err\| | mean \|k1 err\| |
|---|---|---|---|---|---|
| **50 %** | 6–7 | 0.04 px | **0.77 px** | **0.54 px** | **0.0005** |
| 35 % | 7–8 | 0.05 px | 2.76 px | 0.96 px | 0.0015 |
| 22 % | 8–9 | 0.09 px | 7.02 px | 14.86 px | 0.0049 |

The 22 % rows had *more* usable views and a *lower* RMS, yet parameter errors an order
of magnitude worse — **RMS is not a measure of accuracy**. Above ~60 % the board starts
clipping when tilted near the frame corners, so half height is the practical optimum.

### Physical size only sets the distance

At a fixed frame fill, a 30 mm board at 1.2 m and a 125 mm board at 5.0 m put the same
number of pixels on each square. Physical size does not change the achievable accuracy
— it changes how far away you stand.

That matters when the camera is destined for a vehicle: **calibrate at the focus setting
you will actually run**, because refocusing changes K. A larger board lets you shoot
from farther away, closer to a driving focus setting. Lock focus and aperture before
capturing and do not touch them afterwards.

### Print and mount

- Print at **100 % / actual size** — "fit to page" silently rescales.
- One sheet. Tiled sheets introduce steps at the seams, which is exactly the
  non-planarity that held the second attempt at 1.99 px.
- **Matte** paper, 160–200 g/m². Gloss reflects.
- Keep **one square of white quiet zone** outside the outermost inner corners.
- Bond the **whole back** (spray adhesive or full-area double-sided tape) to acrylic,
  MDF or foam board, squeegeeing bubbles out from the centre. Corner-only mounting
  lets the paper ripple with humidity.

### Then measure the squares

Measure **outermost inner corner to outermost inner corner** — not the board's outer
edge, which includes the quiet zone. Across the 9-corner axis that span is 8 squares
(~1000 mm here), so divide by 8; cross-check on the 8-corner axis, which is 7 squares.
Measuring the whole span rather than one square cuts the ruler error by the same factor.

The 125.0 mm in `board.json` is measured. It is consistent with the solved geometry:
the 30 calibration views sit at 2.34–4.46 m from the camera under that scale, which
matches how they were shot.

`square_size_mm` does **not** affect K or the distortion coefficients. Verified on this
camera's own data: calibrating the same corner set as 60 / 30 / 137 mm changed `fx` by
6e-6 px and the distortion terms by 2e-7, while `tvec` scaled by exactly the size ratio.
It matters later, when the camera-to-vehicle extrinsics are solved: a 1 % scale error
becomes a 1 % error in every distance the system reports.

## 1. Capture — `capture_board.py`

Opens a live preview with a coverage HUD. Auto-capture is on by default: a view is
kept only when the board is detected, sharp enough, and sufficiently different from
everything already saved.

| key | action |
|---|---|
| `SPACE` | grab now (bypasses the sharpness/novelty gates) |
| `A` | toggle auto-capture |
| `U` | delete the last saved view |
| `Q` / `ESC` | finish |

The 3 × 3 grid overlay turns green where the board has already been seen. **Fill all
nine cells.** Distortion coefficients are driven almost entirely by corners near the
frame edges — a set of views that all sit in the middle yields a low RMS that is
nonetheless wrong at the periphery.

Also vary the angle: aim for roughly half the views tilted 20–40° out of plane
(the HUD's `tilted` counter tracks this). Fronto-parallel views alone leave focal
length and board distance correlated, so `fx` drifts.

Useful flags:

```powershell
python capture_board.py --manual              # SPACE-only, no auto-capture
python capture_board.py --target 30
python capture_board.py --preview-width 640   # faster preview, slightly worse detect range
python capture_board.py --exposure 8000 --gain 4   # fix exposure if auto hunts
python capture_board.py --auto-exposure-max 40000  # dim room, accepts a slower preview
```

Images land in `captures/`. Re-running appends; delete the folder for a clean set.

### Frame rate

The HUD shows live fps. Expect **~13 fps** at the default 800 px preview; detection
is the bulk of it. Two things matter here:

- **Auto-exposure is capped at 20 ms** (`--auto-exposure-max`). Left uncapped it
  settles near 70 ms in ordinary indoor light, which alone caps acquisition at 14 fps
  and smears a hand-held board. Auto-gain makes up the light.
- The live overlay uses `findChessboardCornersSB` without the exhaustive/accuracy
  flags. The classic detector with `CALIB_CB_FAST_CHECK` measured ~2.5× *slower* on
  these frames, not faster, despite the flag's name.

The preview detector only decides *when* to grab; the corners that reach
`calibrate.py` are always re-detected at full resolution with sub-pixel refinement.

If the startup line reports gain near 16 dB, the room is too dim — the images will be
noisy and corner localisation suffers. Add light rather than raising the exposure cap.

## 2. Solve — `calibrate.py`

```powershell
python calibrate.py                # 5-coefficient plumb-bob model
python calibrate.py --prune 0.4    # drop outlier views, then re-solve
python calibrate.py --rational     # 8 coefficients (k1..k6), strong wide-angle lenses
```

Prints RMS reprojection error, `K`, distortion, focal length in mm, FOV, and a
per-view error table. Writes:

```
results/calibration.yaml     OpenCV FileStorage — cv2.FileStorage(path, READ)
results/calibration.json     same values plus per-view errors and metadata
results/undistorted/         before/after pairs (left original, right undistorted)
```

**Always read the per-view table.** A single mis-localised corner can push one view
to several px while the rest sit near 0.1 px, and that outlier biases the whole
solution — `calibrate.py` flags it and suggests a `--prune` threshold. Pruning and
re-solving is normal practice, not a workaround.

Rough reading of the RMS: `< 0.5 px` good, `0.5–1.0 px` re-check the worst views,
`> 1.0 px` something is structurally wrong (wrong `square_size_mm`, motion blur, or
a board printed on paper that is not perfectly flat — tape it to glass or foamboard).

The square size only scales `tvec` and the focal-length-in-mm readout; `K` in pixels
and the distortion coefficients are unaffected by getting it wrong.

## 3. Using the result

```python
import cv2, numpy as np
fs = cv2.FileStorage("results/calibration.yaml", cv2.FILE_STORAGE_READ)
K = fs.getNode("camera_matrix").mat()
D = fs.getNode("distortion_coefficients").mat()
fs.release()

undistorted = cv2.undistort(frame, K, D)
```

## Files

| file | role |
|---|---|
| `board.json` | board geometry — edit if you switch boards |
| `detect_board_size.py` | sweeps grid sizes to find the board's true corner count |
| `snap.py` | grab one frame and diagnose why detection failed |
| `board.py` | board config + corner detection shared by both scripts |
| `dhcam.py` | Galaxy SDK wrapper, yields BGR numpy frames |
| `capture_board.py` | live capture with coverage/sharpness/novelty gating |
| `calibrate.py` | `cv2.calibrateCamera` + reporting + export |
| `vendor/gxipy/` | patched copy of the SDK's Python API (see `vendor/README.md`) |

## Verification

The pipeline was checked against synthetic views rendered with a known `K` and
distortion: after `--prune`, it recovered `fx` 2181.8 vs 2180.0 true, `cx` 1015.1 vs
1015.0, `k1` −0.1761 vs −0.1750, at 0.046 px RMS. `k3` is weakly constrained and
should not be expected to match — use `--fix-k3` if it comes out wild.
