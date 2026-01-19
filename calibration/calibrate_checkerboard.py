#!/usr/bin/env python3
import argparse
import glob
import json
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class DetectionResult:
    ok: bool
    corners: Optional[np.ndarray] = None


def load_images(pattern: str) -> List[str]:
    paths = sorted(glob.glob(pattern))
    if not paths:
        raise FileNotFoundError(f"No images matched pattern: {pattern}")
    return paths


def preprocess_gray(img_bgr: np.ndarray, clahe: bool) -> np.ndarray:
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    if not clahe:
        return gray
    # CLAHE often helps with low contrast / uneven lighting
    cla = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return cla.apply(gray)


def find_corners(gray: np.ndarray, pattern_size: Tuple[int, int]) -> DetectionResult:
    """
    pattern_size = (cols, rows) = number of INNER corners.
    Uses findChessboardCornersSB (more robust) with fallback.
    """
    cols, rows = pattern_size

    # Robust detector
    flags_sb = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags_sb)
    if ok and corners is not None:
        return DetectionResult(True, corners)

    # Fallback to classic + subpixel refinement
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags=flags)
    if not ok or corners is None:
        return DetectionResult(False, None)

    # Subpixel refinement
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, winSize=(11, 11), zeroZone=(-1, -1), criteria=term)
    return DetectionResult(True, corners)


def corners_bbox_area(corners: np.ndarray) -> float:
    pts = corners.reshape(-1, 2)
    x0, y0 = pts.min(axis=0)
    x1, y1 = pts.max(axis=0)
    return float(max(0.0, (x1 - x0)) * max(0.0, (y1 - y0)))


def auto_choose_pattern(
    gray_images: List[np.ndarray],
    min_cols: int = 4,
    max_cols: int = 16,
    min_rows: int = 4,
    max_rows: int = 12,
) -> Tuple[int, int]:
    """
    Tries many (cols, rows) and chooses a pattern that:
    - is detected in the most images
    - prefers larger grids
    - prefers detections covering more image area
    """
    candidates = [(c, r) for r in range(min_rows, max_rows + 1) for c in range(min_cols, max_cols + 1)]
    # Try larger patterns first
    candidates.sort(key=lambda p: p[0] * p[1], reverse=True)

    best = None  # (score, (cols, rows), hits)
    for pat in candidates:
        hits = 0
        area_sum = 0.0
        for g in gray_images:
            det = find_corners(g, pat)
            if det.ok:
                hits += 1
                area_sum += corners_bbox_area(det.corners)

        # Weighted score: prioritize hits and larger boards, then area coverage
        n = pat[0] * pat[1]
        score = hits * (n ** 2) + 0.001 * area_sum * n

        if best is None or score > best[0]:
            best = (score, pat, hits)

    if best is None or best[2] == 0:
        raise RuntimeError("Auto pattern search failed: no checkerboard detected in any image.")

    return best[1]


def make_object_points(pattern_size: Tuple[int, int], square_size: float) -> np.ndarray:
    """
    Creates a (N,3) array of 3D points on Z=0 plane in board coordinates.
    square_size sets the real-world scale (meters/mm/etc). Extrinsics will be in same units.
    """
    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), np.float32)
    # Note: OpenCV corners are ordered row-major; mgrid returns (x,y) = (col,row)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_size)
    return objp


def reprojection_error(
    objpoints: List[np.ndarray],
    imgpoints: List[np.ndarray],
    rvecs: List[np.ndarray],
    tvecs: List[np.ndarray],
    K: np.ndarray,
    dist: np.ndarray,
) -> Tuple[float, List[float]]:
    per_view = []
    total_err = 0.0
    total_pts = 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        proj = proj.reshape(-1, 2)
        obs = imgpoints[i].reshape(-1, 2)
        err = np.linalg.norm(obs - proj, axis=1)
        rmse = float(np.sqrt(np.mean(err ** 2)))
        per_view.append(rmse)
        total_err += float(np.sum(err ** 2))
        total_pts += len(err)
    overall = float(np.sqrt(total_err / max(1, total_pts)))
    return overall, per_view


def main():
    ap = argparse.ArgumentParser(description="Camera calibration from checkerboard images (OpenCV).")
    ap.add_argument("--images", required=True, help="Glob pattern, e.g. 'calib/*.jpeg'")
    ap.add_argument("--square-size", type=float, required=True,
                    help="Size of ONE square edge in real units (e.g. 0.02 for 20mm if using meters).")
    ap.add_argument("--pattern", type=int, nargs=2, default=None, metavar=("COLS", "ROWS"),
                    help="Checkerboard INNER corners as COLS ROWS (e.g. 9 6). If omitted, auto-search is used.")
    ap.add_argument("--clahe", action="store_true", help="Apply CLAHE to help corner detection under uneven lighting.")
    ap.add_argument("--rational", action="store_true",
                    help="Enable CALIB_RATIONAL_MODEL (more distortion parameters; needs good data).")
    ap.add_argument("--output", default="calibration.json", help="Output JSON file path.")
    ap.add_argument("--visualize", action="store_true", help="Show detections as you process images.")
    args = ap.parse_args()

    paths = load_images(args.images)

    # Load & preprocess
    bgrs = []
    grays = []
    img_size = None
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f"WARNING: Could not read {p}, skipping.")
            continue
        if img_size is None:
            img_size = (img.shape[1], img.shape[0])  # (w,h)
        else:
            if (img.shape[1], img.shape[0]) != img_size:
                raise ValueError(
                    f"Image size mismatch: {p} is {(img.shape[1], img.shape[0])}, expected {img_size}. "
                    "Calibrate with same resolution images."
                )
        bgrs.append((p, img))
        grays.append(preprocess_gray(img, args.clahe))

    if img_size is None or not grays:
        raise RuntimeError("No valid images loaded.")

    # Choose pattern
    if args.pattern is not None:
        pattern = (int(args.pattern[0]), int(args.pattern[1]))
    else:
        pattern = auto_choose_pattern(grays)
        print(f"[auto] Selected pattern (inner corners): COLS={pattern[0]} ROWS={pattern[1]}")

    # Gather points
    objp = make_object_points(pattern, args.square_size)
    objpoints: List[np.ndarray] = []
    imgpoints: List[np.ndarray] = []
    used_files: List[str] = []

    for (p, img), g in zip(bgrs, grays):
        det = find_corners(g, pattern)
        if not det.ok:
            print(f"NO DETECT: {os.path.basename(p)}")
            continue

        objpoints.append(objp.copy())
        imgpoints.append(det.corners.astype(np.float32))
        used_files.append(os.path.basename(p))

        if args.visualize:
            vis = img.copy()
            cv2.drawChessboardCorners(vis, pattern, det.corners, True)
            cv2.imshow("detections", vis)
            key = cv2.waitKey(250)
            if key == 27:  # ESC
                break

    if args.visualize:
        cv2.destroyAllWindows()

    if len(objpoints) < 8:
        raise RuntimeError(
            f"Only {len(objpoints)} usable images. For stable calibration, aim for 10–25 "
            "with varied tilt/position and the board covering much of the frame."
        )

    # Calibrate
    flags = 0
    if args.rational:
        flags |= cv2.CALIB_RATIONAL_MODEL

    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7)
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None, flags=flags, criteria=term
    )

    overall_reproj, per_view = reprojection_error(objpoints, imgpoints, rvecs, tvecs, K, dist)

    # Prepare output
    views = []
    for fn, rvec, tvec, e in zip(used_files, rvecs, tvecs, per_view):
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = tvec.reshape(3)

        views.append({
            "file": fn,
            "rvec": rvec.reshape(-1).tolist(),
            "tvec": tvec.reshape(-1).tolist(),
            "R": R.tolist(),
            "T_cam_from_board_4x4": T.tolist(),
            "reprojection_rmse_px": float(e),
        })

    out = {
        "image_width": int(img_size[0]),
        "image_height": int(img_size[1]),
        "pattern_inner_corners_cols_rows": [int(pattern[0]), int(pattern[1])],
        "square_size_units": float(args.square_size),
        "rms_opencv": float(rms),
        "reprojection_rmse_px_overall": float(overall_reproj),
        "camera_matrix_K": K.tolist(),
        "distortion_coefficients": dist.reshape(-1).tolist(),
        "views": views,
    }

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)

    print("\n=== Calibration results ===")
    print(f"Used images: {len(views)} / {len(paths)}")
    print(f"Pattern (inner corners): {pattern[0]} x {pattern[1]}")
    print(f"RMS (OpenCV): {rms:.6f}")
    print(f"Reprojection RMSE (overall): {overall_reproj:.6f} px")
    print("\nK (camera matrix):")
    print(np.array(K))
    print("\nDistortion (k1,k2,p1,p2,k3,...):")
    print(dist.reshape(-1))
    print(f"\nSaved: {args.output}")


if __name__ == "__main__":
    main()
