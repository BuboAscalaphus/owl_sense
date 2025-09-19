#!/usr/bin/env python3
import argparse
import json
import signal
import sys
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}

# ---- util --------------------------------------------------------------

def eprint(*a, **k):
    print(*a, file=sys.stderr, **k)

def find_images(folder: Path) -> List[Path]:
    return [p for p in sorted(folder.iterdir()) if p.is_file() and p.suffix.lower() in IMAGE_EXTS]

def collect_object_points(board_size: Tuple[int, int], square_size: float) -> np.ndarray:
    cols, rows = board_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_size)
    return objp

def try_detect_corners(gray: np.ndarray, board_size: Tuple[int, int]):
    flags_fast = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ret, corners = cv2.findChessboardCorners(gray, board_size, flags_fast)
    if not ret:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, board_size, flags)
    return ret, corners

def iter_target_dirs(root: Path) -> List[Path]:
    dirs = []
    # include root solo se contiene immagini
    if find_images(root):
        dirs.append(root)
    # include solo le sottocartelle di primo livello
    for p in sorted(root.iterdir()):
        if p.is_dir():
            dirs.append(p)
    return dirs

# ---- core --------------------------------------------------------------

def calibrate_folder(
    folder: Path,
    board_size: Tuple[int, int],
    square_size: float,
    json_name: str,
    verbose: bool,
    detect_downscale: float,
    max_images: int | None,
) -> tuple[bool, str]:
    imgs = find_images(folder)
    if not imgs:
        return False, "Nessuna immagine trovata"

    if max_images:
        imgs = imgs[:max_images]

    if verbose:
        print(f"[{folder.name}] {len(imgs)} immagini da processare…", flush=True)

    objp = collect_object_points(board_size, square_size)
    objpoints = []
    imgpoints = []
    used_images = []

    im_w = im_h = None

    for idx, img_path in enumerate(imgs, 1):
        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if img is None:
            if verbose:
                print("?", end="", flush=True)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if im_w is None:
            im_h, im_w = gray.shape[:2]

        # opzionale: rileva su ridotto e riscalo i corner
        if detect_downscale != 1.0:
            small = cv2.resize(gray, None, fx=1.0/detect_downscale, fy=1.0/detect_downscale, interpolation=cv2.INTER_AREA)
            ok, corners = try_detect_corners(small, board_size)
            if ok:
                corners = corners * detect_downscale  # riscalo alle coordinate originali
        else:
            ok, corners = try_detect_corners(gray, board_size)

        if not ok:
            if verbose:
                print("x", end="", flush=True)
            continue

        # refine subpixel
        cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        )

        objpoints.append(objp.copy())
        imgpoints.append(corners)
        used_images.append(img_path.name)
        if verbose:
            print(".", end="", flush=True)

    if verbose:
        print("")  # newline

    if len(objpoints) < 5:
        return False, f"Troppi pochi scatti validi: {len(objpoints)} (min 5)"

    # calibrazione
    camera_matrix = np.eye(3, dtype=np.float64)
    dist_coeffs = np.zeros((8, 1), dtype=np.float64)

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (im_w, im_h), camera_matrix, dist_coeffs,
        flags=cv2.CALIB_RATIONAL_MODEL,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
    )

    # errori
    per_view_errors = []
    total_err = 0.0
    total_points = 0
    for i in range(len(objpoints)):
        projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        e = cv2.norm(imgpoints[i], projected, cv2.NORM_L2)
        n = len(projected)
        per_view_errors.append(float(np.sqrt((e * e) / n)))
        total_err += e * e
        total_points += n
    mean_err = float(np.sqrt(total_err / total_points))

    data = {
        "folder": str(folder),
        "image_size": {"width": int(im_w), "height": int(im_h)},
        "board": {"cols": int(board_size[0]), "rows": int(board_size[1]), "square_size": float(square_size), "units": "user_units"},
        "rms": float(rms),
        "mean_reprojection_error": mean_err,
        "per_view_errors": per_view_errors,
        "used_images": used_images,
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "rvecs": [rv.reshape(-1).tolist() for rv in rvecs],
        "tvecs": [tv.reshape(-1).tolist() for tv in tvecs],
        "flags": ["CALIB_RATIONAL_MODEL"],
        "opencv_version": cv2.__version__,
    }

    out_path = folder / json_name
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    return True, f"OK → {out_path.name}  (RMS={rms:.4f}, mean reproj={mean_err:.4f}, views={len(used_images)})"

# ---- main --------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Batch chessboard calibration (con log progressivi)")
    parser.add_argument("--root", type=Path, default=Path("Calibration"), help="Cartella padre (default: ./Calibration)")
    parser.add_argument("--board-cols", type=int, default=9, help="Inner corners (colonne)")
    parser.add_argument("--board-rows", type=int, default=6, help="Inner corners (righe)")
    parser.add_argument("--square-size", type=float, default=1.0, help="Lato quadrato (unità a scelta)")
    parser.add_argument("--json-name", type=str, default="calibration.json", help="Nome file JSON output")
    parser.add_argument("--force", action="store_true", help="Ricalibra anche se il JSON esiste")
    parser.add_argument("--verbose", action="store_true", help="Log dettagliato (consigliato)")
    parser.add_argument("--detect-downscale", type=float, default=1.0, help="Fattore di downscale SOLO per il detection (es. 2, 3, 4). 1.0 = nessuno")
    parser.add_argument("--max-images", type=int, default=0, help="Usa al massimo N immagini per cartella (0 = tutte)")
    parser.add_argument("--single-thread", action="store_true", help="Forza OpenCV a un solo thread (debug)")

    args = parser.parse_args()

    # gestione Ctrl-C pulita
    def _sigint(_sig, _frm):
        eprint("\nInterrotto dall'utente (Ctrl-C).")
        sys.exit(130)
    signal.signal(signal.SIGINT, _sigint)

    if args.single_thread:
        try:
            cv2.setNumThreads(1)
        except Exception:
            pass

    if not args.root.exists():
        eprint(f'La cartella "{args.root}" non esiste.')
        sys.exit(1)

    board_size = (args.board_cols, args.board_rows)
    max_images = args.max_images if args.max_images > 0 else None

    targets = iter_target_dirs(args.root)
    if not targets:
        eprint("Nessuna cartella/immagine da processare.")
        sys.exit(1)

    processed = skipped = failures = 0

    for d in targets:
        out_file = d / args.json_name
        if out_file.exists() and not args.force:
            skipped += 1
            if args.verbose:
                print(f"[{d.name}] presente {args.json_name} → skip", flush=True)
            continue

        if args.verbose:
            print(f"[{d.name}] calibrazione in corso…", flush=True)

        ok, msg = calibrate_folder(
            d, board_size, args.square_size, args.json_name, args.verbose,
            detect_downscale=max(1.0, args.detect_downscale),
            max_images=max_images,
        )
        if ok:
            processed += 1
            print(f"[{d.name}] {msg}", flush=True)
        else:
            failures += 1
            print(f"[{d.name}] FAIL: {msg}", flush=True)

    print(f"\nSummary: processed={processed}, skipped={skipped}, failures={failures}, total={len(targets)}", flush=True)

if __name__ == "__main__":
    main()
