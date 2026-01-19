import json
import numpy as np
import cv2

# ------------ CONFIG ------------
LEFT_JSON  = "calibration_left.json"
RIGHT_JSON = "calibration_right.json"   # if you don't have it, set RIGHT_JSON = LEFT_JSON (approximation)

BASELINE_M = 0.426  # meters

# Pixel measurements from each camera (RAW pixels from the original images)
uL, vL = 142.0, 242.0
uR, vR = 414.0, 242.0  # put your right-y here (you said rectified y now matches)

# If your (uL,vL,uR,vR) are from RAW images: keep True
# If your points are already measured on rectified images: set False
POINTS_ARE_RAW = True
# --------------------------------


def load_cal(path: str):
    with open(path, "r", encoding="utf-8") as f:
        cal = json.load(f)
    K = np.array(cal["camera_matrix_K"], dtype=np.float64)
    d = np.array(cal["distortion_coefficients"], dtype=np.float64).reshape(-1, 1)
    w = int(cal["image_width"])
    h = int(cal["image_height"])
    return K, d, (w, h)


K1, d1, size1 = load_cal(LEFT_JSON)
K2, d2, size2 = load_cal(RIGHT_JSON)
if size1 != size2:
    raise RuntimeError(f"Resolution mismatch: left {size1}, right {size2}")

image_size = size1  # (w,h)

# Assumed relative pose: Camera2 is to the right of Camera1 by BASELINE_M
R = np.eye(3, dtype=np.float64)
T = np.array([[BASELINE_M], [0.0], [0.0]], dtype=np.float64)

# Rectification (even with R=I, this handles distortion consistently and gives P1/P2/Q)
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
    K1, d1, K2, d2,
    image_size, R, T,
    flags=cv2.CALIB_ZERO_DISPARITY,
    alpha=0
)

def rectify_point(u, v, K, d, Rrect, Prect):
    pt = np.array([[[u, v]]], dtype=np.float64)
    # Output is in rectified pixel coordinates because we pass P=Prect
    pr = cv2.undistortPoints(pt, K, d, R=Rrect, P=Prect).reshape(2)
    return float(pr[0]), float(pr[1])

if POINTS_ARE_RAW:
    uLr, vLr = rectify_point(uL, vL, K1, d1, R1, P1)
    uRr, vRr = rectify_point(uR, vR, K2, d2, R2, P2)
else:
    uLr, vLr = uL, vL
    uRr, vRr = uR, vR

# Triangulate using rectified projection matrices
ptL = np.array([[uLr], [vLr]], dtype=np.float64)
ptR = np.array([[uRr], [vRr]], dtype=np.float64)

Xh = cv2.triangulatePoints(P1, P2, ptL, ptR)  # 4x1
X = (Xh[:3] / Xh[3]).reshape(3)               # X,Y,Z in left camera frame (meters if baseline is meters)

# Also compute via Q (should match)
dpx = uLr - uRr
Xq = Q @ np.array([uLr, vLr, dpx, 1.0], dtype=np.float64)
Xq = (Xq[:3] / Xq[3]).reshape(3)

print("Rectified points:")
print(f"  Left : ({uLr:.3f}, {vLr:.3f})")
print(f"  Right: ({uRr:.3f}, {vRr:.3f})")
print(f"  disparity d = {dpx:.3f} px")
print()

print("3D from triangulatePoints (left camera frame, meters):")
print(f"  X={X[0]:.6f}, Y={X[1]:.6f}, Z={X[2]:.6f}")
print()

print("3D from Q reprojection (should be very close):")
print(f"  X={Xq[0]:.6f}, Y={Xq[1]:.6f}, Z={Xq[2]:.6f}")
