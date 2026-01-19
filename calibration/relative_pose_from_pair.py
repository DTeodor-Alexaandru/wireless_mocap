import json
import numpy as np
import cv2

def load_intrinsics(path):
    with open(path, "r") as f:
        cal = json.load(f)
    K = np.array(cal["camera_matrix_K"], dtype=np.float64)
    d = np.array(cal["distortion_coefficients"], dtype=np.float64).reshape(-1, 1)
    w = int(cal["image_width"]); h = int(cal["image_height"])
    return K, d, (w, h)

# --- user inputs ---
LEFT_JSON  = "calibration_left.json"
RIGHT_JSON = "calibration_right.json"

# baseline in meters (463 mm)
B = 0.463

# Option A: use estimated R,T (recommended)
USE_ESTIMATED_RT = True
R_est = None
T_est = None
# If you have a stereo_calibration.json from stereoCalibrate, load it:
# with open("stereo_calibration.json","r") as f:
#     s = json.load(f)
# R_est = np.array(s["R"], dtype=np.float64)
# T_est = np.array(s["T"], dtype=np.float64).reshape(3,1)

# Option B: force rotation=0
# USE_ESTIMATED_RT = False

# --- load intrinsics ---
K1, d1, size1 = load_intrinsics(LEFT_JSON)
K2, d2, size2 = load_intrinsics(RIGHT_JSON)
assert size1 == size2
image_size = size1  # (w,h)

# --- choose extrinsics ---
if USE_ESTIMATED_RT:
    if R_est is None or T_est is None:
        raise RuntimeError("Set R_est and T_est (load from your estimate) or disable USE_ESTIMATED_RT.")
    R = R_est
    T = T_est
    T = T * (B / float(np.linalg.norm(T)))  # scale to measured baseline
else:
    R = np.eye(3, dtype=np.float64)
    T = np.array([[B],[0.0],[0.0]], dtype=np.float64)

# --- rectify ---
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
    K1, d1, K2, d2, image_size, R, T,
    flags=cv2.CALIB_ZERO_DISPARITY,
    alpha=0
)

map1x, map1y = cv2.initUndistortRectifyMap(K1, d1, R1, P1, image_size, cv2.CV_32FC1)
map2x, map2y = cv2.initUndistortRectifyMap(K2, d2, R2, P2, image_size, cv2.CV_32FC1)

np.savez("rectify_maps_and_Q.npz",
         image_size=np.array(image_size),
         map1x=map1x, map1y=map1y,
         map2x=map2x, map2y=map2y,
         R=R, T=T, R1=R1, R2=R2, P1=P1, P2=P2, Q=Q)

print("Saved rectify_maps_and_Q.npz")
print("Baseline used:", float(np.linalg.norm(T)))
print("R=\n", R)
print("T=\n", T.reshape(-1))
print("Q=\n", Q)
