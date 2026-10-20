# Shared camera and adjacent-stereo contract

Pure C++17 headers shared by native runtime and Host SDK; no ROS, transport or device ownership.

- Physical calibration/raw-role order: front, right, rear, left.
- X5 4V Lite IDs: 1,0,3,2. RK3588 IDs: 3,2,4,5. 4P retains connector IDs 0,1,2,3 without direction roles.
- Adjacent pair/atlas IDs: 0=front, 1=right, 2=rear, 3=left. L/R order is preserved.
- `ForPlatform(platform)->stereo(Role::Front)` selects pair0 and the platform's two source IDs.
- `product.hpp`: `ProductFromModel`, `DefaultRig` and `DefaultCameraRole` are consumed by MCAP recording and role binding validation. 4P and unknown products have no default rig or role. Use these product-aware queries for identity; `ForPlatform(X5)` is the legacy Lite rig helper, not a 4P mounting declaration.
- `Rectification` defines one common baseline, rectified axes and virtual K for map generation and SDK export.
- Identity and geometry validation belongs to initialization. Images remain single-atlas/ROI views.

See docs/plans/2026-09-08-adjacent-stereo-direction-contract.md for migration and evidence boundaries.
