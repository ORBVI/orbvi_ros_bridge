# Pose metadata and depth/TF diagnosis — 2026-10-21

## Versions and scope

Bridge change based on `origin/main` `45e45afc906472e26311e783593da14cc3ba1ff5`.
SDK Server companion change based on SDK `origin/dev`
`db44d767b704bfff3ff1cde67814725e265f4b88`.
Both tested 4VL devices reported that SDK commit as their running product build.
The initial remote-host tests and subsequent local RViz investigation used
different physical boards; their measurements must not be combined.
Tests used isolated worktrees and an isolated ROS2 domain. The bridge ran
natively on the user's Humble host; Docker was used only for compilation and
unit tests. The initial remote-host tests did not replace the host installation.
During the subsequent local investigation, the user's running preview bridge
was restarted with the repaired node binary; its previous binary was backed up
and its launch configuration and running RViz were retained. Board native
services were not replaced. SDK Server epoch forwarding was exercised as an isolated ARM64 test
on the board, not installed into the running service.

## Confirmed failure and change

Of 100 real SDK pose samples, 85 were `TRACKING/OPTIMIZED`, five were
`DEGRADED/PROPAGATED` with reason `frame_cost_increased`, and ten were
`DEGRADED/OPTIMIZED` with reason `recovering`.

The old bridge recognized only Omni-LIVO `OPTIMIZED`, incorrectly reporting
valid predictions as unsupported source metadata. Both ROS versions now route
predictions to `vio/imu_prediction`. Optimized TF still requires valid tracking;
rejections now name the tracking reason or the actual time/sequence/frame/
calibration failure. This change does not repair or hide estimator degradation.
The SDK Server companion forwards producer `boot_id` and `generation` instead
of losing restart identity during pose conversion; pose payloads are unchanged.

## Verification

| Check | Result |
| --- | --- |
| ROS2 Humble, colcon build/test | 69 reported tests, no failures/errors/skips |
| ROS1 Noetic, catkin build/run_tests | 128 reported tests, no failures/errors/skips |
| ARM64 SDK native source suite, cross-build plus QEMU | 46 tests passed |
| Five targeted native metadata conversion/ingress tests on actual ARM64 board | Passed; localization and SDK service PIDs unchanged |
| Native host bridge, 35-second live 4VL run | 328 optimized Odometry messages, 292 dynamic TF messages; all 292 matched source time and pose within 1e-9 |
| Static TF | Seven edges, all seven available to a late subscriber; four camera/LiDAR/IMU/device lookups passed; one dynamic TF publisher |
| Second 20-second live run | 186 optimized Odometry, 166 matched dynamic TF, 11 prediction Odometry, 196 depth clouds; all frame/tree checks passed |
| Warning behavior in repaired bridge | No unsupported-source warning or generic TF rejection; specific `DEGRADED/recovering` warnings remain |

Build/test entrypoints:

```bash
# ROS2, in the Humble build image, /source mounted read-only:
colcon build --base-paths /source/orbvi_ros_bridge /source/livox_ros_driver2 \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --base-paths /source/orbvi_ros_bridge /source/livox_ros_driver2 \
  --packages-select orbvi_ros_bridge
colcon test-result --verbose

# ROS1, in the Noetic build image, with src/bridge_repo -> /source:
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON -j2
catkin_make run_tests -j2
catkin_test_results --verbose
```

The SDK native source target was built with the repository's RK3588 toolchain
and pinned Protobuf 33.6 dependencies. The board checks selected
`ConvertsOptimizedVinsPoseWithoutRosOdometry`,
`ConvertsOptimizedLivoPoseToExistingSdkWireIdentity`,
`PreservesLivoDegradedPredictionsAndUnknownLegacyState`,
`LivoRecoveryMetadataSurvivesRealIngressAndStreamQueue`, and
`LivoRestartEpochSurvivesSdkConversion` from `test_native_sources`.

## Initial remote-host depth comparison and limits

The initial investigation targeted the reported comparison against Omni-LIVO's
world scan/map. These measurements belong to the first board, not the board
subsequently observed in the local RViz session. A captured optimized pose and registered scan had the exact same
sample timestamp. Reversing that pose and the calibrated IMU/LiDAR transform
placed the scan back onto the current raw LiDAR: nearest-point distances were
0.90 mm median and 1.82 mm at the 90th percentile. This is a consistency check
on a static scene, not a physical calibration accuracy claim.

A separate actual ROS depth cloud and four disparity images had the same
sample timestamp. All 19,927 points reprojected onto source pixels; maximum
absolute optical-depth reconstruction residual was 1.26e-6 m. The cloud used
`orbvi_link`, and matched the declared ROS_BODY axes and source calibration.

Local depth-vs-LiDAR distance differences also appear before any dynamic world
transform. No evidence justified changing the static TF, camera order, units
or adding an artificial offset. Remaining disparity/calibration accuracy and
historical-map differences are unresolved by this metadata patch. Recovery
warnings are real estimator-state warnings. No motion test, long-duration
quality acceptance or physical recalibration is claimed.

## Subsequent local RViz investigation

The local preview actually displays raw `/orbvi/lidar/points` and
`/orbvi/depth/points`, with configured fixed frame `orbvi_link`. Its point-cloud
disagreement therefore also exists without applying the dynamic world pose.
The repaired running node's executable SHA256 matched the tested build:
`94159f5dcae2428bdf3922f0c50ef972999c8044f85038b50b82ff5cd3f575e6`.
The unsupported-source and generic rejection warnings disappeared; explicit
`tracking_state=DEGRADED; tracking_reason=recovering` warnings remain.
A six-second capture after restart received 61 raw LiDAR clouds, 62 depth
clouds, 61 Odometry messages and 61 dynamic TF messages. Static TF still had
seven edges, with exactly the same values as before replacement; each TF topic
had one publisher.
A further 30-second local capture received 301 LiDAR clouds, 302 depth clouds,
294 optimized Odometry messages, 282 dynamic TF messages and six prediction
Odometry messages. The repaired run logged zero unsupported-source warnings
and zero former generic TF rejection warnings. This confirms live prediction
routing; the remaining tracking rejections are not an alignment acceptance.

On this board, all four raw images had mean brightness of approximately
12--16 out of 255. Disparity medians were approximately 27 pixels, reconstructing
depth near 0.55 m. The user confirmed the lights were off or the environment
was very dark. Camera exposure and gain were already near their maxima.
This is evidence of unsuitable input for the observed depth comparison, not
evidence for an artificial TF rotation or scale correction. Alignment remains
unverified until images and point clouds are recaptured with adequate lighting.
No Omni-LIVO algorithm or calibration change was made, and the companion SDK
Server epoch patch was not deployed to this board's production service.

## Consolidation into the existing ROS2 workspace

At the user's request, the bridge fix and the preview's ROS2 TF launch argument
ordering fix were moved into the original ROS2 workspace. Its existing local
overlay launch/RViz files were preserved, along with the current preview layout.
The bridge was rebuilt there with Release and BUILD_TESTING enabled; all 70
reported tests passed, including default/override resolution for the TF switches.
Both bridge and RViz were restarted from that workspace's install prefix.
Temporary bridge worktrees and build directories were then removed. Selected
diagnostic logs and message-count reports were retained in the original
workspace's existing Log directory; temporary raw captures were removed.
SDK Server and Omni-LIVO services were not changed by this consolidation.
Device addresses, credentials and calibration contents are excluded from this
repository.

## Repeated rejection logging and illuminated local-board check

The common ROS1/ROS2 logging policy now reports the first TF rejection
immediately, then limits rejection/recovery reports to one per 60 seconds using
steady host time. Reports include accepted/rejected optimized-pose counts.
Recovery is INFO after at least five seconds of uninterrupted acceptance and
an eligible report interval. Brief state oscillations cannot reset the log
interval. This only changes diagnostics; degraded poses remain excluded from
optimized TF and Odometry publication remains independent.

Three deterministic tests cover repeated/oscillating failures, stable recovery,
the report boundary, reason changes and a healthy stream. The original ROS2
workspace build/test reported 73 tests with no failures. A disposable Noetic
build/test reported 134 tests with no failures; no runtime container was used.
The installed native ROS2 node was restarted in the original workspace.
Successive live rejection warnings were 61.20 seconds apart, rather than five.
A 65-second capture received 646 raw LiDAR clouds, 643 depth clouds, 627
optimized Odometry messages, 595 dynamic TF messages and 17 predictions.
The 594 common TF/Odometry timestamps had maximum absolute pose difference
1.29e-14. Each TF topic had one publisher, with seven static edges available
to the late subscriber.

After the user restored local lighting, raw-image mean brightness increased
to approximately 79--94/255 and depth recovered room-scale structure.
Comparison with calibrated raw LiDAR still showed surface discrepancies; this
does not certify geometric alignment. No artificial TF offset/scale, SDK Server
update or Omni-LIVO change was applied. Images, bounded point-cloud samples,
comparison plots and live reports remain in the workspace's existing Log
directory, outside version control.
