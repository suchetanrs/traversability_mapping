# PRD: Re-port the moment-fused traversability node into the legacy System / LocalMap / KeyFrame architecture

> **Audience assumption:** the implementer has all three trees in hand — the
> legacy core (`archive/` in the core package and `…/LEGACY/…`), the legacy ROS
> layer (`archive/` in the ROS package), and the current monolithic node
> (`local_traversability.cpp`) plus the current ROS-free core (`Moments`,
> `TraversabilityMetrics`, `KeyFrame`, `Parameters`, `Helpers`). This PRD does
> **not** re-explain mechanics that already exist in those trees; it names the
> existing pieces and specifies **where each one goes** and **what changes**.

## Problem Statement

The current system has all the desired runtime features — fixed-frame growing
global grid, cell-local moment fusion, PCA normals over a cell's vicinity, exact
PGO re-binning, and sparse per-cell nav updates — but they live inside a single
monolithic `rclcpp::Node` (`local_traversability.cpp`). Consequently:

- The mapping logic **cannot be built or run without ROS**, so it can't be reused
  or driven by a non-ROS SLAM front-end.
- There is **no `System` entry point** for a SLAM system to drive, and **no
  `LocalMap` unit** that owns its own grid and lifecycle.
- The **keyframe is not an independent, shareable unit** — it can't be routed or
  re-parented, which blocks any future multi-submap (`map_id`) work.
- Additions and PGO updates are processed **synchronously on the ROS callback
  thread**, so a heavy burst can stall reception and the subscriber queue can drop
  pose updates.

The developer wants the proven legacy architecture (`System` → `LocalMap` →
independent `KeyFrame`) back **without losing any current feature**.

## Solution

Redistribute the current node's logic into the legacy three-layer architecture:

- A **ROS-free core library** (`System`, `LocalMap`, `KeyFrame`, plus the existing
  `Moments`, `TraversabilityMetrics`, `Parameters`, `Helpers`, and both
  `PointCloudBuffer` variants) that compiles standalone against PCL / Eigen /
  grid_map_core / Sophus / yaml-cpp.
- A **thin ROS adapter node** that owns TF, message↔plain-data conversion,
  publishers, and the publish timer, driving the core through the legacy `System`
  API.
- `System` = orchestrator (multi-map-ready, runs `map_id = 0` only).
- `LocalMap` = per-submap workhorse (owns grid, threads, arithmetic, outputs).
- `KeyFrame` = independent, shareable, re-parentable record.

## User Stories

1. As a robotics developer, I want a ROS-free core mapping library, so that I can build and link the map without a ROS toolchain.
2. As a robotics developer, I want a `System` orchestrator class, so that I have a single stable entry point to drive the whole pipeline.
3. As a robotics developer, I want a `LocalMap` class that owns its own grid and threads, so that per-map logic is cohesive and self-contained.
4. As a robotics developer, I want an independent `KeyFrame` record, so that a keyframe can be stored, routed, and re-parented without being tied to a map's grid.
5. As a SLAM integrator, I want to add a keyframe by `double` or nanosecond timestamp and have the system fetch the matching cloud from a buffer, so that I can integrate a SLAM that announces keyframes separately from the cloud stream.
6. As a SLAM integrator, I want to add a keyframe with a PCL cloud supplied directly, so that I can bypass the buffer when I already hold the cloud.
7. As a SLAM integrator, I want both a pure-PCL buffer and a ROS (`sensor_msgs`) buffer, selectable by param, so that I can buffer clouds with or without ROS.
8. As a SLAM back-end, I want to update a keyframe pose with `Sophus::SE3f` and with `Eigen::Affine3d`, so that PGO corrections flow in via the legacy API.
9. As a SLAM back-end, I want pose updates accepted in O(1) on the calling thread, so that bursty loop-closure corrections are never dropped by a busy worker or a full subscriber queue.
10. As a SLAM back-end, I want only the latest pose per keyframe retained between ticks, so that superseded corrections don't cause redundant re-binning and memory stays bounded.
11. As a navigation consumer, I want a sparse per-cell update of only the changed cells, so that I can maintain my costmap cheaply in deployment.
12. As a navigation consumer, I want a full snapshot on first connect / reconnect, so that I start consistent without having missed earlier deltas.
13. As a navigation consumer, I want cleared cells communicated explicitly, so that my costmap drops stale obstacles when a keyframe is moved or deleted.
14. As a developer, I want the full `grid_map` and a flattened occupancy grid published only when subscribed, so that I can visualize in RViz without paying for it in deployment.
15. As an operator, I want hazards fused from a cell's vicinity via PCA over all points, so that slope/step/roughness reflect the true local surface.
16. As an operator, I want the global grid to grow as the robot explores, anchored to a fixed absolute lattice, so that a keyframe's contribution stays valid across resizes.
17. As a SLAM back-end, I want a PGO update to exactly subtract a keyframe's old contribution and re-add the new one with cell membership recomputed (partition not frozen), so that the fused map has no residual drift after correction.
18. As a SLAM back-end, I want to delete a keyframe, so that keyframes culled by my back-end are removed exactly from the map.
19. As a SLAM back-end, I want a loop-closure notification that clears and rebuilds the map from retained clouds, so that a large correction yields a consistent map.
20. As a developer, I want the most-recent-N keyframes serviced at a higher rate in their own worker, so that the area around the robot stays fresh.
21. As a developer, I want all keyframes with pending changes serviced by a slower worker that early-returns on idle keyframes, so that older / scrolled-out keyframes still get corrected without wasted work.
22. As a developer, I want each `LocalMap` to own and spawn its own worker threads, so that map lifecycle and thread lifecycle are coupled.
23. As a developer, I want keyframe clouds retained only when keyframe optimization is enabled, so that I can trade memory for PGO-correctability per deployment.
24. As a developer, I want a voxel-downsampled stitched global point cloud on request, so that I can export or inspect accumulated geometry.
25. As a platform owner, I want a multi-map-ready structure keyed by `map_id` (honoring the existing message field), so that multi-submap SLAM can be enabled later without refactor.
26. As a platform owner, I want keyframes routed to the correct map and re-parentable between maps, so that map merges / submap reassignment are possible later.
27. As a developer, I want robot extrinsics resolved once from TF and reused, so that pruning and base-frame transforms are consistent across all keyframes.
28. As a developer, I want cloud pruning (ego / ceiling / range gates + transform to base frame) to run in the core, so that the same filtering applies regardless of how clouds enter.
29. As a developer, I want the ROS adapter to be a thin translation layer, so that all mapping behavior is decided in the core and the node only wires topics and converts.
30. As a maintainer, I want obsolete v1 components removed from the active build, so that superseded approaches don't clutter the codebase.

## Implementation Decisions

### A. Layering / module placement

- **Core (no ROS):** promote `System`, `LocalMap`, `KeyFrame` into the core
  `traversability_mapping` library alongside the existing `Moments`,
  `TraversabilityMetrics`, `Parameters`, `Helpers`, and both buffers. The library
  **must build with no ROS package present.**
- **ROS coupling behind `#ifdef WITH_ROS2_SENSOR_MSGS`**, exactly as legacy did:
  `PointCloudBufferROS` and the `sensor_msgs`-typed overloads are guarded; pure
  `PointCloudBuffer` is always compiled.
- **Move `gridMapToOccupancyGrid` (and any `nav_msgs`/`sensor_msgs` includes) out
  of `Helpers` into the adapter**, so the core has zero ROS headers even behind
  ifdefs.
- **`traversability_mapping_common`** stays as a ROS-free Sophus↔Eigen utility
  (needed by the `Sophus` `updateKeyFrame` overload).
- **Thin adapter node** (in `traversability_mapping_ros`) owns: TF extrinsic
  lookup, `Pose↔Eigen::Affine3f`, `sensor_msgs/PointCloud2 → PCL`,
  `grid_map ↔ grid_map_msgs`, occupancy conversion, sparse-update message packing,
  publishers, and the publish timer. It drives the core via the `System` API.

### B. Ownership (this revises the early "Map owns everything / System thin" idea)

- **`System` (orchestrator, not thin):** owns the map registry (`map_id →
  shared_ptr<LocalMap>`), the **authoritative shared keyframe registry** (`kf_id →
  shared_ptr<KeyFrame>` — promoted up from legacy `LocalMap::keyFramesMap_`), the
  `kf_id → map_id` routing (legacy `allKeyFramesSet_`), both point-cloud buffers,
  the robot extrinsics, and the full legacy public API.
- **`LocalMap` (per submap):** owns its `grid_map::GridMap` (moment + derived
  hazard/nav layers), the master grid mutex, its **two worker threads**, the
  moment arithmetic + hazard recompute, the local-window deque, the dirty-keyframe
  set, the dirty-for-nav cell set, and the working set of keyframes routed to it
  (shared_ptrs from `System`'s registry).
- **`KeyFrame` (shared, re-parentable):** the current lean record
  (`id`, `pose_` = map←base, `cloud_base_`, `partials_`, `rebin()`, `setPose()`)
  **plus** `timestamp`, `parentMapID_`, a **latest-wins pending-pose slot with its
  own small lock**, a dirty flag, and a `setMap` re-parent helper. It still never
  touches a grid. The legacy fat `KeyFrame` members (grid pointer,
  `computeLocalTraversability`, `recomputeCache`) are **not** carried over — grid
  arithmetic lives in `LocalMap`.

### C. Runtime topology

- **Axis A (local vs global): exactly one `LocalMap` runs.** No separate
  rolling-window map object; "global" is an on-demand cross-map stitch
  (`getGlobalPointCloud` now, `multi_map_merge` later), not a live object.
- **Axis B (multi-map by `map_id`): build the full multi-map structure/API, but
  instantiate only `map_id = 0`.** The existing `KeyFrame.msg::map_id` is now
  **honored by routing** (previously ignored).
- **All `LocalMap`s share one absolute global lattice** (common `grid_center_x`,
  `grid_center_y`, `resolution`), so a keyframe's absolute-cell `partials_`
  transfer directly on re-parent.

### D. Threading & reception

- **Two worker threads per `LocalMap`, keeping the legacy names**, spawned in the
  `LocalMap` constructor and stopped/joined in its destructor:
  - **`RunLocalKeyFrames`** — services dirty keyframes that are **in the last-N
    window** at a higher rate.
  - **`RunTraversability`** — services **all** dirty keyframes at a slower pace,
    **early-returning on keyframes with nothing pending** (so it is the cheap
    sweep, not redundant re-binning).
  - `RunUpdateQueue` from legacy is **removed** — its routing job is now O(1) in
    the reception path (see below).
- **Per-keyframe op (both workers):** subtract the keyframe's old partials from the
  grid → `rebin()` at the current pose (**partition not frozen** — membership
  recomputed every time) → add the new partials → recompute hazards over the
  **dilated set of touched cells** (for updates, the dilated union of cells left
  and newly occupied). This is the current node's `processKeyframe` body, moved
  into `LocalMap` and split off from reception.
- **Reception is O(1) and never blocks** (replaces the legacy FIFO
  `keyFrameUpdateQueue_`, while keeping the same public API signatures):
  - `updateKeyFrame(id, pose)` → `System` routes by `kf_id → map_id`, writes the
    **latest** pose into that keyframe's pending-pose slot (under its slot lock),
    and inserts the id into that map's dirty set. Latest-wins collapses bursts and
    bounds memory.
  - `addNewKeyFrame*` → create the keyframe (prune → store cloud → set pose),
    register it, push it to the window, and mark it new+dirty.
- **Concurrency:** all grid writes serialize on the master grid mutex; the
  per-keyframe dirty flag (cleared under the keyframe's slot lock) makes a
  double-touch by both workers idempotent.
- **No OpenMP.** Remove the existing OMP pragmas from `rebin()` and the hazard
  recompute; parallelism comes only from the thread structure. OMP is deferred.

### E. Public API (legacy-compatible) and lifecycle

- **Additions:** by `double` timestamp, by nanosecond timestamp (both via buffer
  `getClosestPointCloud`), and with a directly supplied PCL cloud
  (`addNewKeyFrameWithPCL`). `pushToBuffer` overloads retained.
- **Pose updates:** `Sophus::SE3f` and `Eigen::Affine3d` overloads (the adapter
  converts `geometry_msgs/Pose → Affine3f` for the message path).
- **`deleteKeyFrame`:** subtract its partials, recompute the vacated (dilated)
  cells, erase it from the keyframe registry / window / dirty set, drop its cloud.
- **`informLoopClosure` / `clearEntireMap`:** NaN all grid layers and clear all
  partials, then mark every keyframe dirty so the workers rebuild the grid from the
  retained clouds. (If `is_kf_optimization_enabled` is false, clouds were dropped
  after first bin and those keyframes cannot be rebuilt — documented limitation.)
- **`addNewLocalMap`, `getLocalMap`, `updateKFMap`:** present. `updateKFMap` /
  `setMap` re-parent in the moment model = subtract the keyframe's partials from the
  old map's grid, move its reference into the new map's working set, mark it dirty
  there → the new map's worker re-bins and adds it.
- **`setExtrinsicParameters`** and **`getGlobalPointCloud`** (legacy
  `getStitchedPointCloud`, **voxel-downsampled** via the
  `GetGlobalPointcloud.srv` `voxel_size_x/y/z`) retained.

### F. Output contract (ROS boundary)

- **Sparse path (the only deployment path):** `LocalMap` exposes plain data —
  `drainNavDelta()` → `{cell_keys, layer_names (the nav subset), values
  (row-major), is_full_snapshot}` — drained under the master mutex (brief lock;
  message serialization happens **outside** the lock). The adapter packs it into
  `TraversabilitySparseUpdate`. This is the current node's `fillMessage` /
  `dirty_for_nav_` logic, split: dirty accumulation + value extraction in
  `LocalMap`, message packing in the adapter.
- **Full-snapshot-on-(re)connect lives in the adapter** (it is subscriber-aware):
  it watches the sparse subscriber count and calls `fullNavSnapshot()` (the
  current node's `allOccupiedKeys` logic, as plain data) on connect, then resumes
  draining deltas.
- **Debug path:** the full `grid_map` and flattened occupancy grid are produced
  **only when a subscriber is present**, by reading the live grid under the master
  mutex; blocking the workers during that read is acceptable because it is
  debug-only.

### G. Cloud handling

- **Pruning runs in the core** (`System`/`LocalMap`, reusing the current node's
  `pruneToBase`): ego/invalid-return rejection, transform into the robot base
  frame via the extrinsics, and the ceiling/overhang (`robot_height`) + range
  (`min/max_range_base_frame`) gates. The adapter performs only
  `sensor_msgs → PCL` and TF.
- **Extrinsics:** the adapter resolves `Tsv` (slam←lidar) and `Tbs` (base←slam)
  from TF (current `populateTransforms`), composes `Tbv = Tbs · Tsv`, and passes
  them via `setExtrinsicParameters`; frames come from the existing `slam_frame`,
  `robot_base_frame`, `lidar_frame`, `map_frame` params.
- **Retention gated by `is_kf_optimization_enabled`:** enabled ⇒ every keyframe
  keeps its pruned base-frame cloud and stays PGO-correctable; disabled ⇒ cloud
  dropped after the first bin and that keyframe can no longer be re-binned.

### H. Removed from the active build

- **`HashGrid`** (legacy per-keyframe vicinity index) and **`TraversabilityGrid`**
  (v1 per-keyframe grid) — the moment model reads a cell's vicinity directly from
  the absolute lattice; both stay in `archive/` only.

### I. Redistribution of existing logic (you have both codebases)

- **Adapter ← current node:** ROS params, `additionsCallback` / `updatesCallback`,
  publishers (gridmap / occupancy / sparse) with their QoS, `publish*` timer,
  `poseToAffine`, `populateTransforms`, `sensor_msgs → PCL`, and the
  message-packing halves of `fillMessage` / `allOccupiedKeys`.
- **`LocalMap` ← current node:** `freshMap`, `growToInclude`, `growToIncludeCells`,
  `cellPos`, `addToLayer`, `addPartialToGrid` (±1 with `blankCell` on `N≤0` +
  mark-dirty), `readCellMoment`, `dilate`, `recomputeCell`, `recomputeDirty`
  (minus OMP), the `processKeyframe` body, and the `lattice_`, `gridMap_`,
  `layers_` / `nav_layers_`, `dirty_for_nav_`, `delta_ind_` state. **← legacy
  `LocalMap`:** the two worker threads, `masterGridMapMutex_`, the local-window
  deque, `clearEntireMap`, and `getStitchedPointCloud`.
- **`System` ← legacy `System`:** `localMapsSet_`, `allKeyFramesSet_` (→ routing),
  the buffers, `setExtrinsicParameters`, the `addNewKeyFrame*` variants, the
  `updateKeyFrame` overloads, `updateKFMap`, `deleteKeyFrame`, `informLoopClosure`,
  `addNewLocalMap`, `getLocalMap`, `getGlobalPointCloud` — **plus** the
  authoritative shared keyframe registry promoted up from `LocalMap`.
- **`KeyFrame` ← current `KeyFrame`** (lean), extended with the fields in §B.

## Out of Scope

- **All testing** — no unit, seam, or integration tests are specified or required
  by this change.
- **OpenMP / intra-operation parallelism** — deferred; this change strips existing
  OMP usage.
- **Running more than one map at runtime** — structure is multi-map-ready, but only
  `map_id = 0` runs; exercising the N-map path, independent submap frames, and
  `multi_map_merge` runtime behavior are future work.
- **A live rolling-window "local" map object** — "global" stays an on-demand
  stitch.
- **Distance/count-based cloud eviction or keyframe freezing** — retention is
  all-or-nothing per the optimization flag.
- **The current node's per-keyframe `publish()` during a PGO batch** — publishing
  becomes timer-only; the in-loop publish hack is not carried over.
- **Changes to the hazard math** (`computeGoodness`, PCA normals, gate params) and
  **to the message/service schemas** (`KeyFrame`, `KeyFrameAdditions`,
  `KeyFrameUpdates`, `TraversabilitySparseUpdate`, `GetGlobalPointcloud.srv` all
  unchanged).
- **Changes to the `ground_truth_kfs` simulators** and to `traversability_grid_utils`
  beyond what's needed to drive/consume the adapter.

## Further Notes

- The restructure reconciles with the original intent: with multi-map restored,
  `LocalMap` regains its legacy per-submap meaning and `KeyFrame` regains the
  independence that lets it be shared and re-parented across maps.
- **Suggested implementation order:** extend `KeyFrame` → build `LocalMap`
  single-threaded first (port the grid/arithmetic/recompute, verify the
  add/subtract/rebin/recompute path), then add the two worker threads → add
  `System` orchestration + routing → write the thin adapter node.
- **Agreed defaults:** shared global lattice across all maps; local-window size
  `N` from the existing `num_local_keyframes` param (10); `N` maps ⇒ `2N` threads;
  pruning in the core; `traversability_mapping_common` kept as a ROS-free
  Sophus↔Eigen utility.
