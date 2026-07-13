# gaz_slam

`gaz_slam` is a ROS-free Radapter adapter around the mapping core used by
[`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox): Karto performs
correlative scan matching and loop detection, while Ceres optimizes the pose
graph.

The vendored sources are based on `slam_toolbox` commit
`4a27834f2ec1f9c532a19cf686a27574e8a0c68e` (2026-06-25). Karto is under
`third_party/karto_sdk`; the adapted Ceres solver is under
`third_party/ceres_solver`. Their upstream license files are kept alongside
the sources. The adaptations remove ROS lifecycle/pluginlib dependencies,
replace the solver's ROS parameter interface with a plain C++ config, and use
`std::mutex` instead of Boost.Thread.

Build dependencies are Ceres (with SuiteSparse), Eigen 3, TBB, and Boost
Serialization; on Debian/Ubuntu these are normally provided by
`libceres-dev`, `libeigen3-dev`, `libtbb-dev`, and
`libboost-serialization-dev`.

Solver parameters are Radapter enums and reject unknown strings while parsing
the Lua configuration. The accepted values are:

- `linear_solver`: `SPARSE_NORMAL_CHOLESKY`, `SPARSE_SCHUR`,
  `ITERATIVE_SCHUR`, or `CGNR`;
- `preconditioner`: `JACOBI`, `IDENTITY`, or `SCHUR_JACOBI`;
- `trust_strategy`: `LEVENBERG_MARQUARDT` or `DOGLEG`;
- `dogleg_type`: `TRADITIONAL_DOGLEG` or `SUBSPACE_DOGLEG`;
- `loss_function`: `None`, `HuberLoss`, or `CauchyLoss`.

The `Slam` worker accepts odometry poses plus the LaserScan-shaped `scan`
message emitted by `gaz_nav`'s `Lidar`. It emits:

- `position`: the Karto-corrected robot pose;
- `map`: a dynamically bounded occupancy grid in the nav stack's `GAMP` byte
  format;
- `scan`: accepted scan endpoints in the corrected world frame;
- `covariance` and `slam`: scan-matcher diagnostics.

`nodes/nav.lua` connects this output to `CostmapServer`, both planners, and the
GUI. `Slam:Save("path/base")` writes `path/base.posegraph` and
`path/base.data` using Karto's serialization format.

The map follows Karto's current occupancy-grid bounds and therefore grows or
moves as new space is observed. Its 24-byte header contains `magic`, `width`,
`height`, `resolution`, `origin_x`, and `origin_y`, followed by row-major cell
bytes. Free cells are `0`, occupied cells are `100`, and unobserved cells are
`255` (unknown). `CostmapServer` adopts this geometry and preserves unknown
space, so both planners treat it as impassable until SLAM observes it as free.
Legacy 16-byte `GAMP` maps without an origin remain readable with an implicit
origin of `(0, 0)`.
