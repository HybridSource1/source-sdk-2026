# vphysics_box3d

A VPhysics backend for Mapbase built on [Box3D](https://github.com/erincatto/box3d), Erin Catto's 3D physics engine (the 3D sibling of Box2D). It implements the same interfaces as Valve's IVP-based `vphysics.dll`:

| Interface | Version |
|---|---|
| `IPhysics` | `VPhysics031` |
| `IPhysicsCollision` | `VPhysicsCollision007` |
| `IPhysicsSurfaceProps` | `VPhysicsSurfaceProps001` |

Because it implements those interfaces, the engine and game code use it without changes. Existing models (`.phy`) and maps load as they are: IVP compact surfaces are converted to Box3D hulls at load time.

Box3D is vendored in `src/thirdparty/box3d` (MIT licence; the upstream commit is in `VERSION.txt`).

## Building

The project is `vphysics_box3d`. It is in the `physics` and `everything` VPC groups.

**Windows**
1. Enable `VS2019` or `VS2022` in `vpc_scripts/newer_vs_toolsets.vpc`. Box3D is C17, which the VS2013 toolset can't compile, so the project is skipped without one of these.
2. Generate the solution:
   ```
   devtools\bin\vpc.exe /hl2 +physics /mksln physics.sln
   ```
3. Build `vphysics_box3d` (Win32).

**Linux**
1. You need GCC 8 or newer, the first version with `-std=gnu17`.
2. Generate the makefile and build:
   ```
   devtools/bin/vpc /hl2 +physics /mksln physics
   make -f physics.mak
   ```

The build outputs `game/bin/vphysics_box3d.dll` on Windows, and `vphysics_box3d.so` plus `vphysics_box3d_srv.so` on Linux.

Box3D's `.c` files are compiled as C: per-file `/TC /std:c17` on MSVC, and `$(CC)` with `-std=gnu17` on Linux. The latter needed a small change to `devtools/makefile_base_posix.mak.*`, because those bases used to send every source file to `$(CXX)`.

The Box3D sources, and the whole module on Linux, are built with precise floating point (`/fp:precise`; `-fno-fast-math -ffp-contract=off`).

## Installing

The engine loads `vphysics` from the game's root `bin` folder (for example `Source SDK Base 2013 Singleplayer/bin`), not from the mod.

1. Back up the existing `bin/vphysics.dll` (Linux: `bin/vphysics.so` and `bin/vphysics_srv.so`).
2. Copy `vphysics_box3d.dll` there as `vphysics.dll`. On Linux, copy `vphysics_box3d.so` → `vphysics.so` and `vphysics_box3d_srv.so` → `vphysics_srv.so`.
3. Start the game. The console prints `[VPhysics Box3D] Using Box3D x.y.z`.

To go back to IVP, restore the backup.

**Multiplayer:** the VPhysics interfaces are identical in `sp/` and `mp/`, so the binary built from `sp/` works for multiplayer branches too. It isn't duplicated under `mp/src`, because that tree's toolchains (VS2013, Steam Runtime GCC 4.x) can't build C17.

**Tools:** keep the stock `vphysics.dll` for vbsp, studiomdl, Hammer and the other tools. `CollideWrite` and `UnserializeCollide` use a private Box3D format, so collision written by a Box3D-backed tool can only be read back by this backend. Reading is the other way round: this backend reads everything IVP writes.

## Supported

| Area | What's covered |
|---|---|
| Collision geometry | Convex hulls from `.phy`, BSP and brush-entity collision, `BBoxToCollide`, `ConvexFromVerts`/`ConvexFromPlanes`, polysoup and virtual meshes (displacements, static props) |
| Collision queries | Ray and box traces (`TraceBox`, `TraceCollide`), point and box contents, `CollideGetAABB`, debug meshes, `ICollisionQuery` |
| Rigid bodies | Mass, inertia, centre-of-mass overrides, damping, drag, gravity toggles, materials (`surfaceproperties` with `base` inheritance), triggers, motion and collision enable flags, sleep and wake, `BecomeHinged` |
| Game callbacks | Collision start/end (`IPhysicsCollisionEvent`), friction and scrape events, trigger enter and leave, sleep events, `ShouldCollide` filtering, collision sets, object pair no-collide |
| Constraints | Hinge, ragdoll (one free axis becomes a revolute joint; more free axes become a spherical joint with cone/twist limits and joint friction), ballsocket, fixed, sliding, length (distance or rope), pulley, constraint groups, breakable limits, springs |
| Controllers | Shadow (kinematic and dynamic), player, motion (`IMotionEvent`), fluid (buoyancy and damping), wheeled vehicles (Jeep-style) |
| Save/restore | Objects (including inline shadow controllers), constraints, constraint groups, springs and motion controllers. Game code recreates vehicles and the other controllers, as it does with IVP. |

## Not supported / differences

- **Raycast vehicles** (airboat, jetski): these get the regular wheeled vehicle, so handling is wrong for airboat-style vehicles.
- **`IPhysicsEnvironment::TransferObject`**: returns false.
- **Byte-swapped (Xbox 360/PS3) collision data**: ignored.
- **`IsBoxIntersectingCone`**: uses a bounding-sphere approximation.
- **Behaviour differences**: tuning is not identical to IVP. Ragdolls, vehicle handling and fluid damping are configured to feel close, but they don't match exactly. Adjust them with the convars below.

When a feature is not supported, the backend prints a one-time `DevWarning`. Run `developer 1` to see these warnings.

## Convars

| Convar | Default | Meaning |
|---|---|---|
| `box3d_threads` | 1 | Worker threads per physics environment; applied on map load. |
| `box3d_substeps` | 4 | Solver sub-steps per physics tick. |
| `box3d_max_ticks_per_frame` | 8 | Ticks simulated per `Simulate()` call before time is dropped. |
| `box3d_hit_event_speed` | 20 | Approach speed (in/s) that produces collision callbacks. |
| `box3d_max_collision_events` | 256 | Collision callbacks dispatched per tick. |
| `box3d_friction_events` | 1 | Send friction/scrape callbacks. |
| `box3d_min_inertia_ratio` | 0.05 | Minimum ratio of smallest to largest principal inertia. |
| `box3d_ragdoll_friction_scale` | 1.0 | Scale on ragdoll joint friction. |
| `box3d_ragdoll_min_friction` | 0.05 | Minimum ragdoll joint friction (N·m). |
| `box3d_break_force_scale` | 1.0 | Scale on constraint break-force limits. |
| `box3d_break_torque_scale` | 1.0 | Scale on constraint break-torque limits. |
| `box3d_player_teleport_distance` | 24 | Distance (in) at which the player shadow snaps instead of moving. |
| `box3d_fluid_damping_scale` | 20 | Scale on fluid damping. |
| `box3d_vehicle_torque_scale` | 4 | Wheel torque (N·m) per engine horsepower. |
| `box3d_vehicle_brake_scale` | 1.5 | Brake torque (N·m) per kg of vehicle mass. |

## Layout

| File | Contents |
|---|---|
| `b3vp_interface.cpp` | `IPhysics`, collision sets, Box3D allocator and assert hooks |
| `b3vp_collide.*` | `IPhysicsCollision`: `.phy`/IVP conversion, traces, serialization |
| `b3vp_environment.*` | `IPhysicsEnvironment`: stepping, event dispatch, save/restore |
| `b3vp_object.*` | `IPhysicsObject` |
| `b3vp_constraints.*` | Constraints, constraint groups, springs |
| `b3vp_controllers.*` | Shadow, player, motion, fluid and vehicle controllers |
| `b3vp_parse.*` | `IVPhysicsKeyParser` |
| `b3vp_surfaceprops.*` | `IPhysicsSurfaceProps` |
| `b3vp_base.h` | Unit conversions: Source works in inches and degrees, Box3D in metres and radians |

## Updating Box3D

1. Replace `src/thirdparty/box3d/include` and `src/thirdparty/box3d/src` with the new upstream sources.
2. Update `VERSION.txt`.
3. Re-apply the local patch to `src/ctz.h`, which provides a 32-bit MSVC `b3PopCount64`. It is marked with `Mapbase:`.
4. If files were added or removed, regenerate the file lists in `vphysics_box3d.vpc`.
