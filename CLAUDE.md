# Arm Guy Game

Physics rage/climbing game (Getting Over It / Only Up). A torso with two arms; the arms are the only way to move. **Godot 4.5, full 3D**, locked to the X-Y plane for 2.5D play. (Earlier design notes said 2D / `RigidBody2D` / `PinJoint2D` — that is obsolete; everything below is 3D.)

## Controls

| Input | Action |
|-------|--------|
| Hold **A** | aim/pose the **left** arm at the mouse |
| Hold **D** | aim/pose the **right** arm at the mouse |
| Hold **Left click** | grab with the **left** hand |
| Hold **Right click** | grab with the **right** hand |
| **R** | restart scene |

Released arms freeze in place. **Arm collision is always on:** a posed (non-gripping) arm whose hand presses against a surface shoves the torso when you *rotate* the arm — pressing into the surface peels you off its normal, sweeping along it scoots you the opposite way. That's how you crawl/scoot without grabbing (a still hand does nothing — no jetpack). **Grabbing is a pivot:** the hand pins to the contact point and the body orbits it; while the pose key is held the arm pulls taut (straightens) so the swing rides on a rigid spoke. The cursor steers that swing only while the grabbed arm's pose key (A/D) is also held — with the key up the grip goes passive and the body swings/falls as a free gravity pendulum on the arm (keeping its momentum) rather than chasing the mouse. Releasing the grab flings the body tangentially at the speed it was orbiting, in the direction of the spin (wind up a swing, let go, fly off the arc); release while holding still and you just drop.

Input polling is raw key/mouse-button (`KEY_A`/`KEY_D`, `MOUSE_BUTTON_LEFT`/`RIGHT`) except `restart`, which uses the `restart` input action. The `grab_left`/`grab_right` input actions exist in project settings but are currently unused.

## Architecture

```
Main (Node3D)                       scenes/main.tscn
 ├── Ground (StaticBody3D, 40×1×40)
 ├── Player (RigidBody3D)            scenes/player/player.tscn  — torso
 │    ├── Box (box_B.fbx mesh)
 │    ├── CollisionShape3D (BoxShape3D)
 │    └── ArmFront (arms.glb)        ← arms.gd; contains the Skeleton3D
 ├── Camera3D                        ← camera_follow.gd
 └── DirectionalLight3D
```

- **Player** is a `RigidBody3D`: `mass 4`, `linear_damp 0.6`, `angular_damp 6`, `can_sleep off`, and axis-locked (`linear_z`, `angular_x`, `angular_y`) so it stays in and only rotates within the X-Y plane. No direct locomotion — moves only via the arms. Uses default 3D gravity.
- **arms.glb** is one rigged `Skeleton3D` holding both arms: `bicep.{l,r}` → `forearm.{l,r}` → `wrist.{l,r}` (plus fingers and unused IK-target bones). Every bone points down its local **+Y** toward its child.

## Scripts

**player.gd** (`RigidBody3D`) — only restarts the scene on `R`. All movement comes from the arms.

**arms.gd** (`Node3D` on the arms.glb instance) — drives both arms:
- Analytic 2-bone IK per arm; only bone *rotations* are set (lengths from rest), so aiming a bone's +Y at the next joint moves the chain.
- Mouse target = mouse ray projected onto the camera-facing plane through the shoulders.
- Reach is clamped to the nearest obstacle by sweeping a thin sphere (radius `arm_thickness`, via `cast_motion`) down each bone segment from the shoulder — a "fat raycast" so the arm clears colliders by its real width instead of a 1-D line (excludes the torso; `Area3D`s don't block). This clamp runs only for a *free* (posing) arm; it's skipped while gripping (`_solve_arm`'s `clamp_reach=false`) so the hand stays pinned to the anchor instead of the elbow folding/unfolding as the body orbits.
- Push (`_push_off`): every frame a posed free arm whose hand touches a surface (shoulder→cursor raycast hits) reflects the surface-resisted part of the commanded hand motion back onto the torso's `linear_velocity` (`push_gain`, per-frame add capped by `max_push_speed`). Motion *away* from the surface is dropped, and a motionless hand adds nothing.
- Grab: overlap-test a sphere of radius `grab_margin` at the hand tip (only the palm grabs, not the forearm/elbow); on hit, lock the hand's IK target to the anchor and record the torso→anchor distance as the orbit radius. The anchor is the hand tip, **except** on a `Bar` (duck-typed via `axis_center()`), where it's the center of the bar's circular cross-section so the torso orbits the true center. **No physics joint** — the hold is the IK glue plus the pivot below.
- Pivot/swing (`_pivot_body`): a grip is *active* while its pose key is held and *passive* otherwise. For each active hand the cursor steers *relative* (like a steering wheel): on the first steer frame the body's current orbit angle is pinned, then the body angle tracks however far the cursor sweeps around the anchor (so a grab never snaps the body across the pivot). target = anchor + that angle at orbit radius; the torso's `linear_velocity` is driven toward the average active target (`swing_strength`, capped by `max_swing_speed`). With every grip passive (no pose key) the drive is skipped and `_constrain_to_rope` instead holds the body to the orbit radius (cancels along-arm velocity + nudges drift back onto the circle), so gravity and the body's momentum swing it as a free pendulum rather than freezing it. While the pose key is held the orbit radius eases out toward full arm reach (`straighten_speed`) on **every** grab, so the arm pulls taut into a straight rigid spoke and the swing stays smooth instead of the elbow re-folding (capped per-grab at the arm's slack so the hand can't detach); key up freezes the radius. A 2-bone arm can only be straight at full extension, so this taut spoke *is* the "rigid pose." On a flat (non-bar) grab the contact normal + point are recorded and the velocity drive strips any component pushing the torso *into* that surface — but only while the body is within `surface_clearance` of it, so it can't jam/spasm against the ground/wall up close yet still swings the full arc once clear (bars orbit freely, no guard). Each frame it also tracks the body's angular speed `ω` around the anchor (in the play plane, EMA-smoothed by `spin_smooth`).
- Release fling (`_release`): on letting go, the torso is launched **tangentially** at its orbital speed `v = ω·r` in the spin direction (`fling_gain`, capped by `max_fling_speed`) — not the leftover drive velocity. Releasing while holding still (no spin) just drops you.

Exports: `max_reach 0.98`, `elbow_pole (0,-0.2,1)`, `follow_speed 12`, `grab_margin 0.25`, `surface_offset 0.1`, `arm_thickness 0.06`, `swing_strength 8`, `max_swing_speed 16`, `straighten_speed 6`, `fling_gain 1.0`, `max_fling_speed 20`, `spin_smooth 25`, `push_gain 8`, `max_push_speed 8`.

**bar.gd** (`class_name Bar`, `StaticBody3D`) — `scenes/obstacles/bar.gd`. A grabbable cylindrical bar (monkey-bar / high-bar). Exports `axis` (local axis the cylinder runs along, default +Y); `axis_center(world_point)` returns the point on that axis nearest the hand — the cross-section center the grab pivots on. The five `MonkeyBars/Bar*` in `tutorial_level.tscn` use it.

**camera_follow.gd** (`Camera3D`) — follows the Player on X/Y, holds Z (depth) constant, preserves the authored offset. `follow_speed 8` (0 = snap).

## Not yet built

- No goal, level geometry beyond the flat ground, HUD, hazards, or named collision layers (collision is default; the arm raycast just excludes the torso).
- The grip pivot is cursor-driven only while the pose key is held (velocity overwritten toward the orbit point); release the key and it becomes a free gravity pendulum via `_constrain_to_rope`. Multiple simultaneous passive grips aren't averaged — the rope constraints apply in sequence (last wins), which is approximate for a rare two-hand passive hold.
- The push is velocity-additive recoil, not a true contact force — it can't *hold* you against gravity (that's the grab's job); it only scoots you while the arm is in motion.

## Feel goals

Heavy/floppy arms (high angular damping), momentum matters (release flings), long catastrophic falls, no hand-holding.
