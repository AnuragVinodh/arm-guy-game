# Arm Guy Game

Physics rage/climbing game (Getting Over It / Only Up). A torso with two arms; the arms are the only way to move. **Godot 4.5, full 3D**, locked to the X-Y plane for 2.5D play. (Earlier design notes said 2D / `RigidBody2D` / `PinJoint2D` — that is obsolete; everything below is 3D.)

## Controls

| Input | Action |
|-------|--------|
| Hold **A** | aim/pose the **left** arm at the mouse |
| Hold **D** | aim/pose the **right** arm at the mouse |
| Hold **Left click** | grab with the **right** hand (crossed mapping) |
| Hold **Right click** | grab with the **left** hand |
| **R** | restart scene |

Released arms freeze in place. Grabbing locks the hand to the contact point; while gripping, dragging the mouse hauls the body toward the cursor (leashed within arm's reach) — this is the climb/hoist. Releasing keeps the body's velocity (fling).

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
- Reach is clamped to the nearest obstacle via a raycast from shoulder along the arm (excludes the torso; `Area3D`s don't block) so hands don't clip through colliders.
- Grab: raycast shoulder→palm (+`grab_margin`); on hit, lock the hand's IK target to the contact point. **No physics joint** — the hold is the IK glue plus the pull below.
- Pull/hoist (`_pull_body`): for each gripping hand, target = cursor leashed within arm reach of the grab point; the torso's `linear_velocity` is driven toward the average target (`pull_strength`, capped by `max_pull_speed`).

Exports: `max_reach 0.98`, `elbow_pole (0,-0.2,1)`, `follow_speed 12`, `grab_margin 0.25`, `pull_strength 12`, `max_pull_speed 16`.

**camera_follow.gd** (`Camera3D`) — follows the Player on X/Y, holds Z (depth) constant, preserves the authored offset. `follow_speed 8` (0 = snap).

## Not yet built

- No goal, level geometry beyond the flat ground, HUD, hazards, or named collision layers (collision is default; the arm raycast just excludes the torso).
- No free pendulum *swing* while gripping (the grip is a controllable pull, not a rigid pivot).
- Pull direction is "body follows cursor"; inverting to a rope-style pull is a sign flip.

## Feel goals

Heavy/floppy arms (high angular damping), momentum matters (release flings), long catastrophic falls, no hand-holding.
