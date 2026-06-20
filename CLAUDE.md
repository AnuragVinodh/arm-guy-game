# Arm Guy Game

## Concept

2.5D physics-based rage/climbing game inspired by Bennett Foddy (Getting Over It) and Only Up. You control a torso with two arms. The arms are the only way to move — grab ledges, surfaces, and objects to drag, pull, and hoist the torso upward toward a goal. No legs. No jumps. Pure arm physics chaos.

## Core Mechanic

- **Left mouse drag** → controls left arm angle/reach around left shoulder
- **Right mouse drag** → controls right arm angle/reach around right shoulder
- **Left click** → left hand grabs whatever it's touching
- **Right click** → right hand grabs whatever it's touching
- When a hand grabs a surface a `PinJoint2D` is created, tethering that point to the torso
- Using both arms together lets you swing, pull, climb, and fling the torso

## Architecture

```
Player (RigidBody2D)          ← torso, fully physics-driven, no direct movement
  ├── LeftShoulder (Marker2D) ← pivot point for left arm
  │     └── LeftArm (Arm)     ← arm node, handles IK + grab
  └── RightShoulder (Marker2D)
        └── RightArm (Arm)
```

**Arm.gd** — reusable arm class:
- Tracks a target position (mouse-driven)
- Solves 2-bone IK (upper arm + forearm)
- On grab: spawns a `PinJoint2D` connecting torso to grab point
- On release: removes the joint

**Player.gd** — routes mouse input to left/right arm, applies damping/stabilization to torso.

## Scenes

| Scene | Purpose |
|-------|---------|
| `scenes/player/player.tscn` | The arm guy — torso + 2 arms |
| `scenes/levels/level_01.tscn` | First test level with platforms |
| `scenes/ui/hud.tscn` | Minimal HUD (height tracker, death counter) |
| `scenes/main.tscn` | Boot scene — loads level + player |

## 2.5D Visual Style

- 2D physics (simpler, more controllable for a jam)
- Parallax background layers to sell depth
- Player and foreground elements on z=0, mid/bg on negative z layers
- Camera follows torso with look-ahead in the up direction

## Layers (project settings)

| Layer | Name | Used for |
|-------|------|---------|
| 1 | terrain | Static platforms/walls the arms can grab |
| 2 | player | Player torso collision |
| 3 | grabbable | Dynamic objects that can be dragged |
| 4 | hazard | Death/reset zones |

## Input Map (to configure in Project → Input Map)

| Action | Default |
|--------|---------|
| `grab_left` | Left Mouse Button |
| `grab_right` | Right Mouse Button |
| `restart` | R |

## Feel Goals

- Arms should feel heavy and floppy — high angular damping on torso
- Momentum matters — swinging builds speed, letting go flings you
- Falling should feel catastrophic (long falls, dramatic)
- No hand-holding — rage game by design
