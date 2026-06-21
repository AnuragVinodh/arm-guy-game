extends Camera3D
## Follows a target on the X and Y axes while keeping a constant Z (depth), so
## the camera tracks the player as they climb/slide around but never changes how
## far back it sits. The starting offset in the scene is preserved.

## The node to follow (set to the Player in the scene).
@export var target_path: NodePath

## How quickly the camera catches up. 0 = snap instantly (no smoothing); higher
## is tighter, lower is laggier/floatier.
@export_range(0.0, 30.0, 0.5) var follow_speed: float = 8.0

var _target: Node3D
var _offset: Vector3 # captured X/Y gap between camera and target at startup
var _fixed_z: float  # the depth we lock to


func _ready() -> void:
	_target = get_node_or_null(target_path) as Node3D
	if _target == null:
		push_warning("camera_follow: target_path is not set to a Node3D")
		set_physics_process(false)
		return
	# Preserve whatever framing the scene was authored with.
	_offset = global_position - _target.global_position
	_fixed_z = global_position.z


func _physics_process(delta: float) -> void:
	# Follow in _physics_process so we stay in lockstep with the physics-driven
	# player and don't jitter.
	var desired := Vector3(
		_target.global_position.x + _offset.x,
		_target.global_position.y + _offset.y,
		_fixed_z, # Z never tracks the target — the depth stays constant.
	)
	if follow_speed <= 0.0:
		global_position = desired
	else:
		global_position = global_position.lerp(desired, clamp(follow_speed * delta, 0.0, 1.0))
