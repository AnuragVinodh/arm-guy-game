class_name Arm
extends Node2D

## Reusable arm node. Attach to a shoulder Marker2D that is child of a RigidBody2D torso.
## Solves 2-bone IK toward a target, creates/destroys PinJoint2D grabs.

@export var upper_arm_length: float = 55.0
@export var forearm_length: float = 50.0
@export var grab_radius: float = 14.0
@export var grab_layer_mask: int = 0b0101  # layers 1 (terrain) + 3 (grabbable)

var is_grabbing: bool = false
var target_world: Vector2 = Vector2.ZERO

var _grab_joint: PinJoint2D = null
var _elbow_pos: Vector2 = Vector2.ZERO
var _hand_pos: Vector2 = Vector2.ZERO


func _process(_delta: float) -> void:
	_solve_ik(target_world)
	queue_redraw()


# ── IK ──────────────────────────────────────────────────────────────────────

func _solve_ik(world_target: Vector2) -> void:
	var origin: Vector2 = global_position
	var to_target: Vector2 = world_target - origin
	var dist: float = clampf(to_target.length(), 1.0, upper_arm_length + forearm_length - 1.0)
	var dir: Vector2 = to_target.normalized()

	# Law of cosines to find elbow angle
	var a := upper_arm_length
	var b := forearm_length
	var c := dist

	var cos_a := clampf((a * a + c * c - b * b) / (2.0 * a * c), -1.0, 1.0)
	var angle_a := acos(cos_a)

	# Elbow bends "inward" (perpendicular offset, flip based on arm side)
	var elbow_bend := dir.rotated(angle_a * _bend_sign())
	_elbow_pos = origin + elbow_bend * a
	_hand_pos = _elbow_pos + (_hand_pos - _elbow_pos).normalized() * b if is_grabbing \
		else _elbow_pos + (world_target - _elbow_pos).normalized() * b


func _bend_sign() -> float:
	# Subclasses or export can flip this so left/right arm bends naturally
	return 1.0


# ── Draw ─────────────────────────────────────────────────────────────────────

func _draw() -> void:
	var origin_local := Vector2.ZERO
	var elbow_local := to_local(_elbow_pos)
	var hand_local := to_local(_hand_pos)

	draw_line(origin_local, elbow_local, Color.WHITE, 6.0, true)
	draw_line(elbow_local, hand_local, Color.WHITE, 5.0, true)
	# Hand circle
	var grab_color := Color.YELLOW if is_grabbing else Color.WHITE
	draw_circle(hand_local, 6.0, grab_color)


# ── Grab / Release ────────────────────────────────────────────────────────────

func try_grab() -> void:
	if is_grabbing:
		return

	var space := get_world_2d().direct_space_state
	var query := PhysicsPointQueryParameters2D.new()
	query.position = _hand_pos
	query.collision_mask = grab_layer_mask
	query.collide_with_bodies = true
	query.collide_with_areas = false

	var results := space.intersect_point(query, 1)
	if results.is_empty():
		return

	var hit_body: PhysicsBody2D = results[0]["collider"]
	_attach_grab(hit_body, _hand_pos)


func release() -> void:
	if not is_grabbing:
		return
	if is_instance_valid(_grab_joint):
		_grab_joint.queue_free()
	_grab_joint = null
	is_grabbing = false


func _attach_grab(body: PhysicsBody2D, point: Vector2) -> void:
	var torso: RigidBody2D = get_parent().get_parent() as RigidBody2D
	if torso == null:
		return

	_grab_joint = PinJoint2D.new()
	_grab_joint.position = point
	_grab_joint.node_a = torso.get_path()
	_grab_joint.node_b = body.get_path()
	_grab_joint.softness = 0.0
	get_tree().current_scene.add_child(_grab_joint)

	is_grabbing = true
	_hand_pos = point  # lock hand to grab point
