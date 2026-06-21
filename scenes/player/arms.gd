extends Node3D
## Arm controller — drives the rigged arms (a Skeleton3D inside arms.glb) with
## analytic 2-bone inverse kinematics so both arms reach toward the mouse.
##
## Each arm is a 2-bone chain:  bicep (upper arm) -> forearm -> wrist (hand).
## Every bone in this rig points down its own local +Y axis toward its child
## (verified from the glb: each child sits at a positive Y offset). That single
## fact is what makes the IK below simple — to aim a bone we just rotate it so
## its +Y points at the next joint, and the child follows automatically because
## we keep the bones at their rest *lengths* and only change their *rotations*.
##
## Think of it as a 2-bone pendulum hanging off each shoulder, chasing a target.

## The two arm chains in the skeleton. Bone names come straight from the glb.
## `key` is the keyboard key that activates that arm — hold it to make the arm
## track the mouse; release it and the arm freezes in its last pose.
const ARMS := [
	{ "bicep": "bicep.l", "forearm": "forearm.l", "wrist": "wrist.l", "key": KEY_A },
	{ "bicep": "bicep.r", "forearm": "forearm.r", "wrist": "wrist.r", "key": KEY_B },
]

## How far the hand can reach, as a fraction of the full arm length. Keeping this
## below 1.0 stops the arm from snapping bolt-straight when the mouse is far away.
@export_range(0.1, 1.0, 0.01) var max_reach: float = 0.98

## Which way the elbow bows. In the SCREEN/climbing plane the arms work in the
## X-Y plane, so a small Z pole pushes the elbows out toward the camera and keeps
## them from collapsing into a straight line. Tweak to taste.
@export var elbow_pole: Vector3 = Vector3(0.0, -0.2, 1.0)

## Smoothing — higher = snappier, lower = floppier / more pendulum-like.
@export_range(1.0, 40.0, 0.5) var follow_speed: float = 12.0

var _skeleton: Skeleton3D
var _camera: Camera3D

## Cached per-arm bone data so we don't look it up every frame.
##   bicep_idx, forearm_idx, wrist_idx : bone indices
##   l1, l2                            : upper-arm and forearm rest lengths
##   root                              : skeleton-space position of the shoulder
var _chains: Array = []

## The world-space point both hands are currently reaching toward (smoothed).
var _target_world: Vector3


func _ready() -> void:
	# arms.gd is attached to the glb instance root; the Skeleton3D is somewhere
	# below it. Find it instead of hard-coding a path, so re-imports can't break us.
	_skeleton = _find_skeleton(self)
	if _skeleton == null:
		push_error("arms.gd: no Skeleton3D found under %s" % name)
		set_physics_process(false)
		return

	_camera = get_viewport().get_camera_3d()

	for arm in ARMS:
		var bicep_idx := _skeleton.find_bone(arm["bicep"])
		var forearm_idx := _skeleton.find_bone(arm["forearm"])
		var wrist_idx := _skeleton.find_bone(arm["wrist"])
		if bicep_idx == -1 or forearm_idx == -1 or wrist_idx == -1:
			push_warning("arms.gd: missing bone in chain %s" % [arm])
			continue

		# Rest lengths: a bone's length is just how far its child sits along +Y.
		var l1 := _skeleton.get_bone_rest(forearm_idx).origin.length()
		var l2 := _skeleton.get_bone_rest(wrist_idx).origin.length()

		_chains.append({
			"bicep": bicep_idx,
			"forearm": forearm_idx,
			"wrist": wrist_idx,
			"l1": l1,
			"l2": l2,
			"key": arm["key"],
		})

	# Start the target somewhere sensible so the first frame doesn't lurch.
	_target_world = global_position


func _physics_process(delta: float) -> void:
	if _skeleton == null or _chains.is_empty():
		return
	if _camera == null:
		_camera = get_viewport().get_camera_3d()
		if _camera == null:
			return

	# 1. Turn the mouse cursor into a world-space target, then ease toward it.
	var goal := _mouse_target()
	var t: float = clamp(follow_speed * delta, 0.0, 1.0)
	_target_world = _target_world.lerp(goal, t)

	# 2. Targets/poses for the skeleton live in skeleton-LOCAL space, so convert
	#    the world target into that space once and reuse it for both arms.
	var to_skel := _skeleton.global_transform.affine_inverse()
	var target_local := to_skel * _target_world

	# 3. Solve each arm independently — but only while its key is held. A
	#    released arm is simply skipped, so it stays frozen in its last pose.
	for chain in _chains:
		if Input.is_key_pressed(chain["key"]):
			_solve_arm(chain, target_local)


## Project the mouse ray onto the vertical plane the player climbs in (the plane
## through the skeleton, facing the camera) and return that world point.
func _mouse_target() -> Vector3:
	var mouse := get_viewport().get_mouse_position()
	var ray_origin := _camera.project_ray_origin(mouse)
	var ray_dir := _camera.project_ray_normal(mouse)

	# Plane passes through the shoulders, with a normal facing the camera so the
	# arms swing in the X-Y "screen" plane like a 2D pendulum.
	var plane_point := _skeleton.global_position
	var plane_normal := Vector3.BACK # +Z; the 2.5D play plane faces the camera.
	var plane := Plane(plane_normal, plane_point.dot(plane_normal))

	var hit = plane.intersects_ray(ray_origin, ray_dir)
	return hit if hit != null else _target_world


## Analytic two-bone IK for one arm, all in skeleton-local space.
func _solve_arm(chain: Dictionary, target: Vector3) -> void:
	var bicep_idx: int = chain["bicep"]
	var forearm_idx: int = chain["forearm"]
	var l1: float = chain["l1"]
	var l2: float = chain["l2"]

	# Shoulder anchor — this joint doesn't move, the arm pivots around it.
	var a := _skeleton.get_bone_global_pose(bicep_idx).origin

	var to_target := target - a
	var dist := to_target.length()
	if dist < 1e-4:
		return

	# Clamp the reach so the arm can't over-extend (and never hits the singular
	# fully-straight or fully-folded cases that make the math blow up).
	var reach_max: float = (l1 + l2) * max_reach
	var reach_min: float = absf(l1 - l2) + 1e-3
	dist = clamp(dist, reach_min, reach_max)

	var dir := to_target.normalized()

	# Law of cosines: how far along the shoulder->target line the elbow projects,
	# and how far off to the side it bends.
	var cos_shoulder: float = clamp((l1 * l1 + dist * dist - l2 * l2) / (2.0 * l1 * dist), -1.0, 1.0)
	var along := l1 * cos_shoulder
	var aside := l1 * sqrt(max(0.0, 1.0 - cos_shoulder * cos_shoulder))

	# Bend direction: the part of the pole that's perpendicular to the arm line.
	var bend := (elbow_pole - dir * elbow_pole.dot(dir))
	if bend.length() < 1e-4:
		# Pole is parallel to the arm; fall back to any perpendicular axis.
		bend = dir.cross(Vector3.RIGHT)
		if bend.length() < 1e-4:
			bend = dir.cross(Vector3.UP)
	bend = bend.normalized()

	# Elbow position, then the two bone directions.
	var elbow := a + dir * along + bend * aside
	var bicep_dir := (elbow - a).normalized()      # shoulder -> elbow
	var forearm_dir := (target - elbow).normalized() # elbow -> wrist

	# Aim each bone's +Y down its direction. Use `bend` as the roll reference so
	# the elbow consistently hinges in the bend plane.
	var bicep_global := _aim_y(bicep_dir, bend)
	var forearm_global := _aim_y(forearm_dir, bend)

	# Convert the desired GLOBAL (skeleton-space) orientations into LOCAL bone
	# rotations and apply them. Order matters: set the bicep first, because the
	# forearm's parent IS the bicep — its desired local rotation is measured
	# relative to the bicep's new orientation.
	var bicep_parent := _skeleton.get_bone_parent(bicep_idx)
	var bicep_parent_basis := _skeleton.get_bone_global_pose(bicep_parent).basis.orthonormalized()
	var bicep_local := bicep_parent_basis.inverse() * bicep_global
	_skeleton.set_bone_pose_rotation(bicep_idx, bicep_local.get_rotation_quaternion())

	# Forearm is parented to the bicep, so measure against the bicep's new basis.
	var forearm_local := bicep_global.inverse() * forearm_global
	_skeleton.set_bone_pose_rotation(forearm_idx, forearm_local.get_rotation_quaternion())


## Build an orthonormal basis whose +Y points along `y_dir`, using `roll_ref`
## to pin down the rotation around that axis (the "roll").
func _aim_y(y_dir: Vector3, roll_ref: Vector3) -> Basis:
	var yb := y_dir.normalized()
	var xb := roll_ref.cross(yb)
	if xb.length() < 1e-5:
		# roll_ref is parallel to y_dir; pick any stable perpendicular.
		var fallback := Vector3.RIGHT if absf(yb.x) < 0.9 else Vector3.FORWARD
		xb = fallback.cross(yb)
	xb = xb.normalized()
	var zb := xb.cross(yb).normalized()
	return Basis(xb, yb, zb)


## Depth-first search for the first Skeleton3D under `node`.
func _find_skeleton(node: Node) -> Skeleton3D:
	for child in node.get_children():
		if child is Skeleton3D:
			return child
		var found := _find_skeleton(child)
		if found != null:
			return found
	return null
