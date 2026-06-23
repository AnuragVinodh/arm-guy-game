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
##   key    — hold to pose that (free) arm with the mouse; release and it freezes.
##            While posed, a hand pressed on a surface pushes the body (see
##            _push_off) — that's how you scoot around without grabbing.
##   button — hold that mouse button to GRAB whatever the palm is touching, turning
##            it into a pivot the body swings around; release to let go. Mapping is
##            crossed: left click grabs with the RIGHT hand, right click the LEFT.
const ARMS := [
	{ "bicep": "bicep.l", "forearm": "forearm.l", "wrist": "wrist.l", "key": KEY_A, "button": MOUSE_BUTTON_RIGHT },
	{ "bicep": "bicep.r", "forearm": "forearm.r", "wrist": "wrist.r", "key": KEY_D, "button": MOUSE_BUTTON_LEFT },
]

## How far the hand can reach, as a fraction of the full arm length. Keeping this
## below 1.0 stops the arm from snapping bolt-straight when the mouse is far away.
@export_range(0.1, 1.0, 0.01) var max_reach: float = 0.98

## Which way the elbow bows, in WORLD space. The play happens in the X-Y plane,
## so a pure +Z (toward-camera) pole bends the elbow out of the screen — which
## means that when the arm has to fold (mouse close, or reach clamped at an
## obstacle) the elbow swings toward the viewer instead of rotating sideways
## across the screen. Keep it on +Z to avoid that sideways "arm rotates inward"
## look; add a small X/Y lean only if you want a more natural in-plane bend and
## can live with some screen-plane swing. NOTE: this is world space on purpose —
## the rig itself is rotated, so a rig-local pole would leak into the screen
## plane and produce exactly the rotation we're avoiding.
@export var elbow_pole: Vector3 = Vector3(0.0, 0.0, 1.0)

## Smoothing — higher = snappier, lower = floppier / more pendulum-like.
@export_range(1.0, 40.0, 0.5) var follow_speed: float = 12.0

## How close (world units) the palm must be to a surface to be able to grab it.
@export_range(0.0, 2.0, 0.05) var grab_margin: float = 0.25

## Skin/back-off (world units) kept between the arm and any surface it bumps into.
## The IK solves the *wrist bone*, but the visible hand and fingers stick out past
## it, so we stop the arm this far short of a collider to keep them from poking
## through. Bump it up if hands still clip; down if the arm stops too far away.
@export_range(0.0, 0.5, 0.01) var surface_offset: float = 0.1

## Arm thickness (world units) for the reach-clamp's shape-cast. The clamp sweeps a
## sphere of this radius down each bone segment instead of a 1-D ray, so the arm
## clears colliders by its real width and can't slip a thin edge between rays.
## Bigger = the posing arm keeps more distance from surfaces (total clearance is
## roughly arm_thickness + surface_offset). Only the free-arm clip-avoidance uses
## this — the clamp is skipped while gripping, so it doesn't affect the swing.
@export_range(0.0, 0.3, 0.01) var arm_thickness: float = 0.06

## GRAB = PIVOT. A gripping hand is pinned to its grab point and the body orbits
## that point. While that arm's pose key is held the cursor steers like a steering
## wheel — rotating it around the anchor turns the body the SAME way, starting from
## wherever the body is when steering begins (so a grab never snaps the body across
## the pivot). Release the key and the grip goes passive (free pendulum).
## `swing_strength` is how hard it chases the steered angle; `max_swing_speed` caps
## the swing so a fast cursor flick doesn't teleport.
@export_range(1.0, 40.0, 0.5) var swing_strength: float = 18.0
@export var max_swing_speed: float = 16.0

## How far (world units) the cursor must sit from the grab pivot before it steers.
## Inside this radius a tiny mouse move sweeps a huge angle around the pivot (and
## crossing the pivot flips it ~180°), which would spin the body — so within it the
## swing just holds its current angle, ignoring how close the cursor is. Bigger =
## a wider neutral zone around the pivot.
@export_range(0.0, 2.0, 0.05) var steer_deadzone: float = 0.35

## STRAIGHTEN-ON-GRIP. While a grabbed arm's pose key is held, its orbit radius
## eases out toward the arm's full reach (`straighten_speed`, world units/sec) so a
## folded arm pulls taut into a straight, rigid spoke — that's what keeps the swing
## smooth instead of the elbow re-folding at different angles as you go around.
## Capped per-grab to the arm's actual slack so the hand can't detach; key up
## freezes the current radius.
@export_range(0.0, 40.0, 0.5) var straighten_speed: float = 18.5

## RELEASE FLING. Let go mid-swing and the body launches tangentially at the speed
## it was orbiting (v = ω·r), in the direction it was rotating — wind up a swing,
## release, and you fly off the arc. `fling_gain` scales it (1.0 = true orbital
## speed), `max_fling_speed` caps the launch, and `spin_smooth` smooths the measured
## spin so one jittery frame can't spike it. Let go while holding still → no spin →
## you just drop.
@export var fling_gain: float = 1.0
@export var max_fling_speed: float = 20.0
@export_range(1.0, 60.0, 1.0) var spin_smooth: float = 25.0

## ALWAYS-ON ARM PUSH. A *free* (posed, not gripping) arm whose hand is touching a
## surface shoves the body when you rotate the arm — pressing into the surface
## pushes you off along its normal, sweeping along it recoils you opposite the
## sweep. This is how you scoot/crawl without grabbing. `push_gain` scales the
## shove per unit of commanded hand motion; `max_push_speed` caps a single frame's
## shove. Only arm *motion* pushes (a still hand resting on a wall does nothing),
## so there's no jetpack — that holding job belongs to the grab.
@export_range(0.0, 60.0, 0.5) var push_gain: float = 8
@export var max_push_speed: float = 8.0

var _skeleton: Skeleton3D
var _camera: Camera3D

## The torso this arm rig hangs off — the RigidBody3D a grab pins to the world.
var _torso: PhysicsBody3D

## Physics bodies the reach-raycast should ignore (the player's own torso, so an
## arm never "collides" with the body it's attached to).
var _exclude: Array[RID] = []

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

	# Walk up to the owning physics body (the player torso) and exclude it from
	# the reach-raycast, so an arm doesn't stop short on its own collision box.
	var p := get_parent()
	while p != null:
		if p is PhysicsBody3D:
			_torso = p as PhysicsBody3D
			_exclude.append(_torso.get_rid())
			break
		p = p.get_parent()

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
			"button": arm["button"],
			"grabbed": false,
			"anchor": Vector3.ZERO, # world-space grab point while grabbed
			"pivot_r": 0.0,         # body's orbit radius around the grab anchor
			"pivot_r_max": 0.0,     # radius the arm straightens out to while gripping
			"on_bar": false,        # grabbed a Bar? (bars orbit free; flats get a guard)
			"grab_normal": Vector3.UP, # surface normal at grab — anti-spasm half-space (flats)
			"prev_desired": null,   # last frame's commanded hand pos (world), for push
			"swing_angle": 0.0,     # body's angle around the anchor (play plane), for fling
			"swing_omega": 0.0,     # smoothed angular speed of the swing, rad/s
			"steering": false,        # mid active-steer session? (relative cursor steering)
			"steer_body_ang": 0.0,    # body's orbit angle the current steer started from (rad)
			"steer_cursor_ang": 0.0,  # last cursor angle around the anchor (rad)
			"steer_has_cursor": false,# was the cursor off the pivot last steer frame?
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
	#    world points into that space. The mouse target is shared by both arms;
	#    a grabbed arm uses its own fixed anchor instead.
	var to_skel := _skeleton.global_transform.affine_inverse()
	var mouse_target_local := to_skel * _target_world

	for chain in _chains:
		# 3. Grab / release: hold the button to grip, release to let go.
		var holding: bool = Input.is_mouse_button_pressed(chain["button"])
		if holding and not chain["grabbed"]:
			_try_grab(chain)
		elif not holding and chain["grabbed"]:
			_release(chain)

		# 4. Solve the arm. A grabbed hand stays glued to its world anchor (so it
		#    holds on as the body swings around it); otherwise it tracks the mouse
		#    while its pose key is held — and while tracking, a free hand that
		#    touches a surface shoves the body (the always-on push); otherwise the
		#    arm is left frozen in place.
		if chain["grabbed"]:
			# Don't clamp the reach while gripping: the hand must stay glued to the
			# anchor, and the surface-avoidance raycast would otherwise pull it off and
			# make the elbow fold/unfold as the body orbits (clunky). Clipping is fine
			# here — you're holding on.
			_solve_arm(chain, to_skel * chain["anchor"], false)
			chain["prev_desired"] = null
		elif Input.is_key_pressed(chain["key"]):
			_solve_arm(chain, mouse_target_local)
			_push_off(chain)
		else:
			chain["prev_desired"] = null

	# 5. Swing the body. Every gripping hand is a pivot the body orbits at fixed
	#    radius; the cursor sets the angle while that arm's pose key is held. We
	#    drive the torso toward the average of those orbit points. This is the
	#    climb/swing.
	_pivot_body(delta)


## Try to grab whatever the palm is touching. Overlap-tests a small sphere
## (radius `grab_margin`) at the hand tip; if it finds a solid collider, we pin
## the torso's pivot to the hand. Only the palm can grab — because the test sits
## at the tip, the forearm/elbow can't trigger a grab and fold the arm onto a wall.
func _try_grab(chain: Dictionary) -> void:
	if _torso == null:
		return
	var space := _skeleton.get_world_3d().direct_space_state
	if space == null:
		return

	var hand: Vector3 = _skeleton.global_transform * _skeleton.get_bone_global_pose(chain["wrist"]).origin

	# A sphere at the hand catches a surface the palm rests against from any angle.
	# (Like the old ray it ignores Area3Ds, so only solid colliders can be grabbed.)
	var shape := SphereShape3D.new()
	shape.radius = grab_margin
	var query := PhysicsShapeQueryParameters3D.new()
	query.shape = shape
	query.transform = Transform3D(Basis(), hand)
	query.exclude = _exclude
	var hits := space.intersect_shape(query, 1)
	if hits.is_empty():
		return

	# Pivot on the hand tip itself (not the surface point), so the arm stays
	# extended and the body swings around the palm. A Bar is the exception: it
	# pivots on its cross-section center (the bar axis), so the torso orbits the
	# true center and can spin around the bar without the arm jamming off-center.
	var collider: Object = hits[0]["collider"]
	var anchor: Vector3 = hand
	# Duck-typed so this file needn't depend on the Bar class: anything exposing
	# axis_center() (i.e. a Bar) supplies its own cross-section-center pivot.
	if collider != null and collider.has_method("axis_center"):
		anchor = collider.call("axis_center", hand)
		anchor.z = _torso.global_position.z  # keep the swing in the 2.5D play plane
	# Surface normal at the grab, for the flat-surface anti-spasm guard. Orient it to
	# point from the surface toward the body (the open side) so the half-space test in
	# _pivot_body is correct regardless of the engine's normal convention.
	var rest := space.get_rest_info(query)
	var gn: Vector3 = rest["normal"] if rest.has("normal") else Vector3.UP
	if gn.dot(_torso.global_position - anchor) < 0.0:
		gn = -gn
	chain["grab_normal"] = gn
	_grab(chain, anchor, collider)


## Lock the hand onto `anchor_world` and turn it into a pivot. We don't use a
## rigid joint; instead we remember the grab point and the body's distance to it
## (the orbit radius), and _pivot_body() swings the torso around it.
func _grab(chain: Dictionary, anchor_world: Vector3, body: Object) -> void:
	chain["anchor"] = anchor_world
	chain["pivot_r"] = maxf(0.05, _torso.global_position.distance_to(anchor_world))
	chain["grabbed"] = true
	# Bars orbit freely all the way around; flat grabs instead get the anti-spasm
	# half-space guard in _pivot_body. Both straighten the arm out toward a rigid
	# spoke (so the elbow stops re-folding as the body orbits).
	chain["on_bar"] = body != null and body.has_method("axis_center")
	# Fresh grab: clear any stale steer session so the first steer frame re-pins the
	# body's angle here (no snap across the pivot).
	chain["steering"] = false
	# How far the radius may straighten out: extend only by the arm's current slack
	# (full reach from the shoulder minus how extended it already is), so pulling the
	# arm taut into a spoke can't yank the hand off the anchor.
	var world_scale: float = _skeleton.global_transform.basis.get_scale().x
	var reach_max: float = (chain["l1"] + chain["l2"]) * max_reach * world_scale
	var shoulder: Vector3 = _skeleton.global_transform * _skeleton.get_bone_global_pose(chain["bicep"]).origin
	var slack: float = maxf(0.0, reach_max - shoulder.distance_to(anchor_world))
	chain["pivot_r_max"] = chain["pivot_r"] + slack * 0.95
	# Seed swing tracking from the current angle so the first frame reads ~0 spin
	# (not a spike) — the fling builds up only as you actually start swinging.
	var rel := _torso.global_position - anchor_world
	chain["swing_angle"] = atan2(rel.y, rel.x)
	chain["swing_omega"] = 0.0


## Let go. The torso is launched tangentially at the swing's orbital speed
## (v = ω·r), in the direction it was rotating, so a well-timed release flings you
## along the arc. Holding still at release (no spin) just drops you.
func _release(chain: Dictionary) -> void:
	chain["grabbed"] = false
	if _torso == null:
		return
	var rel: Vector3 = _torso.global_position - chain["anchor"]
	rel.z = 0.0
	# Rotating rel by 90° gives the tangent direction; scaling by the signed angular
	# speed turns the spin into the matching orbital velocity (magnitude |ω|·r).
	var fling: Vector3 = Vector3(-rel.y, rel.x, 0.0) * (chain["swing_omega"] * fling_gain)
	if fling.length() > max_fling_speed:
		fling = fling.normalized() * max_fling_speed
	_torso.linear_velocity = fling


## Swing the torso around each gripping hand. The hand is a fixed pivot the body
## orbits at the grab radius. Holding the arm's pose key actively steers the swing
## toward the cursor; with the key up the grip goes passive — gravity and the body's
## own momentum swing it as a free pendulum on the arm (we just hold it to the rope),
## so letting go of A/D hands control back to physics instead of freezing the body.
## With nothing gripping, gravity rules outright.
func _pivot_body(delta: float) -> void:
	if _torso == null:
		return

	var target := Vector3.ZERO
	var active := 0
	var passive: Array = []
	var block_normal := Vector3.ZERO  # blocked into-surface dir from active flat grabs
	for chain in _chains:
		if not chain["grabbed"]:
			continue
		var to_body: Vector3 = _torso.global_position - chain["anchor"]

		# Track how fast the body is sweeping around the anchor (in the play plane),
		# EMA-smoothed, so _release can fling it off tangentially at that speed.
		var ang := atan2(to_body.y, to_body.x)
		var d_ang := wrapf(ang - chain["swing_angle"], -PI, PI)
		chain["swing_angle"] = ang
		var inst_omega := d_ang / maxf(delta, 1e-5)
		chain["swing_omega"] = lerpf(chain["swing_omega"], inst_omega, clampf(spin_smooth * delta, 0.0, 1.0))

		if Input.is_key_pressed(chain["key"]):
			# ACTIVE: the cursor steers like a steering wheel — rotating it around the
			# anchor rotates the body the SAME way, starting from wherever the body is
			# when steering begins. So grabbing/holding never snaps the body across the
			# pivot; it only turns as you turn the cursor. Ease the radius out toward
			# full reach so the arm straightens into a rigid spoke (the smooth swing);
			# a flat grab records its surface normal so the drive below can't shove the
			# torso into that surface as it straightens (the anti-spasm guard).
			chain["pivot_r"] = move_toward(chain["pivot_r"], chain["pivot_r_max"], straighten_speed * delta)
			if not chain["on_bar"]:
				block_normal += chain["grab_normal"]
			# Begin a steer session on the first active frame: pin the body's CURRENT
			# angle so there's no jump; the cursor only adds rotation from here.
			if not chain["steering"]:
				chain["steering"] = true
				chain["steer_body_ang"] = atan2(to_body.y, to_body.x)
				chain["steer_has_cursor"] = false
			# Steer only when the cursor is clear of the pivot. Inside steer_deadzone the
			# angle is hypersensitive (and flips ~180° across the pivot), so we hold the
			# angle and DROP the reference — re-pinning when the cursor leaves the zone so
			# crossing it doesn't whip the body. This removes the cursor-distance spin.
			var cursor_rel: Vector3 = _target_world - chain["anchor"]
			if cursor_rel.length() > steer_deadzone:
				var cursor_ang: float = atan2(cursor_rel.y, cursor_rel.x)
				if chain["steer_has_cursor"]:
					chain["steer_body_ang"] += wrapf(cursor_ang - chain["steer_cursor_ang"], -PI, PI)
				chain["steer_cursor_ang"] = cursor_ang
				chain["steer_has_cursor"] = true
			else:
				chain["steer_has_cursor"] = false
			var ba: float = chain["steer_body_ang"]
			target += chain["anchor"] + Vector3(cos(ba), sin(ba), 0.0) * chain["pivot_r"]
			active += 1
		else:
			# PASSIVE: key up — no cursor steering. End the steer session (so the next
			# one re-pins) and let gravity/momentum drive it via the rope (below).
			chain["steering"] = false
			passive.append(chain)

	# An actively-steered grip wins: velocity-drive the torso toward the cursor spot.
	# (The release fling is computed separately in _release from the tracked spin.)
	if active > 0:
		target /= active
		var to_target := target - _torso.global_position
		var vel := to_target * swing_strength
		if vel.length() > max_swing_speed:
			vel = vel.normalized() * max_swing_speed
		# Anti-spasm: never drive the torso INTO a grabbed flat surface — it would jam
		# against the collider and oscillate as the straighten pushes it back. Strip the
		# into-surface part so the body can only slide along the surface or pull away.
		if block_normal.length() > 1e-4:
			var bn := block_normal.normalized()
			var into := vel.dot(bn)
			if into < 0.0:
				vel -= bn * into
		vel.z = 0.0
		_torso.linear_velocity = vel
		return

	# Otherwise every grip is passive: let the body swing/fall as a pendulum on the
	# rope instead of freezing it in place.
	for chain in passive:
		_constrain_to_rope(chain, delta)


## Keep a passively-gripping body on its rope (the orbit radius) WITHOUT driving it:
## cancel the velocity along the arm and nudge any drift back onto the circle, so
## gravity and the body's own momentum carry it as a free pendulum around the anchor.
## This is what makes a key-up grip swing under gravity instead of freezing.
func _constrain_to_rope(chain: Dictionary, delta: float) -> void:
	var rel: Vector3 = _torso.global_position - chain["anchor"]
	rel.z = 0.0
	var r := rel.length()
	if r < 1e-4:
		return
	var n := rel / r  # radial (along-the-arm) direction
	var v: Vector3 = _torso.linear_velocity
	# Fixed-length spoke: drop the radial part so the body can only move tangentially
	# (gravity's pull along the arm turns into swing, not stretch).
	v -= n * v.dot(n)
	# Position fix via velocity: pull the body back onto the circle if it has drifted,
	# so the hand neither detaches nor creeps toward the anchor.
	v += n * ((chain["pivot_r"] - r) / maxf(delta, 1e-5))
	v.z = 0.0
	_torso.linear_velocity = v


## Always-on arm push for a free (posed, non-gripping) arm. If the hand is pressed
## against a surface, *rotating* the arm shoves the torso: the part of the
## commanded hand motion the surface resists (into it, or sweeping along it) is
## reflected back onto the body — push into a wall and you peel off its normal,
## swipe along the ground and you scoot the opposite way. Lifting the hand away
## from the surface, or holding it still, does nothing (no jetpack).
func _push_off(chain: Dictionary) -> void:
	if _torso == null:
		return
	var space := _skeleton.get_world_3d().direct_space_state
	if space == null:
		return

	var xform := _skeleton.global_transform
	var shoulder: Vector3 = xform * _skeleton.get_bone_global_pose(chain["bicep"]).origin
	var hand: Vector3 = xform * _skeleton.get_bone_global_pose(chain["wrist"]).origin

	# Where the arm is *commanded* to put the hand: the cursor, clamped to reach.
	var world_scale := xform.basis.get_scale().x
	var reach_max: float = (chain["l1"] + chain["l2"]) * max_reach * world_scale
	var to_t := _target_world - shoulder
	var dlen := to_t.length()
	if dlen < 1e-4:
		chain["prev_desired"] = null
		return
	var desired := shoulder + (to_t / dlen) * minf(dlen, reach_max)

	# Only a hand actually touching a surface can push. Cast shoulder -> commanded
	# hand; a hit means the surface is in the way of where we're aiming.
	var query := PhysicsRayQueryParameters3D.create(shoulder, desired)
	query.exclude = _exclude
	var hit := space.intersect_ray(query)
	if hit.is_empty():
		chain["prev_desired"] = null
		return
	var n: Vector3 = hit["normal"]

	# How the commanded hand moved since last frame, with any motion *away* from
	# the surface dropped — you can't push off a surface by pulling back from it.
	var prev = chain["prev_desired"]
	chain["prev_desired"] = desired
	if prev == null:
		return
	var move: Vector3 = desired - prev
	var away := move.dot(n)
	if away > 0.0:
		move -= n * away
	if move.length() < 1e-6:
		return

	# The body recoils opposite that resisted motion.
	var push := -move * push_gain
	if push.length() > max_push_speed:
		push = push.normalized() * max_push_speed
	var vel: Vector3 = _torso.linear_velocity + push
	vel.z = 0.0
	_torso.linear_velocity = vel


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


## Analytic two-bone IK for one arm, all in skeleton-local space. `clamp_reach`
## pulls the arm in so its bones don't clip geometry — wanted for a free posing arm,
## but skipped while gripping so the hand stays pinned to its anchor (see caller).
func _solve_arm(chain: Dictionary, target: Vector3, clamp_reach: bool = true) -> void:
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

	# The bow pole is authored in world space (so it can point cleanly at the
	# camera regardless of how the rig is rotated); bring it into skeleton-local
	# space, which is where the IK math runs.
	var pole := (_skeleton.global_transform.affine_inverse().basis * elbow_pole).normalized()

	# Don't let the arm clip through solid geometry. The arm is a *bent* chain,
	# so checking the straight shoulder->hand line isn't enough — the elbow and
	# forearm can dip into a surface the hand isn't pointing at (e.g. an arm
	# lying along the ground). Pull the reach in until both real bone segments
	# clear all colliders, with a margin so the hand mesh doesn't poke through.
	# Skipped while gripping so the hand stays pinned to the anchor.
	if clamp_reach:
		dist = _clamp_arm(a, dir, dist, reach_min, l1, l2, pole)

	# Resolve the elbow at the final, safe reach.
	var solved := _bend_solve(a, dir, dist, l1, l2, pole)
	var elbow: Vector3 = solved["elbow"]
	var bend: Vector3 = solved["bend"]
	var wrist := a + dir * dist                      # exact analytic hand position

	var bicep_dir := (elbow - a).normalized()        # shoulder -> elbow
	var forearm_dir := (wrist - elbow).normalized()  # elbow -> wrist

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


## Solve the elbow for one arm at a given reach `dist`, in skeleton-local space.
## `pole` is the (skeleton-local) bow direction. Returns the elbow position and
## the bend direction (the side the elbow bows to).
func _bend_solve(a: Vector3, dir: Vector3, dist: float, l1: float, l2: float, pole: Vector3) -> Dictionary:
	# Law of cosines: how far along the shoulder->hand line the elbow projects,
	# and how far off to the side it bends.
	var cos_shoulder: float = clamp((l1 * l1 + dist * dist - l2 * l2) / (2.0 * l1 * dist), -1.0, 1.0)
	var along := l1 * cos_shoulder
	var aside := l1 * sqrt(max(0.0, 1.0 - cos_shoulder * cos_shoulder))

	# Bend direction: the part of the pole that's perpendicular to the arm line.
	var bend := (pole - dir * pole.dot(dir))
	if bend.length() < 1e-4:
		# Pole is parallel to the arm; fall back to any perpendicular axis.
		bend = dir.cross(Vector3.RIGHT)
		if bend.length() < 1e-4:
			bend = dir.cross(Vector3.UP)
	bend = bend.normalized()

	return { "elbow": a + dir * along + bend * aside, "bend": bend }


## Pull the reach `dist` in until neither bone segment (shoulder->elbow,
## elbow->wrist) cuts through a collider. Because shrinking the reach moves the
## elbow, we re-solve and re-test a few times so the pose converges. All inputs
## are skeleton-local; `reach_min` is the tightest the arm may ever fold.
func _clamp_arm(a: Vector3, dir: Vector3, dist: float, reach_min: float, l1: float, l2: float, pole: Vector3) -> float:
	var space := _skeleton.get_world_3d().direct_space_state
	if space == null:
		return dist

	var xform := _skeleton.global_transform
	for _i in 4:
		var solved := _bend_solve(a, dir, dist, l1, l2, pole)
		var elbow: Vector3 = solved["elbow"]
		var wrist := a + dir * dist

		# Test the upper arm first; if it's blocked, the forearm test is moot
		# because the elbow itself has to move.
		var h1 = _cast_local(space, xform, a, elbow)
		if h1 != null:
			# Shrink the whole arm toward the fraction of the bicep that's clear.
			var frac: float = clampf(a.distance_to(h1) / maxf(1e-4, a.distance_to(elbow)), 0.0, 1.0)
			dist = maxf(reach_min, dist * frac)
			continue

		var h2 = _cast_local(space, xform, elbow, wrist)
		if h2 != null:
			# Forearm blocked: bring the hand back to (just shy of) the hit point.
			dist = maxf(reach_min, a.distance_to(h2))
			continue

		break # both bones clear — this reach is safe
	return dist


## Sweep a thin sphere (radius `arm_thickness`) between two skeleton-local points —
## a "fat raycast" that gives the arm real width, so it clears a collider by its
## thickness instead of only where a 1-D ray happens to land (no slipping a thin
## edge between rays). On a hit, return the contact position (skeleton-local) backed
## off toward the start by `surface_offset` so the hand/finger mesh stops short of
## the surface. Returns null when the segment is clear. Areas are ignored, so only
## solid colliders block the arm.
func _cast_local(space: PhysicsDirectSpaceState3D, xform: Transform3D, p0: Vector3, p1: Vector3) -> Variant:
	var from := xform * p0
	var to := xform * p1
	var motion := to - from
	var seg_len := motion.length()
	if seg_len < 1e-5:
		return null

	var shape := SphereShape3D.new()
	shape.radius = arm_thickness
	var params := PhysicsShapeQueryParameters3D.new()
	params.shape = shape
	params.transform = Transform3D(Basis(), from)
	params.motion = motion
	params.exclude = _exclude

	# cast_motion returns [safe, unsafe] fractions along `motion`. unsafe == 0 means
	# the sphere already overlaps at the start (e.g. the shoulder is flush on a wall);
	# treat that as "no usable clamp" so the arm doesn't collapse against itself.
	var res := space.cast_motion(params)
	if res.is_empty() or res[1] <= 0.0:
		return null
	var safe: float = res[0]
	if safe >= 1.0:
		return null  # segment is clear

	var hit_dist := seg_len * safe
	var back: float = minf(surface_offset, hit_dist)
	var hpos := from + (motion / seg_len) * (hit_dist - back)
	return xform.affine_inverse() * hpos


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
