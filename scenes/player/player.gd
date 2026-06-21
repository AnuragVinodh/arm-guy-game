extends CharacterBody3D
## Player — the "arm guy" torso (a box) with an arm mounted on each Z face.
## Left/Right arrows move along the X axis; Space jumps. Movement is kept
## deliberately minimal here — the arm behaviour lives in arms.gd.

@export var move_speed: float = 6.0
@export var jump_velocity: float = 8.0

# Pull gravity from the project so the player matches the rest of the world.
var gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity", 9.8)


func _physics_process(delta: float) -> void:
	# Apply gravity while airborne.
	if not is_on_floor():
		velocity.y -= gravity * delta

	# Jump — Space maps to "ui_accept" by default.
	if Input.is_action_just_pressed("ui_accept") and is_on_floor():
		velocity.y = jump_velocity

	# Left / Right arrows ("ui_left" / "ui_right") drive the X axis only.
	var dir := Input.get_axis("ui_left", "ui_right")
	velocity.x = dir * move_speed

	# No Z movement for now — the guy lives in a 2.5D plane.
	velocity.z = 0.0

	move_and_slide()
