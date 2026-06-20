extends RigidBody2D

## The arm guy. A torso driven entirely by two arms — no legs, no jumps.
## Left mouse controls & grabs left arm. Right mouse controls & grabs right arm.

@onready var left_shoulder: Marker2D = $LeftShoulder
@onready var right_shoulder: Marker2D = $RightShoulder
@onready var left_arm: Arm = $LeftShoulder/LeftArm
@onready var right_arm: Arm = $RightShoulder/RightArm
@onready var camera: Camera2D = $Camera2D

## How quickly the camera lerps to follow the torso
@export var camera_smoothing: float = 4.0
## Extra upward look-ahead so you can see where you're climbing
@export var camera_look_ahead: float = 120.0

var _mouse_world: Vector2 = Vector2.ZERO


func _ready() -> void:
	linear_damp = 0.5
	angular_damp = 4.0


func _input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT:
			if event.pressed:
				left_arm.try_grab()
			else:
				left_arm.release()

		elif event.button_index == MOUSE_BUTTON_RIGHT:
			if event.pressed:
				right_arm.try_grab()
			else:
				right_arm.release()

	if event.is_action_pressed("restart"):
		get_tree().reload_current_scene()


func _process(delta: float) -> void:
	_mouse_world = get_global_mouse_position()

	# Both arms track the mouse — left arm follows left-of-torso biased target,
	# right arm follows right-of-torso biased target.
	# Simple split: left arm chases mouse when it's left of torso center, right when right.
	var torso_x: float = global_position.x
	if _mouse_world.x <= torso_x:
		left_arm.target_world = _mouse_world
		right_arm.target_world = right_shoulder.global_position + Vector2(40, 20)
	else:
		right_arm.target_world = _mouse_world
		left_arm.target_world = left_shoulder.global_position + Vector2(-40, 20)

	_update_camera(delta)


func _update_camera(delta: float) -> void:
	var target_pos := global_position + Vector2(0, -camera_look_ahead)
	camera.global_position = camera.global_position.lerp(target_pos, camera_smoothing * delta)
