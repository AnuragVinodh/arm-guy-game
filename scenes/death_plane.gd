extends Area3D
## A respawn trigger sitting at the bottom of the pit. Anything that falls this
## far (the player missing the monkey bars, or sliding off an edge) is sent back
## to the level's start instead of dropping forever.

## Where a fallen body is placed again. Defaults to the player's spawn in the
## tutorial level; override per-scene if the start moves.
@export var respawn_position: Vector3 = Vector3(0.0, 2.0, 0.0)


func _ready() -> void:
	body_entered.connect(_on_body_entered)


func _on_body_entered(body: Node3D) -> void:
	# Only the player torso (a RigidBody3D) respawns; static geometry that happens
	# to overlap this volume is ignored.
	if not (body is RigidBody3D):
		return
	var rb := body as RigidBody3D
	rb.linear_velocity = Vector3.ZERO
	rb.angular_velocity = Vector3.ZERO
	rb.global_position = respawn_position
	rb.rotation = Vector3.ZERO
