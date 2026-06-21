extends RigidBody3D
## Player — the "arm guy" torso. Pure physics: it has NO direct locomotion.
## The only way to move is through the arms (see arms.gd), which grab surfaces
## and drag/swing this body around. Gravity, damping and the 2.5D plane lock are
## configured on the node in player.tscn.

func _physics_process(_delta: float) -> void:
	# R restarts the level (rage-game staple).
	if Input.is_action_just_pressed("restart"):
		get_tree().reload_current_scene()
