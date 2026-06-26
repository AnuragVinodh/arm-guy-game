extends Area3D

@onready var label: Label = $"../HUD/ThankYou"

func _on_body_entered(body: Node3D) -> void:
	if body.name == "Player":
		label.visible = true
		await get_tree().create_timer(3.0).timeout
		label.visible = false
