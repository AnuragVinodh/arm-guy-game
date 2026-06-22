class_name Bar
extends StaticBody3D
## A grabbable cylindrical bar (monkey-bar / high-bar).
##
## When an arm grabs a normal surface the swing pivots on the hand itself. A Bar
## overrides that: the pivot becomes the CENTER of the bar's circular cross-section
## — the point on the bar's axis nearest the hand — instead of the off-center
## surface contact. The torso then orbits the true center, so it can rotate all the
## way around the bar smoothly without the arm jamming on a lopsided anchor.

## The bar's central axis, in LOCAL space. Godot's CylinderShape3D / CylinderMesh
## both run along local +Y, so +Y is the default. Only change this if the bar's
## collider is built along a different local axis.
@export var axis: Vector3 = Vector3.UP


## The point on the bar's central axis nearest `world_point` — i.e. the center of
## the circular cross-section the hand is grabbing. Used as the swing pivot.
func axis_center(world_point: Vector3) -> Vector3:
	var o := global_position
	var d := global_transform.basis * axis
	if d.length() < 1e-6:
		return o
	d = d.normalized()
	return o + d * (world_point - o).dot(d)
