tool
extends Spatial

export(NodePath) var rest_position_path
export(NodePath) var ray_cast_position_path
export(NodePath) var pole_position_path
export(NodePath) var pole_rotation_path
export(NodePath) var root_joint_path setget _set_root_joint_path

export var step_interval_ms = 400.0
export var step_clock_offset_ms = 200.0
export var step_duration_ms = 100.0
export var step_height = 0.5
export var step_prediction_ratio = 1.0
export var step_min_distance = 0.3

# use squared value to avoid root calculations
var step_min_distance_squared = step_min_distance * step_min_distance
var previous_step_time = 0
var target_position = Vector3.ZERO
var next_position = Vector3.ZERO
var previous_position = Vector3.ZERO
var previous_tick = 0

# debug
onready var target_debug = _make_mesh(Color(0, 1, 0))
onready var next_position_debug = _make_mesh(Color(0, 0, 1))
onready var previous_position_debug = _make_mesh(Color(1, 0, 0))

onready var inverse_kinematic_solver = get_node("%inverse_kinematic_solver")


func _make_mesh(color):
	var mesh_instance = MeshInstance.new()
	add_child(mesh_instance)
	mesh_instance.mesh = SphereMesh.new()
	mesh_instance.global_scale(Vector3(0.1, 0.1, 0.1))
	var material = SpatialMaterial.new()
	material.albedo_color = color
	mesh_instance.material_override = material
	return mesh_instance


func _set_root_joint_path(node_path):
	root_joint_path = node_path
	if node_path and previous_step_time != 0:
		var joints = _extract_joints(get_node(root_joint_path))
		inverse_kinematic_solver.set_joints(joints)


func _ready():
	var joints = _extract_joints(get_node(root_joint_path))
	inverse_kinematic_solver.set_joints(joints)


func _extract_joints(root_joint_node):
	var current_node = root_joint_node
	var new_joints = []
	while current_node != null:
		var prev_node = current_node
		current_node = null
		for child in prev_node.get_children():
			if child is BoneAttachment:
				current_node = child
				new_joints.append(child)
	print("found ", new_joints.size(), " joints")
	return new_joints


func _process(_delta):
	var rest_position = get_node(rest_position_path).global_transform.origin
	var ray_cast_position = get_node(ray_cast_position_path).global_transform.origin
	var current_time = OS.get_ticks_msec()
	# adding offset helps this instance to get its next tick earlier than instances without offset
	var tick = floor((current_time + step_clock_offset_ms) / step_interval_ms)
	# update cast positions
#	if current_time - previous_step_time >= step_interval_ms:
	if tick - previous_tick > 0:
		previous_step_time = current_time
		previous_tick = tick
		var cast_start = ray_cast_position
		var cast_end = rest_position
		# use delta to rest position instead of previous end position, otherwise predictions are too late
		var last_step_delta = rest_position - previous_position
		cast_end += step_prediction_ratio * last_step_delta
		# ensure we cast to under the surface, this shouldnt be too large
		# because it effects movement_prediction (triangle to cast_end)
		cast_end += Vector3.DOWN * 2
		var cast_result = get_world().direct_space_state.intersect_ray(cast_start, cast_end)
		previous_position = next_position
		next_position = cast_result.position if cast_result else rest_position
	# interpolate target to new position
	if previous_position.distance_squared_to(next_position) > step_min_distance_squared:
		var transition_ratio = min(1, (current_time - previous_step_time) / step_duration_ms)
		target_position = lerp(previous_position, next_position, transition_ratio)
		target_position.y += step_height - abs(lerp(-step_height, step_height, transition_ratio))
#	else: # small jittering feels unnatural, better not move at all
#		$target.global_transform.origin = $current_position.global_transform.origin
	# solve ik position to target
	inverse_kinematic_solver.iteration_count = 1
	inverse_kinematic_solver.target_position = target_position
	inverse_kinematic_solver.pole_position = get_node(pole_position_path).global_transform.origin
	inverse_kinematic_solver.pole_rotation = get_node(pole_rotation_path).global_transform.origin
	inverse_kinematic_solver.update_joint_transforms()
	if Engine.is_editor_hint():
		target_debug.global_transform.origin = target_position
		next_position_debug.global_transform.origin = next_position
		previous_position_debug.global_transform.origin = previous_position
