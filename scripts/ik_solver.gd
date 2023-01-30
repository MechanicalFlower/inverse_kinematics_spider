# translated from unity ik: https://www.youtube.com/watch?v=qqOAzn05fvk
# helpful links:
# - https://www.youtube.com/watch?v=e6Gjhr1IP6w
# - https://www.youtube.com/watch?v=qqOAzn05fvk https://www.youtube.com/watch?v=EEP4Vgcnjxs

tool
extends Skeleton

#export (NodePath) var root_joint_path setget _set_root_node
export var iteration_count = 10
export var target_position = Vector3.ZERO
export var pole_position = Vector3.ZERO
export var pole_rotation = Vector3.ZERO
var joints = []
var joint_lengths = 0
var positions = []
var max_length = 0


func set_joints(new_joints):  # there is ordering to joints. joints[0] is root
	joints = new_joints
	joint_lengths = []
	max_length = 0
	for i in range(1, joints.size()):
		joint_lengths.append(
			(joints[i].global_transform.origin - joints[i - 1].global_transform.origin).length()
		)
		max_length += joint_lengths[joint_lengths.size() - 1]
	positions = []
	for joint in joints:
		positions.append(joint.global_transform.origin)


func update_joint_transforms():
	if !joints or joints.size() == 0:
		return

	for i in range(0, joints.size()):
		positions[i] = joints[i].global_transform.origin

	if (
		joints[0].global_transform.origin.distance_squared_to(target_position)
		>= max_length * max_length
	):
		var direction = (target_position - positions[0]).normalized()
		for i in range(1, positions.size()):
			positions[i] = positions[i - 1] + direction * joint_lengths[i - 1]

	else:
		for i in range(iteration_count):
			# backward
			var epsilon = 0.01
			if (
				positions[positions.size() - 1].distance_squared_to(target_position)
				< epsilon * epsilon
			):
				break

			for j in range(positions.size() - 1, 0, -1):
				if j == positions.size() - 1:
					positions[j] = target_position
				else:
					positions[j] = (
						positions[j + 1]
						+ (positions[j] - positions[j + 1]).normalized() * joint_lengths[j]
					)

			# forward
			for j in range(1, positions.size()):
				positions[j] = (
					positions[j - 1]
					+ (positions[j] - positions[j - 1]).normalized() * joint_lengths[j - 1]
				)

	for i in range(1, positions.size() - 1):  # we can only move the middle joints (towards pole)
		var normal = (positions[i + 1] - positions[i - 1]).normalized()
		# put a plane on i-1 so we can check how much middle
		# node must be rotated to reach pole projection on the plane
		var plane = Plane(normal, positions[i - 1].dot(normal))
		var projected_pole = plane.project(pole_position)
		var projected_joint = plane.project(positions[i])
		var angle = (projected_joint - positions[i - 1]).angle_to(projected_pole - positions[i - 1])
		if (
			(projected_joint - positions[i - 1]).cross(projected_pole - positions[i - 1]).dot(
				plane.normal
			)
			< 0
		):  # need signed angle
			angle = -angle
		positions[i] = (
			Quat(normal, angle).normalized() * (positions[i] - positions[i - 1])
			+ positions[i - 1]
		)

	for i in range(0, joints.size()):
		joints[i].global_transform.origin = positions[i]
		var next_position = positions[i + 1] if i < joints.size() - 1 else target_position
		var up = pole_rotation - positions[i]  # keep up directed at pole
		joints[i].look_at(next_position, up)
