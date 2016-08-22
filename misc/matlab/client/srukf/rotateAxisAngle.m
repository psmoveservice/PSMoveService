function new_axisAngle = rotateAxisAngle(axisAngle, quat)
new_axisAngle = quat2AxisAngle(quaternion_multiply(quat, quaternion_multiply(axisAngle2Quat(axisAngle), quaternion_conjugate(quat))));