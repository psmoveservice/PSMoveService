function new_axisAngle = rotateAxisAngle(axisAngle, quat)
q = axisAngle2Quat(axisAngle);
rconj = quaternion_conjugate(quat);
new_axisAngle = quat2AxisAngle(quaternion_multiply(quat, quaternion_multiply(q, rconj)));