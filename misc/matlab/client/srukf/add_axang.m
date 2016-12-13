function output = add_axang(axang1, axang2)
q1 = axisAngle2Quat(axang1);
q2 = axisAngle2Quat(axang2);
qout = quaternion_multiply(q1, q2);
output = quat2AxisAngle(qout);