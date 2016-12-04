function output = sub_axang(axang1, axang2)
q1 = axisAngle2Quat(axang1);
q2 = axisAngle2Quat(axang2);
q2conj = quaternion_conjugate(q2);
deltaq = quaternion_multiply(q1, q2conj);
qnorm = sqrt(sum(deltaq.^2));
qout = bsxfun(@rdivide, deltaq, qnorm);
output = quat2AxisAngle(qout);