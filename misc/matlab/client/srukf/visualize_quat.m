function visualize_quat(quat, varargin)
[x,y,z] = sphere;h = surf(x,y,z);axis('square');
xlabel('x'); ylabel('y'); zlabel('z');
axang = quat2AxisAngle(quat);
if length(varargin) > 0
    axang_ref = quat2AxisAngle(varargin{1});
else
    axang_ref = 0;
end
daxang = axang - axang_ref;
rotate(h,[1,0,0],daxang(1)*180/pi)
rotate(h,[0,1,0],daxang(2)*180/pi)
rotate(h,[0,0,1],daxang(3)*180/pi+90)