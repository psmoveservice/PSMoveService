function new_state = process_function(old_state, Q, dt)
new_state = nan(size(old_state));
%posx, velx, accx, posy, vely, accy, posz, velz, accz, angx, avelx, angy, avely, angz, avelz

% Cartesian updates
pos = old_state([1, 4, 7]);
lvel = old_state([2, 5, 8]);
lacc = old_state([3, 6, 9]);

new_state([1, 4, 7]) = pos + dt*lvel + 0.5 * dt * dt * lacc;
new_state([2, 5, 8]) = lvel + dt*lacc;
new_state([3, 6, 9]) = lacc;

% Orientation updates
axang = old_state([10, 12, 14]);
avel = old_state([11, 13, 15]);
new_state([10, 12, 14]) = add_axang(axang, dt*avel);
new_state([11, 13, 15]) = avel;

new_state = new_state + Q.mu;