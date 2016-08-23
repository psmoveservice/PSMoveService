function new_state = process_function(old_states, process_noise, dt)
new_state = nan(size(old_states));
%posx, velx, accx, posy, vely, accy, posz, velz, accz, angx, avelx, angy, avely, angz, avelz

%TODO: Check how srukf handles process_noise

% Cartesian updates
pos = old_states([1, 4, 7], :);
lvel = old_states([2, 5, 8], :);
lacc = old_states([3, 6, 9], :);

new_state([1, 4, 7], :) = pos + dt*lvel + 0.5 * dt * dt * lacc;
new_state([2, 5, 8], :) = lvel + dt*lacc;
new_state([3, 6, 9], :) = lacc;

% Orientation updates
axang = old_states([10, 12, 14], :);
avel = old_states([11, 13, 15], :);
new_state([10, 12, 14], :) = add_axang(axang, dt*avel);
new_state([11, 13, 15], :) = avel;

if ~isempty(process_noise)
    %TODO: Handle process_noise having a different dimensionality than
    %state vector.
    isaxang = ismember(1:15, [10 12 14]);
	new_state(~isaxang, :) = new_state(~isaxang, :) + process_noise(~isaxang, :);
    new_state(isaxang, :) = add_axang(new_state(isaxang, :), process_noise(isaxang, :));
    %TODO: How does one add noise to an axisAngle?
end