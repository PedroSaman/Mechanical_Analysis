function FP = forces_position(block_type,force_type)
%returns the position of a specified force type for a specified block type
%input: block_type: 1x1, 1x2, 2x1, 2x2
%       force_type: friction force, X axis normal force, Y axis normal force
%output: FP = NYS

pf1 = [-1.25,     0, 1.5];  pf2 = [    0, -1.25, 1.5];  pf3 = [    0,  1.25, 1.5];  pf4 = [ 1.25,     0, 1.5];
Pf11 = [pf1; pf2; pf3; pf4]; %1x1 blocks friction forces position
Pf_11 = [Pf11(1:4, 1:2), -Pf11(1:4, 3)]; %Same as above but negative z axis

FP = [Pf11,Pf_11]