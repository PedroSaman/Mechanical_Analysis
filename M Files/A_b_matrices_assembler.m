function [A,b] = A_b_matrices_assembler(F,force_f,model,T)
%Assembly the A and b matrices from the nonlinear part of the
%linear programing problem (A.x <= b).
%input: F: The force vector (all forces acting in the model).
%       force_f: Number of friction forces
%       model: Input model
%       T: Maximum friction force possible for one knob.
%output: A: number_of_knobs by 3*number of forces (x,y,z) matrix.
%        b: number_of_knobs by 1 matrix.

knobs = knob(model,0);
n_knob = size(knobs,1);
b  = ones(n_knob, 1) * T; % Each knob can handle T amount of friction force
force = size(F,1);
A = zeros(n_knob, 3*force+1); % Each line represents one knob, end each set of 3 columns is one force.

for i = 1:force_f
    current_force_block = F(i,2);
    current_force_knob = F(i,3);
    knob_number = current_force_knob;
    x = 1;
    while(current_force_block - x >= 1) % Discover in which knob the force i is acting
        [col,row] = col_row_converter(model(current_force_block - x,5));
        knob_number = knob_number + row*col;
        x = x + 1;
    end
    A(knob_number,3*i) = 1; % Register this force in the correct knob
end
A(1:n_knob, 3*force+1) = 1; % Needed for the capacity calculation