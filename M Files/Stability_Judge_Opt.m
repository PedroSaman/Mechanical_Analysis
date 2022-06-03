%% Stability_Judge.m
% T - A*x will show how much force is overflowing (case negative) or still
% can be applied (case positive) for each knob in the given model.
%(need to understand and explain better what knob is which)

%% Initialization
clear; clc; tic;

%% constant numbers
g = 9.8;
T = 307*g; %Maximum static friction force of one set of convex part
M = [11,100;12,1.39/20;21,10000;13,17/175;31,17/175;14,1.03/8;41,1.03/8;22,8.1/64;24,3.9/16;42,3.9/16;28,11/24;82,11/24];
good_margin = T*0.8; %arbitrary minimum value for stability

%% Load the block model data and search for structural problems
filename = '../Dat Files/A_1.dat'; %Specify the data file name
fprintf('Filename: %s \n',filename);
model_original = load(filename); % model_original = (x, y, z, type)
model = putcolor(model_original); % model = (BlockNo., x, y, z, type, color)
check = model_check(model,M);
if(check == -1 ) %If returns 1, this model is not supported
    fprintf('This model has a block that is not currently available in the laboratory. \n');
    return;
elseif(check == -2)
    fprintf('This model has multiples blocks occupying the same spot. \n');
    return;
end

%% Model and knobs information
N = size(model,1); %Total number of blocks
z_max = lines(model); % Structure height
knobs = knob(model,0); % Knobs information (entire block model)

%% Find the forces acting on the model
F_nx = [ ];  F_ny = [ ];

%for the first layer
knob_i = knob(model, 1); %knobs in this layer information
adjoin_i = adjoin(knob_i); %adjoint blocks information
F_f = Ff_0_180723(model); %Friction force of blocks in the first layer
F_nz = Fnz_0(model); %Normal force in the z axis in the first layer

if(adjoin_i ~= -1) %There are adjacent blocks 
    Fnx_i = Fnx(adjoin_i); %Normal force in the x axis
    Fny_i = Fny(adjoin_i); %Normal force in the y axis
    if(Fnx_i(1) ~= -1) %If there is normal force in X axis
        F_nx = Fnx_i;
    end
    if(Fny_i(1) ~= -1) %If there is normal force in Y axis
        F_ny = Fny_i;
    end
end

for n = 2 : z_max %for layer = 2 to the end 
    knob_i = knob(model, n); %knobs in this layer information
    adjoin_i = adjoin(knob_i); %adjoint blocks information
    join_i = join(knobs, n); %knobs are snaped togheter

    Ff_i = Ff_180723(join_i); %Friction force of blocks in the second layer or higher
    Fnz_i = Fnz(join_i);   %Normal force in the z axis in the second layer or higher
    Fnx_i = Fnx(adjoin_i); %Normal force in the x axis in the second layer or higher
    Fny_i = Fny(adjoin_i); %Normal force in the y axis in the second layer or higher
    
    %Add the forces determined in the current iteration in the final forces matrix
    F_f = [F_f; Ff_i];
    F_nz = [F_nz; Fnz_i];
    if(Fnx_i(1) ~= -1) %If there is normal force in X axis
        F_nx = [F_nx; Fnx_i];
    end
    if(Fny_i(1) ~= -1) %If there is normal force in Y axis
        F_ny = [F_ny; Fny_i];
    end
end

%% Find the number of forces
force_f = size(F_f,1);
force_nz = size(F_nz,1);
force_nx = size(F_nx,1);
force_ny = size(F_ny,1);
F = [F_f; F_nz; F_nx; F_ny];
force = force_f + force_nz + force_nx + force_ny;

%% Lower bounds (Ff, Fn)
lb = -Inf(3*force+1, 1); %Number of forces x 3 (x, y, z) + Capacity
lb(3:3:3*force_f) = 0;  %F_f lower bound component in z is 0
lb(3*force_f+1:3*force) = 0; %lower bound of the F_n is 0 in all 3 axis

%% Upper bounds (Ff, Fn)
ub = Inf(3*force+1, 1); %Number of forces x 3 (x, y, z) + Capacity
ub(3*force_f + 1:3:3*(force_f+force_nz) - 2) = 0; %Fnz upper bound component in x is 0
ub(3*force_f + 2:3:3*(force_f+force_nz) - 1) = 0; %Fnz upper bound component in y is 0
ub(3*(force_f+force_nz) + 2:3:3*(force_f+force_nz+force_nx) - 1) = 0; %Fnx upper bound component in y is 0
ub(3*(force_f+force_nz) + 3:3:3*(force_f+force_nz+force_nx)) = 0; %Fnx upper bound component in z is 0
ub(3*(force_f+force_nz+force_nx) + 1:3:3*force - 2) = 0; %Fny upper bound component in x is 0
ub(3*(force_f+force_nz+force_nx) + 3:3:3*force) = 0; %Fny upper bound component in z is 0

%% Fn_line Lower and Upper bounds 
for i = 1 : force_f
    if((F_f(i, 7) == -1))
        %x points to -infinite, y = 0 
        lb(3*i-1) =  0;   ub(3*i-2 : 3*i-1) = [0, 0]; %no force component in Y axis and in X axis = [-inf,0]
    elseif((F_f(i, 7) == -2)) 
        %x = 0, y points to -infinite 
        lb(3*i-2) =  0;   ub(3*i-2 : 3*i-1) = [0, 0]; %no force component in X axis and in Y axis = [-inf,0]
    elseif((F_f(i, 7) == 2))
        %x = 0, y points to +infinite
        lb(3*i-2 : 3*i-1) = [0, 0];   ub(3*i-2) =  0; %no force component in X axis and in Y axis = [0,+inf]
    elseif((F_f(i, 7) == 1))
        %x points to +infinite, y = 0
        lb(3*i-2 : 3*i-1) = [0, 0];   ub(3*i-1) = 0; %no force component in Y axis and in X axis = [0,+inf]
    end
    if(F_f(i,8) == 0) %If this force is from a n==1 layer
        F(i, 7) = -1; %The bottom is the foundation
    else
        F(i, 7:6:13) = [-1, 1];
    end %The Fn_line orientation is determined by the height 
end

%% Fn_line upper limit setting (Only the inner Fn_line from the nx2/2xn blocks)
for i = 1 : force_f
    if(F(i, 4) == -0.75)
        lb(3*i-2) = -T;
    elseif(F(i, 4) == 0.75)
        ub(3*i-2) = T;
    elseif(F(i, 5) == -0.75)
        lb(3*i-1) = -T;
    elseif(F(i, 5) == 0.75)
        ub(3*i-1) = T;
    end
end

%% Linear programming problem
[A,b] = A_b_matrices_assembler(F,force_f,model,T); %Linear Inequalities
[Aeq,beq] = Aeq_beq_matrices_assembler(F,N,force,model,M); %Linear equalities
f = zeros(3*force+1, 1); %Evaluate function
f((3*force + 1), 1) = -1; %To minimize the evaluate function result
[x,fval,exitflag,output] = linprog(f,A,b,Aeq,beq,lb,ub);  %Solve the problem

if(~isempty(x))
    fprintf('Solution found\n');
    XX =[x(1:3:3*force-2), x(2:3:3*force-1), x(3:3:3*force)];  %Force acting on the block model 
    CM = x(3*force+1); %Capacity CM 
    if(CM >= good_margin)
        fprintf('Stability with good security margin. CM = %.4f \n',CM);
    else
        fprintf('Stability can not be guaranteed. CM = %.4f \n',CM);
    end
else
    fprintf('No feasible solution found \n');
end
fprintf('Time elapsed: %.2d \n',toc);