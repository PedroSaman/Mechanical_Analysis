%% Stability_Judg_pushing.m

clear; clc; tic;

%% constant numbers
g = 9.81;
T = 70*g;  %Maximum static friction force of one set of convex part
M = 2/60; %Mass of a 1x1 block
good_margin = 590; %arbitrary minimum value for stability

%% Load the block model data
filename = '../Dat Files/A_1.dat'; %Specify the data file name
model_original = load(filename); % model_original = (x, y, z, type)
model = putcolor(model_original); % model = (BlockNo., x, y, z, type, color)
model_size = size(model_original,1);

%% Model and knobs information
N = size(model,1);
b_push = pushing(model, N); % Forces and Torques that the insertion of the last block causes
N = N - 1; % Ignore the last block in the usual stability judge. If it stay stable during the insertion, it will be in a static scenario.
model = model(1:N,:); %Narrow down the model. Exclude the last block.
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

for n = 2 : z_max %from layer number 2 and ahead 
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
check = 0;
for i = 1 : force_f
if((F_f(i, 7) == -1))
    %x points to -infinite, y = 0 
    lb(3*i-1) =  0;   ub(3*i-2 : 3*i-1) = [0, 0]; %no force component in Y axis and in X axis = [-inf,0]
    check = 1;
elseif((F_f(i, 7) == -2)) 
    %x = 0, y points to -infinite 
    lb(3*i-2) =  0;   ub(3*i-2 : 3*i-1) = [0, 0]; %no force component in X axis and in Y axis = [-inf,0]
    check = 1;
elseif((F_f(i, 7) == 2))
    %x = 0, y points to +infinite
    lb(3*i-2 : 3*i-1) = [0, 0];   ub(3*i-2) =  0; %no force component in X axis and in Y axis = [0,+inf]
    check = 1;
elseif((F_f(i, 7) == 1))
    %x points to +infinite, y = 0
    lb(3*i-2 : 3*i-1) = [0, 0];   ub(3*i-1) =  0; %no force component in Y axis and in X axis = [0,+inf]
    check = 1;
end
if(check == 1) %The Fn_line orientation is determined by the height 
    if(F_f(i,8) == 0) %If this force is from a n==1 layer
        F(i, 7) = -1; %The bottom is the foundation
    else
        F(i, 7:6:13) = [-1, 1];
    end
end
check = 0;
end

%% Fn_line upper limit setting (Only the inner Fn_line from the 2x2 blocks)
for i = 1 : force_f
    if(F(i, 4) == -0.75) %lb(3*i-2)
        lb(3*i-2) = -T;
    elseif(F(i, 4) == 0.75) %ub(3*i-2)
        ub(3*i-2) = T;
    elseif(F(i, 5) == -0.75) %lb(3*i-1)
        lb(3*i-1) = -T;
    elseif(F(i, 5) == 0.75) %ub(3*i-1)
        ub(3*i-1) = T;
    end
end

%% Linear inequalities
% Capacity Ci is evaluated by the number of connecting knobs
n_knob = size(knobs,1);
A = zeros(n_knob, 3*force+1); %each line represents one knob, end each set of 3 columns is one force.
b  = ones(n_knob, 1) * T;
knob_counter = 1;
n = 1;
while(n <= force_f) %for each friction force fill the A matrix
    force_counter = 0;    
    type_1 = model(F(n,2),5); %current upper block type    
    [col,row] = col_row_converter(type_1);
    if(F(n, 8) == 0) %z = 1
        if(type_1 == 11) %special case for 1x1 block
            A(knob_counter,3*n:3:3*(n+7)) = 1; %Add 4 forces to A and duplicate it
            knob_counter = knob_counter + 1;   %go to the next knob
            force_counter = force_counter + 8; %iterate to the next force
        else
            starting_knob = knob_counter; %need to store which knonb were the first one to later duplicate the A matrix
            if(row>=col) %if the block is taller than wide or square
                for i = 1:col %for each column
                    for j = 1:row %for each row
                        if(j==1 || j==row) %if this knob is from the first or last row
                            A(knob_counter,3*(n+force_counter):3:3*((n+force_counter)+2)) = 1; %Add 3 forces to A
                            force_counter = force_counter + 3; %iterate to the next force
                        else %if this knob is from a middle row
                            A(knob_counter,3*(n+force_counter):3:3*((n+force_counter)+1)) = 1; %Add 2 forces to A
                            force_counter = force_counter + 2; %iterate to the next force
                        end
                        knob_counter = knob_counter + 1; %go to the next knob
                    end
                end
            else %if the block is wider than tall
                for j = 1:col %for each column
                    for i = 1:row %for each row
                        if(j==1 || j==col) %if this is the first or the last column
                            A(knob_counter,3*(n+force_counter):3:3*((n+force_counter)+2)) = 1; %Add 3 forces to A
                            force_counter = force_counter + 3; %iterate to the next force
                        else %if this is a middle column
                            A(knob_counter,3*(n+force_counter):3:3*((n+force_counter)+1)) = 1; %Add 2 forces to A
                            force_counter = force_counter + 2; %iterate to the next force
                        end
                        knob_counter = knob_counter + 1; %iterate to the next knob
                    end
                end
            end
            %Need to double the A matrix as there are pairs of friction
            %forces
            A(starting_knob:(knob_counter-1),3*(n+force_counter):3:3*(n+(force_counter)*2-1)) = A(starting_knob:knob_counter-1,3*n:3:3*(n+force_counter-1));
            force_counter = force_counter*2;
        end
    else %z > 1 Here there is no need to duplicate the force aftwards 
        if(type_1 == 11) %special case for 1x1 block
            A(knob_counter,3*n:3:3*(n+7)) = 1; %Add 4 forces to A and duplicate it
            knob_counter = knob_counter + 1;   %go to the next knob
            force_counter = force_counter + 8; %iterate to the next force
        elseif(row>=col) %if the block is taller than wide
            while(F(n+force_counter,2) == F(n,2)) %while in the same block
                m = F(n+force_counter,3); %store the block number
                if(rem(m,row) == 1 || rem(m,row) == 0) %if this is the first or the last row
                    A(knob_counter,3*(n+force_counter):3:3*((n+force_counter)+5)) = 1; %Add 6 forces to A
                    force_counter = force_counter + 6; %iterate to the next force
                else %if this is a middle row
                    A(knob_counter,3*(n+force_counter):3:3*((n+force_counter)+3)) = 1; %Add 4 forces to A
                    force_counter = force_counter + 4; %iterate to the next force
                end
                knob_counter = knob_counter + 1; %iterate to the next knob
            end
        else
            while(F(n+force_counter,2) == F(n,2)) %while in the same block
                m = F(n+force_counter,3); %store the block number
                if(m == 1 || m == 2 || m == col*row-1 || m == col*row) %if this is the first or the last row
                    A(knob_counter,3*(n+force_counter):3:3*((n+force_counter)+5)) = 1; %Add 6 forces to A
                    force_counter = force_counter + 6; %iterate to the next force
                else %if this is a middle row
                    A(knob_counter,3*(n+force_counter):3:3*((n+force_counter)+3)) = 1; %Add 4 forces to A
                    force_counter = force_counter + 4; %iterate to the next force
                end
                knob_counter = knob_counter + 1; %iterate to the next knob
            end
        end
    end

    n = n + force_counter; %update n value
end
A(1:n_knob, 3*force+1) = -1; %Only the last column of the A matrix

%% Linear equalities
Aeq = zeros(6*N, 3*force+1);
beq = zeros(6*N, 1);
for n = 1 : N   %for each block in the model
    K_i = zeros(3, 3*force+1); %This is the upper portion of the "Wj" matrix from the formulation
    p_i = zeros(force, 3); %This is all the "pk" vectors from the formulation into one matrix
    PN_i = eye(3*force + 1); %This is the "A" matrix from the formulation. (A = diag(A1,A2,...,An))
                            %Here the Zero matrix inside A, case the kth force does not appear in 
                            %the block is not in A but in Wj.
    for m = 1 : force 
        if(F(m, 2) == n) %if the current force first block is the current one
            K_i(1:3, (3*m - 2):3*m) = eye(3); %Force Balance 
            p_i(m, 1 : 3) = F(m, 4:6); %Start point coordinates 
            if(F(m, 7) == -1) %Correct the direction of force 
                PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = -eye(3);
            end
            elseif(F(m, 8) == n) %if the current force second block is the current one
            K_i(1:3, (3*m - 2):3*m) = eye(3); %Force Balance
            p_i(m, 1 : 3) = F(m, 10:12); %Start point coordinates
            if(F(m, 13) == -1) %Correct the direction of force
                PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = -eye(3);
            end
        end
    end
    P_i = zeros(3, 3*force + 1); %This is the botton portion of the "Wj" matrix from the formulation 
    for l = 3 : 3 : 3*force
        P_i(1:3, (l-2):l) = [0, -p_i(l/3, 3), p_i(l/3, 2); p_i(l/3, 3), 0, -p_i(l/3, 1); -p_i(l/3, 2), p_i(l/3, 1), 0];
    end
    Aeq((6*n - 5) : 6*n, 1 : (3*force + 1)) = [K_i; P_i] * PN_i; %[W1*A1;W2*A2,...,Wn*An]
    %Mass changes depending on block type [b1;b2;...;bn]
    [col,row] = col_row_converter(model(n, 5));
    beq((6*n - 5) : 6*n, 1) = [0; 0; col*row*M*g; 0; 0; 0]; %[(0,0,M1*g,0,0,0);...;(0,0,Mn*g,0,0,0)]
end

%% Add pushing force and moment to beq 
N_push = size(b_push,1); %Number of blocks to which pushing force is applied
for n = 1 : N_push
    pbn = b_push(n, 1); %The number of the block to which the pushing force is applied 
    b_push(n, 4) = b_push(n, 4) + beq(6*pbn-3); %Pushing force in Z (Fz) + gravity 
    b_dummy = b_push(n, 2:7).';%'
    beq(6*pbn-5 : 6*pbn) = b_dummy;
end

%% Solve problem
f = zeros(3*force+1, 1);
f((3*force + 1), 1) = 1;
options = optimoptions('linprog','Display','none');
[x,fval,exitflag,output] = linprog(f,A,b,Aeq,beq,lb,ub,options);  %Linear programming problem
if(~isempty(x))
    XX =[x(1:3:3*force-2), x(2:3:3*force-1), x(3:3:3*force)];  %Force acting on the block model 
    CM = -x(3*force+1); %Capacity CM 
    fprintf('CM value is %.4f \n',CM);
else
    fprintf('No feasible solution found. This model is not feasible\n');
end

%% Final messages
if(CM > good_margin)
    fprintf('This model is Stable within the good CM value.\n');
else
    fprintf('This model is Unstable!\n');
end
fprintf('Time elapsed: %.5f \n',toc);