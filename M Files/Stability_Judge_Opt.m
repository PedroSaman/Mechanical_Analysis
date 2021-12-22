%% Stability_Judge.m

clear
clc

tic;

%% constant numbers
g = 9.81;
T=70*g;  %Maximum static friction force of one set of convex part
M=2/15; %Mass of a 2x2 block

%% Load the block model data
filename = '../Dat Files/fail_30.dat' %Specify the data file name
model_original = load(filename); % model_original = (x, y, z, type)
model = putcolor(model_original); % model = (BlockNo., x, y, z, type, color)

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
lb(3:3:3*force_f) = 0;  %The lower bound for the z axis component of the F_f force is 0
lb(3*force_f+1:3*force) = 0; %The lower bound of the F_n force is 0 in all 3 axis

%% Upper bounds (Ff, Fn)
ub = Inf(3*force+1, 1); %Number of forces x 3 (x, y, z) + Capacity
ub(3*force_f + 1:3:3*(force_f+force_nz) - 2) = 0; %The Fnz force component in x is 0
ub(3*force_f + 2:3:3*(force_f+force_nz) - 1) = 0; %The Fnz force component in y is 0
ub(3*(force_f+force_nz) + 2:3:3*(force_f+force_nz+force_nx) - 1) = 0; %The Fnx force component in y is 0
ub(3*(force_f+force_nz) + 3:3:3*(force_f+force_nz+force_nx)) = 0; %The Fnx force component in z is 0
ub(3*(force_f+force_nz+force_nx) + 1:3:3*force - 2) = 0; %The Fny force component in x is 0
ub(3*(force_f+force_nz+force_nx) + 3:3:3*force) = 0; %The Fny force component in z is 0

%%Fn_line Lower and Upper bounds 
check = 0;
for i = 1 : force_f
   if((F_f(i, 7) == -10)||(F_f(i, 7) == -11))
       % x < 0, y = 0
       lb(3*i-1) =  0;   ub(3*i-2 : 3*i-1) = [0, 0];
       check = 1;
   elseif((F_f(i, 7) == -20)||(F_f(i, 7) == -21))
       % x = 0, y < 0
       lb(3*i-2) =  0;   ub(3*i-2 : 3*i-1) = [0, 0];
       check = 1;
   elseif((F_f(i, 7) == 20)||(F_f(i, 7) == 21))
       % x = 0, y > 0
       lb(3*i-2 : 3*i-1) = [0, 0];   ub(3*i-2) =  0;
       check = 1;
   elseif((F_f(i, 7) == 10)||(F_f(i, 7) == 11))
       % x > 0, y = 0
       lb(3*i-2 : 3*i-1) = [0, 0];   ub(3*i-1) =  0;
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

%%Fn_line upper limit setting (Only the inner Fn_line from the 2x2 blocks)
for i = 1 : force_f
   if(((F(i, 4) == -0.75)||(F(i, 4) == 0.75))&&((F(i, 5) == -2)||(F(i, 5) == 2)))
      if(lb(3*i-2) ~= 0)
        lb(3*i-2) = -1*T;
      elseif(lb(3*i-1) ~= 0 )
        lb(3*i-1) = -1*T;
      elseif( ub(3*i-1) ~= 0 )
        ub(3*i-1) = 1*T;
      elseif( ub(3*i-2) ~= 0 )
        ub(3*i-2) = 1*T;
      end
   end
end

%% Linear inequalities
% Capacity Ci is evaluated by the number of connecting knobs
n_knob = size(knobs,1);
A = zeros(n_knob, 3*force+1); %each line represents one knob, end each set of 3 collums is one force.
b  = ones(n_knob, 1) * T;
count = 1;  check = 0;
max_m = 39; %Maximum number of friction forces that can appear in a block -1 (currently 2x4 block with 39)
n = 1;
while(n <= force_f) %for each friction force fill the A matrix
    block_1 = F(n, 2);
    block_2 = F(n, 8);
    n_block = 1;

    if(F(n, 8) == 0) %z = 1
        for m = 1:max_m %Count every force in the current block
            if(m >= force_f)   ,break;   end
            if((F(n+m, 2)~=block_1))
                break;
            else
                n_block = n_block + 1;
            end
        end
        
        if(n_block==8) %1x1 block
            A(count+1,3*n:3:3*(n+7)) = 1;
            count = count + 1;
        elseif(n_block==12) %1x2 or 2x1 block
            A(count+1,3*n:3:3*(n+2))     = 1; A(count+1,3*(n+6):3:3*(n+8))  = 1;
            A(count+2,3*(n+3):3:3*(n+5)) = 1; A(count+2,3*(n+9):3:3*(n+11)) = 1;
            count = count + 2;
        elseif(n_block==24) %2x2 block
            A(count+1,3*n:3:3*(n+2))      = 1; A(count+1,3*(n+12):3:3*(n+14)) = 1;
            A(count+2,3*(n+3):3:3*(n+5))  = 1; A(count+2,3*(n+15):3:3*(n+17)) = 1;
            A(count+3,3*(n+6):3:3*(n+8))  = 1; A(count+3,3*(n+18):3:3*(n+20)) = 1;
            A(count+4,3*(n+9):3:3*(n+11)) = 1; A(count+4,3*(n+21):3:3*(n+23)) = 1;
            count = count + 4;
        end
    else %z ~= 1
        for m = 1:max_m %Count every force in the current block
            if(m >= force_f)   ,break;   end
            if((F(n+m, 2)~=block_1))
                break;
            else
                n_block = n_block + 1;
            end
        end
        if(n_block == 8) %upper block is 1x1
            A(count+1,3*n:3:3*(n+7)) = 1;
        elseif(n_block == 6) %upper block is not a 1x1
            A(count+1,3*n:3:3*(n+5)) = 1;
        end
        count = count + 1;
    end
    n = n + n_block;
end
A(1:n_knob, 3*force+1) = -1; %Only the last collum of the A matrix

%% Linear equalities
Aeq = zeros(6*N, 3*force+1);
beq = zeros(6*N, 1);
for n = 1 : N   %for each block in the model
    K_i = zeros(3, 3*force+1); %This is the upper portion of the "Wj" matrix from the formulation
    p_i = zeros(force, 3); %This is all the "pk" vectors from the formulation into one matrix
    PN_i = eye(3*force + 1); %This is the "A" matrix from the formulation. (A = diag(A1,A2,...,An))
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
    if(model(n, 5) == 22)
        beq((6*n - 5) : 6*n, 1) = [0; 0; M*g; 0; 0; 0];
    elseif((model(n, 5) == 12) || (model(n, 5) == 21))
        beq((6*n - 5) : 6*n, 1) = [0; 0; M*g/2; 0; 0; 0];
    elseif(model(n, 5) == 11)
        beq((6*n - 5) : 6*n, 1) = [0; 0; M*g/4; 0; 0; 0];
    end
end
%% Solve problem
f = zeros(3*force+1, 1);
f((3*force + 1), 1) = 1;
[x,fval,exitflag,output] = linprog(f,A,b,Aeq,beq,lb,ub)  %Linear programming problem
XX =[x(1:3:3*force-2), x(2:3:3*force-1), x(3:3:3*force)]  %Force acting on the block model 
CM = -x(3*force+1) %Capacity CM 
time = toc