%% Stability_Judg_pushing.m

clear; clc; tic;

%% constant numbers
g = 9.81;
T= 70*g;  %Maximum static friction force of one set of convex part
M= 2/60; %Mass of a 1x1 block

%% Load the block model data
filename = '../Dat Files/Pedro Functionality Tests/Dat_Tests/23 - Heavy4LayerBetterBridge.dat'; %Specify the data file name
model_original = load(filename); % model_original = (x, y, z, type)
model = putcolor(model_original); % model = (BlockNo., x, y, z, type, color)

%% Model and knobs information
N = size(model,1); %Total number of blocks
z_max = lines(model); % Structure height
knobs = knob(model,0); % Knobs information (entire block model)

% �������ݗ͂̉����u���b�N�̔ԍ��C�������܂�邱�ƂŃu���b�N�ɉ����͂ƃ��[�����g�𒲂ׂ�
%   �� b_push = (Block_No., Fx, Fy, Fz, Mx, My, Mz)
b_push = pushing(model, N);
%�}������u���b�N���u���b�N���f���f�[�^����폜
model = model(1:(N - 1), :);
%% ���u���b�N�� N, ���� z_max, �ʕ���� knobs
N = N - 1;
%% 1�i�ڂ��珇�Ƀu���b�N���f���ɓ����͂����߂�
F_nx = [ ];  F_ny = [ ];  %��s��
for n = 1 : z_max
   if(n == 1) % 1�i��
       %�ʕ� knob_i, �אڃu���b�N adjoin_i
       knob_i = knob_line(model, 1);
       adjoin_i = adjoin(knob_i)
       %���C�� Ff, �@���� Fn
       F_f = Ff_0_180723(model);
       F_nz = Fnz_0(model);
       if(adjoin_i == -1) % �אڂ���u���b�N�Ȃ�
           F_nx = -1;  F_ny = -1;
           force_nx = 0;  force_ny = 0;
       else % �אڂ���u���b�N����
           Fnx_i = Fnx(adjoin_i);
           Fny_i = Fny(adjoin_i);
           if(Fnx_i(1) ~= -1)
               F_nx = Fnx_i;
               force_nx = -1;
           else
               force_nx = 0;
           end
           if(Fny_i(1) ~= -1)
               F_ny = Fny_i;
               force_ny = -1;
           else
               force_ny = 0;
           end
       end
   else % n�i�� (n >= 2)
       %�ʕ� knob_i, �אڃu���b�N adjoin_i, �ڑ��ʕ� join_i
       knob_i = knob_line(model, n);
       adjoin_i = adjoin(knob_i);
       join_i = join(knobs, n);
       %���C�� Ff, �@���� Fn
       Ff_i = Ff_180723(join_i);
       Fnz_i = Fnz(join_i);
       Fnx_i = Fnx(adjoin_i);
       Fny_i = Fny(adjoin_i);
       %�ǉ�
       F_f = [F_f; Ff_i];
       F_nz = [F_nz; Fnz_i];
       if(Fnx_i(1) ~= -1) % �אڂ���u���b�N����(x������)
           if(force_nx == 0)
               F_nx = Fnx_i;
               force_nx = -1;
           else
               F_nx = [F_nx; Fnx_i];
           end
       end
       if(Fny_i(1) ~= -1) % �אڂ���u���b�N����(y������)
           if(force_ny == 0)
               F_ny = Fny_i;
               force_ny = -1;
           else
               F_ny = [F_ny; Fny_i];
           end
       end
   end
end
%% �͂̐������߂�
force_number = size(F_f);  force_f = force_number(1);
force_number = size(F_nz);  force_nz = force_number(1);
if((force_nx == 0) && (force_ny == 0))
    F = [F_f; F_nz];
elseif(force_ny == 0)
    F = [F_f; F_nz; F_nx];
    force_number = size(F_nx);  force_nx = force_number(1);
elseif(force_nx == 0)
    F = [F_f; F_nz; F_ny];
    force_number = size(F_ny);  force_ny = force_number(1);
else
    F = [F_f; F_nz; F_nx; F_ny];
    force_number = size(F_nx);  force_nx = force_number(1);
    force_number = size(F_ny);  force_ny = force_number(1);
end
force = force_f + force_nx + force_ny + force_nz;
%% Lower bounds (Ff, Fn)
lb = -Inf(3*force+1, 1);  % �͂̐� �~ 3 (x, y, z) + �L���p�V�e�B
lb(3:3:3*force_f, 1) = 0;  %Ff(z) > 0
lb(3*force_f+1:3*force, 1) = 0;  %Fn > 0
%% Upper bounds (Ff, Fn)
ub = Inf(3*force+1, 1);  % �͂̐� �~ 3 (x, y, z) + �L���p�V�e�B
ub( (3*force_f + 1):3:(3*(force_f+force_nz) - 2), 1) = 0;  %Fnz(x) = 0
ub( (3*force_f + 2):3:(3*(force_f+force_nz) - 1), 1) = 0;  %Fnz(y) = 0
ub( (3*(force_f+force_nz) + 2):3:(3*(force_f+force_nz+force_nx) - 1), 1) = 0;  %Fnx(y) = 0
ub( (3*(force_f+force_nz) + 3):3:(3*(force_f+force_nz+force_nx)), 1) = 0;  %Fnx(z) = 0
ub( (3*(force_f+force_nz+force_nx) + 1):3:(3*force - 2), 1) = 0;  %Fny(x) = 0
ub( (3*(force_f+force_nz+force_nx) + 3):3:(3*force), 1) = 0;  %Fny(z) = 0
%% Fn' Lower and Upper bounds
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
   if(check == 1) %Fn' �̌���(+-)�͍����Ō��܂�
      if(F_f(i,8) == 0) %���i���y��
          F(i, 7) = -1;
      else
          F(i, 7:6:13) = [-1, 1];
      end
   end
   check = 0;
end
%% Fn' ����l�ݒ� (2*2�̎d�؂�̂�)
for i = 1 : force_f
   if( ((F(i, 4) == -0.75)||(F(i, 4) == 0.75))&&((F(i, 5) == -2)||(F(i, 5) == 2)) )
      if( lb(3*i-2) ~= 0 )
          lb(3*i-2) = -1*T;
      elseif( lb(3*i-1) ~= 0 )
          lb(3*i-1) = -1*T;
      elseif( ub(3*i-1) ~= 0 )
          ub(3*i-1) = 1*T;
      elseif( ub(3*i-2) ~= 0 )
          ub(3*i-2) = 1*T;
      end
   end
end
% �㉺���l �m�F�p LB, UB
LB = [lb(1:3:3*force_f-2), lb(2:3:3*force_f-1), lb(3:3:3*force_f)];
UB = [ub(1:3:3*force_f-2), ub(2:3:3*force_f-1), ub(3:3:3*force_f)];
%% Linear inequalities
% �L���p�V�e�BCi��]������̂͐ڑ��ʕ��̐�
SIZE = size(knobs);  n_knob = SIZE(1);
A = zeros(n_knob, 3*force+1);
b  = ones(n_knob, 1) * T;
count = 1;  check = 0;
for n = 1 : force_f
    if(check ~= 0)

    else
        block_1 = F(n, 2);
        block_2 = F(n, 8);
        n_block = 1;
        for m = 1 : 23
            if(m >= force_f)   ,break;   end
            if((F(n+m, 2)~=block_1) || (F(n+m, 8)~=block_2))
                break;
            else
                n_block = n_block + 1;
            end
        end
        if(n_block == 8) %�� 1
            A(count, 3*n : 3 : 3*n+21) = 1;
            check = 8;  count = count + 1;
        elseif(n_block == 6) %�� 1
            A(count, 3*n : 3 : 3*n+15) = 1;
            check = 6;  count = count + 1;
        elseif(n_block == 12) %�� 2
            if(F(n, 8) == 0) % z = 1
                A(count, 3*n : 3 : 3*(n+2)) = 1;        A(count, 3*(n+6) : 3 : 3*(n+8)) = 1;
                A(count+1, 3*(n+3) : 3 : 3*(n+5)) = 1;  A(count+1, 3*(n+9) : 3 : 3*(n+11)) = 1;
            else
                 A(count, 3*n : 3 : 3*n+15) = 1;
                 A(count+1, 3*n+18 : 3 : 3*n+33) = 1;
            end
            check = 12;  count = count + 2;
        elseif(n_block == 24) %�� 4
            if(F(n, 8) == 0) % z = 1
                A(count, 3*n : 3 : 3*(n+2)) = 1;        A(count, 3*(n+12) : 3 : 3*(n+14)) = 1;
                A(count+1, 3*(n+3) : 3 : 3*(n+5)) = 1;  A(count+1, 3*(n+15) : 3 : 3*(n+17)) = 1;
                A(count+2, 3*(n+6) : 3 : 3*(n+8)) = 1;  A(count+2, 3*(n+18) : 3 : 3*(n+20)) = 1;
                A(count+3, 3*(n+9) : 3 : 3*(n+11)) = 1; A(count+3, 3*(n+21) : 3 : 3*(n+23)) = 1;
            else
                 A(count, 3*n : 3 : 3*n+15) = 1;
                 A(count+1, 3*n+18 : 3 : 3*n+33) = 1;
                 A(count+2, 3*n+36 : 3 : 3*n+51) = 1;
                 A(count+3, 3*n+54 : 3 : 3*n+69) = 1;
            end
            check = 24;  count = count + 4;
        end
    end
    check = check - 1;
end
A( 1 : n_knob, 3*force+1) = -1;
%% Linear equalities
Aeq = zeros(6*N, 3*force+1);
beq = zeros(6*N, 1);
for n = 1 : N   % n�Ԗڃu���b�N�ɂ��Ē��ׂ�
    K_i = zeros(3, 3*force+1);
    p_i = zeros(force, 3);
    PN_i = eye(3*force + 1);
    for m = 1 : force   % n�Ԗڃu���b�N�ɓ����͂�����΂荇�����ɒǉ�
       if(F(m, 2) == n)
           K_i(1:3, (3*m - 2):3*m) = eye(3);   % �͂̂荇��
           p_i(m, 1 : 3) = F(m, 4:6);   % �n�_���W
           P_i = zeros(3, 3*force + 1);
           for l = 3 : 3 : 3*force   % ���[�����g�̂荇��
               P_i(1:3, (l-2):l) = [0, -p_i(l/3, 3), p_i(l/3, 2); p_i(l/3, 3), 0, -p_i(l/3, 1); -p_i(l/3, 2), p_i(l/3, 1), 0];
           end
           if(F(m, 7) == -1)   %�͂̌������C��
               PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = -eye(3);
           end
       elseif(F(m, 8) == n)
           K_i(1:3, (3*m - 2):3*m) = eye(3);   % �͂̂荇��
           p_i(m, 1 : 3) = F(m, 10:12);   % �n�_���W
           P_i = zeros(3, 3*force + 1);
           for l = 3 : 3 : 3*force   % ���[�����g�̂荇��
               P_i(1:3, (l-2):l) = [0, -p_i(l/3, 3), p_i(l/3, 2); p_i(l/3, 3), 0, -p_i(l/3, 1); -p_i(l/3, 2), p_i(l/3, 1), 0];
           end
           if(F(m, 13) == -1)   %�͂̌������C��
               PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = -eye(3);
           end
       end
    end
    Aeq((6*n - 5) : 6*n, 1 : (3*force + 1)) = [K_i; P_i] * PN_i;
    % type�ɂ���Ď��ʂ��ς��
    if(model(n, 5) == 22)
        beq((6*n - 5) : 6*n, 1) = [0; 0; M*g; 0; 0; 0];
    elseif((model(n, 5) == 12) || (model(n, 5) == 21))
        beq((6*n - 5) : 6*n, 1) = [0; 0; M*g/2; 0; 0; 0];
    elseif(model(n, 5) == 11)
        beq((6*n - 5) : 6*n, 1) = [0; 0; M*g/4; 0; 0; 0];
    end
end
%% �������ݗ͂���у��[�����g�� beq �ɒǉ�
SIZE = size(b_push);
N_push = SIZE(1);   % �������ݗ͂̉����u���b�N�̐�
for n = 1 : N_push
    pbn = b_push(n, 1);   % �������ݗ͂̉����u���b�N�̔ԍ�
    b_push(n, 4) = b_push(n, 4) + beq(6*pbn-3);   % �������ݗ� + �d��
    b_dummy = b_push(n, 2:7).';
    beq(6*pbn-5 : 6*pbn) = b_dummy;
end
beq;
%% Solve problem
f = zeros(3*force+1, 1);
f((3*force + 1), 1) = 1;
[x,fval,exitflag,output] = linprog(f,A,b,Aeq,beq,lb,ub)  %���`�v����
XX =[x(1:3:3*force-2), x(2:3:3*force-1), x(3:3:3*force)]  %�u���b�N���f���ɓ�����
CM = x(3*force+1) %�L���p�V�e�BCM
time = toc  %�v�����ԕ\��