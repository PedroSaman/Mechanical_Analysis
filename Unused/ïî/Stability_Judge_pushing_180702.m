%% Stability_Judg_pushing_180702.m
% �g�����̃u���b�N���f���̗͊w���
%%constant number
g = 9.81; T=70*g; M=2/15;
%% �u���b�N���f���̃f�[�^��ǂݍ���
% �t�@�C�����w�� **********************************************************
filename = 'fail_21r5.dat'
% �f�[�^�� model_original �ɏ�������
model_original = feval('load',filename);
%% �F����ǉ� (model)
% 22 => 1, 12 => 2, 21 => 3, 11 => 4
model = putcolor(model_original);
%% �g���r���̗͊w���
% [����2016]�̌��ؗp
% �u���b�N���f���Ō�̃u���b�N���������ނ��Ƃ�z��
SIZE = size(model);  N = SIZE(1);
%�������ݗ͂̂�����u���b�N�����߂�
b_push = pushing(model, N)    %�g�ݗ��Ă�u��
%b_push = pushing_0(model, N)
%�Ō�̃u���b�N���폜
model = model(1:(N - 1), 1:5);
%% ���u���b�N�� N, ���� z_max, �ʕ���� knobs
N = N - 1
z_max = lines(model)
knobs = knob(model);
%% 1�i�ڂ��珇�Ƀu���b�N���f���ɓ����͂����߂�
F_nx = [ ];  F_ny = [ ];  %��s��
for n = 1 : z_max
   if(n == 1) % 1�i��
       %�ʕ� knob_i, �אڃu���b�N adjoin_i
       knob_i = knob_line(model, 1);
       adjoin_i = adjoin(knob_i);
       %���C�� Ff, �@���� Fn
       F_f = Ff_0_180629(model);
       F_nz = Fnz_0(model);
       if(adjoin_i == -1)
           F_nx = -1;  F_ny = -1;
           force_nx = 0;  force_ny = 0;
       else
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
   else % 2�i�ڈȏ�
       %�ʕ� knob_i, �אڃu���b�N adjoin_i, �ڑ��ʕ� join_i
       knob_i = knob_line(model, n);
       adjoin_i = adjoin(knob_i);
       join_i = join(knobs, n);
       %���C�� Ff, �@���� Fn
       Ff_i = Ff_180629(join_i);
       Fnz_i = Fnz(join_i);
       Fnx_i = Fnx(adjoin_i);
       Fny_i = Fny(adjoin_i);
       %�ǉ�
       F_f = [F_f; Ff_i];
       F_nz = [F_nz; Fnz_i];
       if(Fnx_i(1) ~= -1)
           if(force_nx == 0)
               F_nx = Fnx_i;
               force_nx = -1;
           else
               F_nx = [F_nx; Fnx_i];
           end
       end
       if(Fny_i(1) ~= -1)
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
force_number = size(F_f);  force_f = force_number(1)
force_number = size(F_nz);  force_nz = force_number(1)
if((force_nx == 0) && (force_ny == 0))
    F = [F_f; F_nz];
elseif(force_ny == 0)
    F = [F_f; F_nz; F_nx];
    force_number = size(F_nx);  force_nx = force_number(1)
elseif(force_nx == 0)
    F = [F_f; F_nz; F_ny];
    force_number = size(F_ny);  force_ny = force_number(1)
else
    F = [F_f; F_nz; F_nx; F_ny];
    force_number = size(F_nx);  force_nx = force_number(1)
    force_number = size(F_ny);  force_ny = force_number(1)
end
force = force_f + force_nx + force_ny + force_nz;
%% Lower bounds
lb = -Inf(3*force+1, 1);  % �͂̐� �~ 3 (x, y, z) + �L���p�V�e�B
lb(3:3:3*force_f, 1) = 0;  %Ff(z) > 0
lb(3*force_f+1:3*force, 1) = 0;  %Fn > 0
%% Upper bounds
ub = Inf(3*force+1, 1);  % �͂̐� �~ 3 (x, y, z) + �L���p�V�e�B
ub( (3*force_f + 1):3:(3*(force_f+force_nz) - 2), 1) = 0;  %Ffz(x) = 0
ub( (3*force_f + 2):3:(3*(force_f+force_nz) - 1), 1) = 0;  %Ffz(y) = 0
ub( (3*(force_f+force_nz) + 2):3:(3*(force_f+force_nz+force_nx) - 1), 1) = 0;  %Ffx(y) = 0
ub( (3*(force_f+force_nz) + 3):3:(3*(force_f+force_nz+force_nx)), 1) = 0;  %Ffx(z) = 0
ub( (3*(force_f+force_nz+force_nx) + 1):3:(3*force - 2), 1) = 0;  %Ffy(x) = 0
ub( (3*(force_f+force_nz+force_nx) + 3):3:(3*force), 1) = 0;  %Ffy(z) = 0
%% Linear inequalities
%Cmax��]������͓̂ʂ̐�
SIZE = size(knobs);  n_knob = SIZE(1);
A = zeros(n_knob, 3*force+1);
b  = ones(n_knob, 1) * T;
count = 1;  check = 0;
for n = 1 : force_f;
    if(check ~= 0)

    else
        block_1 = F(n, 2);
        block_2 = F(n, 7);
        n_block = 1;
        for m = 1 : 11
            if((F(n+m, 2)==block_1) && (F(n+m, 7)==block_2))
                n_block = n_block + 1;
            else
                break;
            end
        end
        if(n_block == 8) %�� 1
            A(count, 3*n : 3 : 3*n+21) = 1;
            check = 8;  count = count + 1;
        elseif(n_block == 6) %�� 1
            A(count, 3*n : 3 : 3*n+15) = 1;
            check = 6;  count = count + 1;
        elseif(n_block == 12) %�� 2
            A(count, 3*n : 3 : 3*n+15) = 1;
            A(count+1, 3*n+18 : 3 : 3*n+33) = 1;
            check = 12;  count = count + 2;
%        elseif(n_block == 18) %�� 3
%            A(count, 3*n : 3 : 3*n+15) = 1;
%            A(count+1, 3*n+18 : 3 : 3*n+33) = 1;
%            A(count+2, 3*n+36 : 3 : 3*n+51) = 1;
%            check = 18;  count = count + 3;
        elseif(n_block == 24) %�� 4
            A(count, 3*n : 3 : 3*n+15) = 1;
            A(count+1, 3*n+18 : 3 : 3*n+33) = 1;
            A(count+2, 3*n+36 : 3 : 3*n+51) = 1;
            A(count+3, 3*n+54 : 3 : 3*n+69) = 1;
            check = 24;  count = count + 4;
        end
    end
    check = check - 1;
end
A( 1 : n_knob, 3*force+1) = -1;
%% Linear equalities
Aeq = zeros(6*N, 3*force+1);
beq = zeros(6*N, 1);
for n = 1 : N
    K_i = zeros(3, 3*force+1);
    p_i = zeros(force, 3);
    PN_i = eye(3*force + 1);
    for m = 1 : force
       if(F(m, 2) == n)
           K_i(1:3, (3*m - 2):3*m) = eye(3);
           p_i(m, 1 : 3) = F(m, 3 : 5);
           P_i = zeros(3, 3*force + 1);
           for l = 3 : 3 : 3*force
               P_i(1:3, (l-2):l) = [0, -p_i(l/3, 3), p_i(l/3, 2); p_i(l/3, 3), 0, -p_i(l/3, 1); -p_i(l/3, 2), p_i(l/3, 1), 0];
           end
           if(F(m, 6) == -1)
               PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = -eye(3);
           end
       elseif(F(m, 7) == n)
           K_i(1:3, (3*m - 2):3*m) = eye(3);
           p_i(m, 1 : 3) = F(m, 8 : 10);
           P_i = zeros(3, 3*force + 1);
           for l = 3 : 3 : 3*force
               P_i(1:3, (l-2):l) = [0, -p_i(l/3, 3), p_i(l/3, 2); p_i(l/3, 3), 0, -p_i(l/3, 1); -p_i(l/3, 2), p_i(l/3, 1), 0];
           end
           if(F(m, 11) == -1)
               PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = -eye(3);
           end
       end
    end
    Aeq((6*n - 5) : 6*n, 1 : (3*force + 1)) = [K_i; P_i] * PN_i;
    % type�ɂ���Ď��ʂ��ς��
    if(model(n, 4) == 22)
        beq((6*n - 5) : 6*n, 1) = [0; 0; M*g; 0; 0; 0];
    elseif((model(n, 4) == 12) || (model(n, 4) == 21))
        beq((6*n - 5) : 6*n, 1) = [0; 0; M*g/2; 0; 0; 0];
    elseif(model(n, 4) == 11)
        beq((6*n - 5) : 6*n, 1) = [0; 0; M*g/4; 0; 0; 0];
    end
end
%BBB =[beq(1:6:559), beq(2:6:560), beq(3:6:561), beq(4:6:562), beq(5:6:563), beq(6:6:564)]
%% �������ݗ͂���у��[�����g�� beq �ɒǉ�
SIZE = size(b_push);
N_push = SIZE(1);
for n = 1 : N_push
    pbn = b_push(n, 1);
    b_push(n, 4) = b_push(n, 4) + beq(6*pbn-3);
    b_dummy = b_push(n, 2:7).';
    beq(6*pbn-5 : 6*pbn) = b_dummy;
end
beq
%% Solve proble
f = zeros(3*force+1, 1);
f((3*force + 1), 1) = 1;
[x,fval,exitflag,output] = linprog(f,A,b,Aeq,beq,lb,ub)
XX =[x(1:3:3*force-2), x(2:3:3*force-1), x(3:3:3*force)]