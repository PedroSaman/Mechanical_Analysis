%% Stability_Judge_180906.m
% ブロックモデルの力学解析(静止時)  % Fn' 上限値設定
tic; %時間計測スタート
%% constant number
g = 9.81; %重力加速度
T=70*g;  %1組の凸部を接続するのに必要な力(最大静止摩擦力)
M=2/15; %2*2ブロックの質量
%% ブロックモデルのデータを読み込み
%   → model_original = (x, y, z, type)
filename = 'test2.dat'
model_original = feval('load',filename);
%% 色情報を追加 % 22 => 1, 12 => 2, 21 => 3, 11 => 4
%   → model = (BlockNumber, x, y, z, type, color)
model = putcolor(model_original);
%% 色情報付きのデータを入力
% model = feval('load',filename);
% model = putnumber(model);
%% 総ブロック数 N, 高さ z_max, 凸部情報 knobs
SIZE = size(model);  N = SIZE(1);
z_max = lines(model);
knobs = knob(model);
%% 1段目から順にブロックモデルに働く力を求める
F_nx = [ ];  F_ny = [ ];  %空行列
for n = 1 : z_max
   if(n == 1) % 1段目
       %凸部 knob_i, 隣接ブロック adjoin_i
       knob_i = knob_line(model, 1);
       adjoin_i = adjoin(knob_i);
       %摩擦力 Ff, 法線力 Fn
       F_f = Ff_0_180723(model);
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
   else % 2段目以上
       %凸部 knob_i, 隣接ブロック adjoin_i, 接続凸部 join_i
       knob_i = knob_line(model, n);
       adjoin_i = adjoin(knob_i);
       join_i = join(knobs, n);
       %摩擦力 Ff, 法線力 Fn
       Ff_i = Ff_180723(join_i);
       Fnz_i = Fnz(join_i);
       Fnx_i = Fnx(adjoin_i);
       Fny_i = Fny(adjoin_i);
       %追加
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
%% 力の数を求める
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
%% Lower bounds
lb = -Inf(3*force+1, 1);  % 力の数 × 3 (x, y, z) + キャパシティ
lb(3:3:3*force_f, 1) = 0;  %Ff(z) > 0
lb(3*force_f+1:3*force, 1) = 0;  %Fn > 0
%% Upper bounds
ub = Inf(3*force+1, 1);  % 力の数 × 3 (x, y, z) + キャパシティ
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
   if(check == 1) %Fn' の向き(+-)は高さで決まる
      if(F_f(i,8) == 0) %下段が土台
          F(i, 7) = -1;
      else
          F(i, 7:6:13) = [-1, 1];
      end
   end
   check = 0;
end
LB = [lb(1:3:3*force_f-2), lb(2:3:3*force_f-1), lb(3:3:3*force_f)];
UB = [ub(1:3:3*force_f-2), ub(2:3:3*force_f-1), ub(3:3:3*force_f)];
F;
%% Linear inequalities
%Cmaxを評価するのは凸の数
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
        if(n_block == 8) %凸 1
            A(count, 3*n : 3 : 3*n+21) = 1;
            check = 8;  count = count + 1;
        elseif(n_block == 6) %凸 1
            A(count, 3*n : 3 : 3*n+15) = 1;
            check = 6;  count = count + 1;
        elseif(n_block == 12) %凸 2
            if(F(n, 8) == 0) % z = 1
                A(count, 3*n : 3 : 3*(n+2)) = 1;        A(count, 3*(n+6) : 3 : 3*(n+8)) = 1;
                A(count+1, 3*(n+3) : 3 : 3*(n+5)) = 1;  A(count+1, 3*(n+9) : 3 : 3*(n+11)) = 1;
            else
                 A(count, 3*n : 3 : 3*n+15) = 1;
                 A(count+1, 3*n+18 : 3 : 3*n+33) = 1;
            end
            check = 12;  count = count + 2;
        elseif(n_block == 24) %凸 4
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
for n = 1 : N   % n番目ブロックについて調べる
    K_i = zeros(3, 3*force+1);
    p_i = zeros(force, 3);
    PN_i = eye(3*force + 1);
    for m = 1 : force   % n番目ブロックに働く力があればつり合い式に追加
       if(F(m, 2) == n)
           K_i(1:3, (3*m - 2):3*m) = eye(3);   % 力のつり合い
           p_i(m, 1 : 3) = F(m, 4:6);   % 始点座標
           P_i = zeros(3, 3*force + 1);
           for l = 3 : 3 : 3*force   % モーメントのつり合い
               P_i(1:3, (l-2):l) = [0, -p_i(l/3, 3), p_i(l/3, 2); p_i(l/3, 3), 0, -p_i(l/3, 1); -p_i(l/3, 2), p_i(l/3, 1), 0];
           end
           if(F(m, 7) == -1)   %向きを修正
               PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = -eye(3);
           end
       elseif(F(m, 8) == n)
           K_i(1:3, (3*m - 2):3*m) = eye(3);   % 力のつり合い
           p_i(m, 1 : 3) = F(m, 10:12);   % 始点座標
           P_i = zeros(3, 3*force + 1);
           for l = 3 : 3 : 3*force   % モーメントのつり合い
               P_i(1:3, (l-2):l) = [0, -p_i(l/3, 3), p_i(l/3, 2); p_i(l/3, 3), 0, -p_i(l/3, 1); -p_i(l/3, 2), p_i(l/3, 1), 0];
           end
           if(F(m, 13) == -1)   %向きを修正
               PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = -eye(3);
           end
       end
    end
    Aeq((6*n - 5) : 6*n, 1 : (3*force + 1)) = [K_i; P_i] * PN_i;
    % typeによって質量が変わる
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
[x,fval,exitflag,output] = linprog(f,A,b,Aeq,beq,lb,ub)
XX =[x(1:3:3*force-2), x(2:3:3*force-1), x(3:3:3*force)]
CM = x(3*force+1);
time = toc;