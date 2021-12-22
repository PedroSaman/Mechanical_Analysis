function Ff = Ff_0_180705(model)
%%constant number
% 1*1
pf1 = [-1.25,     0, 1.5];  pf2 = [    0, -1.25, 1.5];  pf3 = [    0,  1.25, 1.5];  pf4 = [ 1.25,     0, 1.5];
Pf11 = [pf1; pf2; pf3; pf4];  Pf_11 = [Pf11(1:4, 1:2), -Pf11(1:4, 3)];
%1*2
pf1 = [-1.25,    -2, 1.5];  pf2 = [    0, -3.25, 1.5];  pf3 = [    0, -0.75, 1.5];  pf4 = [ 1.25,    -2, 1.5];
Pf12_1 = [pf1; pf2; pf3; pf4];  Pf_12_1 = [Pf12_1(1:4, 1:2), -Pf12_1(1:4, 3)];
pf1 = [-1.25,     2, 1.5];  pf2 = [    0,  0.75, 1.5];  pf3 = [    0,  3.25, 1.5];  pf4 = [ 1.25,     2, 1.5];
Pf12_2 = [pf1; pf2; pf3; pf4];  Pf_12_2 = [Pf12_2(1:4, 1:2), -Pf12_2(1:4, 3)];
Pf12_0 = [Pf12_1(1:2, 1:3); Pf12_1(4, 1:3); Pf12_2(1, 1:3); Pf12_2(3:4,1:3)];  Pf_12_0 = [Pf12_0(1:6, 1:2), -Pf12_0(1:6, 3)];
%2*1
pf1 = [-3.25,     0, 1.5];  pf2 = [   -2, -1.25, 1.5];  pf3 = [   -2,  1.25, 1.5];  pf4 = [-0.75,     0, 1.5];
Pf21_1 = [pf1; pf2; pf3; pf4];  Pf_21_1 = [Pf21_1(1:4, 1:2), -Pf21_1(1:4, 3)];
pf1 = [0.75,     0, 1.5];  pf2 = [   2, -1.25, 1.5];  pf3 = [   2,  1.25, 1.5];  pf4 = [3.25,     0, 1.5];
Pf21_2 = [pf1; pf2; pf3; pf4];  Pf_21_2 = [Pf21_2(1:4, 1:2), -Pf21_2(1:4, 3)];
Pf21_0 = [Pf21_1(1:3, 1:3); Pf21_2(2:4, 1:3)];  Pf_21_0 = [Pf21_0(1:6, 1:2), -Pf21_0(1:6, 3)];
%2*2
pf1 = [-3.25,    -2, 1.5];  pf2 = [   -2, -3.25, 1.5];  pf3 = [   -2, -0.75, 1.5];  pf4 = [-0.75,    -2, 1.5];
Pf22_1 = [pf1; pf2; pf3; pf4];  Pf_22_1 = [Pf22_1(1:4, 1:2), -Pf22_1(1:4, 3)];
pf1 = [-3.25,    2, 1.5];  pf2 = [   -2, 0.75, 1.5];  pf3 = [   -2, 3.25, 1.5];  pf4 = [-0.75,    2, 1.5];
Pf22_2 = [pf1; pf2; pf3; pf4];  Pf_22_2 = [Pf22_2(1:4, 1:2), -Pf22_2(1:4, 3)];
pf1 = [0.75,    -2, 1.5];  pf2 = [   2, -3.25, 1.5];  pf3 = [   2, -0.75, 1.5];  pf4 = [3.25,    -2, 1.5];
Pf22_3 = [pf1; pf2; pf3; pf4];  Pf_22_3 = [Pf22_3(1:4, 1:2), -Pf22_3(1:4, 3)];
pf1 = [0.75,    2, 1.5];  pf2 = [   2, 0.75, 1.5];  pf3 = [    2 3.25, 1.5];  pf4 = [3.25,    2, 1.5];
Pf22_4 = [pf1; pf2; pf3; pf4];  Pf_22_4 = [Pf22_4(1:4, 1:2), -Pf22_4(1:4, 3)];
Pf22_0 = [Pf22_1(1:2, 1:3); Pf22_1(4, 1:3); Pf22_2(1, 1:3); Pf22_2(3:4, 1:3); Pf22_3(1:2, 1:3); Pf22_3(4, 1:3); Pf22_4(1, 1:3); Pf22_4(3:4, 1:3)];
Pf_22_0 = [Pf22_0(1:12, 1:2), -Pf22_0(1:12, 3)];
%% Ff = (Force_number, Block_number1, x1, y1, z1, -1, 0, 0, 0, 0, 0)
loop = size(model);  count = 0;
for n = 1 : loop;
   if(model(n, 3) == 1)
       count = count + 1;  %1段目ブロックの数をカウント
   else
       break;
   end
end
force = 1;
F = zeros(count*12, 11);
F(1:(count*24), 1) = 1: (count*24);  %Force Number
F(1:(count*24), 6) = -1;  %力の向きは負
for n = 1 : count;
   if(model(n, 4) == 11)
       F(force:(force+7), 2) = n;  %Block Number 1
       F(force:(force+3), 3:5) = Pf_11;  %座標(z = -1.5)
       F((force+4):(force+7), 3:5) = Pf_11;  %座標(z = -0.5)
       F((force+4):(force+7), 5) = -0.5 * ones(4, 1);  %座標(z = -0.5)
       F((force+4):(force+7), 6) = [10; 20; -20; -10]; %%垂直抗力の上下限値設定用
       force = force + 8;
   elseif(model(n, 4) == 12)
       F(force:(force+11), 2) = n;  %Block Number 1
       F(force:(force+5), 3:5) = Pf_12_0;  %座標(z = -1.5)
       F((force+6):(force+11), 3:5) = Pf_12_0;  %座標(z = -0.5)
       F((force+6):(force+11), 5) = -0.5 * ones(6, 1);  %座標(z = -0.5)
       F((force+6):(force+11), 6) = [10; 20; -10; 10; -20; -10]; %%垂直抗力の上下限値設定用
       force = force + 12;
   elseif(model(n, 4) == 21)
       F(force:(force+11), 2) = n;  %Block Number 1
       F(force:(force+5), 3:5) = Pf_21_0;  %座標(z = -1.5)
       F((force+6):(force+11), 3:5) = Pf_21_0;  %座標(z = -0.5)
       F((force+6):(force+11), 5) = -0.5 * ones(6, 1);  %座標(z = -0.5)
       F((force+6):(force+11), 6) = [10; 20; -20; 20; -20; -10]; %%垂直抗力の上下限値設定用
       force = force + 12;
   elseif(model(n, 4) == 22)
       F(force:(force+23), 2) = n;  %Block Number 1
       F(force:(force+11), 3:5) = Pf_22_0;  %座標(z = -1.5)
       F((force+12):(force+23), 3:5) = Pf_22_0;  %座標(z = -0.5)
       F((force+12):(force+23), 5) = -0.5 * ones(12, 1);  %座標(z = -0.5)
       F((force+12):(force+23), 6) = [10; 20; -10; 10; -20; -10; 10; 20; -10; 10; -20; -10]; %%垂直抗力の上下限値設定用
       force = force + 24;
   end
end
force = force - 1;
Ff = F(1:force, 1:11);