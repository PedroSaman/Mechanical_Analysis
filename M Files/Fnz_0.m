function Fnz = Fnz_0(model)
%Find the normal forces position in the z axis of blocks that are in the first layer of the block model in regard to its own coordinate axis.
%input: model:(Block_Number, x, y, z, type, color)
%output: Fnz:(Force_Number, Block_Number, 0, x, y, z, 1, 0, 0, 0, 0, 0, 0)

% 1x1 Block
pn1 = [-2, -2, 1.5];  pn2 = [-2,  2, 1.5];  pn3 = [ 2, -2, 1.5];  pn4 = [ 2,  2, 1.5];
Pnz11 = [pn1; pn2; pn3; pn4];  Pnz_11 = [Pnz11(1:4, 1:2), -Pnz11(1:4, 3)];

% 1x2 Block
pn1 = [-2, -4, 1.5];  pn2 = [-2,  0, 1.5];  pn3 = [ 2, -4, 1.5];  pn4 = [ 2,  0, 1.5];
Pnz12_1 = [pn1; pn2; pn3; pn4];  Pnz_12_1 = [Pnz12_1(1:4, 1:2), -Pnz12_1(1:4, 3)];
pn1 = [-2,  0, 1.5];  pn2 = [-2,  4, 1.5];  pn3 = [ 2,  0, 1.5];  pn4 = [ 2,  4, 1.5];
Pnz12_2 = [pn1; pn2; pn3; pn4];  Pnz_12_2 = [Pnz12_2(1:4, 1:2), -Pnz12_2(1:4, 3)];
Pnz12_0 = [Pnz12_1(1, 1:3); Pnz12_2(2, 1:3); Pnz12_1(3, 1:3); Pnz12_2(4, 1:3)];
Pnz_12_0 = [Pnz12_0(1:4, 1:2), -Pnz12_0(1:4, 3)];

% 2x1 Block
pn1 = [-4, -2, 1.5];  pn2 = [-4,  2, 1.5];  pn3 = [ 0, -2, 1.5];  pn4 = [ 0,  2, 1.5];
Pnz21_1 = [pn1; pn2; pn3; pn4];  Pnz_21_1 = [Pnz21_1(1:4, 1:2), -Pnz21_1(1:4, 3)];
pn1 = [0, -2, 1.5];  pn2 = [0,  2, 1.5];  pn3 = [4, -2, 1.5];  pn4 = [4,  2, 1.5];
Pnz21_2 = [pn1; pn2; pn3; pn4];  Pnz_21_2 = [Pnz21_2(1:4, 1:2), -Pnz21_2(1:4, 3)];
Pnz21_0 = [Pnz21_1(1:2, 1:3); Pnz21_2(3:4, 1:3)];
Pnz_21_0 = [Pnz21_0(1:4, 1:2), -Pnz21_0(1:4, 3)];

% 2x2 Block
pn1 = [-4, -4, 1.5];  pn2 = [-4,  0, 1.5];  pn3 = [ 0, -4, 1.5];  pn4 = [ 0,  0, 1.5];
Pnz22_1 = [pn1; pn2; pn3; pn4];  %Pnz_22_1 = [Pnz22_1(1:4, 1:2), -Pnz22_1(1:4, 3)];
pn1 = [-4, 0, 1.5];  pn2 = [-4, 4, 1.5];  pn3 = [ 0, 0, 1.5];  pn4 = [ 0, 4, 1.5]; 
Pnz22_2 = [pn1; pn2; pn3; pn4];  %Pnz_22_2 = [Pnz22_2(1:4, 1:2), -Pnz22_2(1:4, 3)];
pn1 = [0, -4, 1.5];  pn2 = [0,  0, 1.5];  pn3 = [4, -4, 1.5];  pn4 = [4,  0, 1.5];
Pnz22_3 = [pn1; pn2; pn3; pn4];  %Pnz_22_3 = [Pnz22_3(1:4, 1:2), -Pnz22_3(1:4, 3)];
pn1 = [0, 0, 1.5];  pn2 = [0, 4, 1.5];  pn3 = [4, 0, 1.5];  pn4 = [4, 4, 1.5];
Pnz22_4 = [pn1; pn2; pn3; pn4];  %Pnz_22_4 = [Pnz22_4(1:4, 1:2), -Pnz22_4(1:4, 3)];

Pnz22_0 = [Pnz22_1(1, 1:3); Pnz22_2(2, 1:3); Pnz22_3(3, 1:3); Pnz22_4(4, 1:3)]; % 2x2 block edges position
Pnz_22_0 = [Pnz22_0(1:4, 1:2), -Pnz22_0(1:4, 3)]; %Same as above but negative z axis

%Pnz22_12 = [Pnz22_1(1, 1:3); Pnz22_2(2, 1:3); Pnz22_1(3, 1:3); Pnz22_2(4, 1:3)];
%Pnz_22_12 = [Pnz22_12(1:4, 1:2), -Pnz22_12(1:4, 3)];
%Pnz22_34 = [Pnz22_3(1, 1:3); Pnz22_4(2, 1:3); Pnz22_3(3, 1:3); Pnz22_4(4, 1:3)];
%Pnz_22_34 = [Pnz22_34(1:4, 1:2), -Pnz22_34(1:4, 3)];
%Pnz22_13 = [Pnz22_1(1:2, 1:3); Pnz22_3(3:4, 1:3)];
%Pnz_22_13 = [Pnz22_13(1:4, 1:2), -Pnz22_13(1:4, 3)];
%Pnz22_24 = [Pnz22_2(1:2, 1:3); Pnz22_4(3:4, 1:3)];
%Pnz_22_24 = [Pnz22_24(1:4, 1:2), -Pnz22_24(1:4, 3)];

loop = size(model);  count = 0;
for n = 1 : loop;
   if(model(n, 4) == 1)
       count = count + 1; %Count the number of blocks in the first stage 
   else
       break;
   end
end

Fnz = zeros(count*4, 13);
Fnz(1:count*4, 1) = 1: count*4;  %Force Number
Fnz(1:count*4, 7) = 1;
force = 1;
for n = 1 : count %for each block in the 1st layer
    Fnz(force:(force+3), 2) = model(n, 1); %block number
   if(model(n, 5) == 11)
       Fnz(force:(force+3), 4:6) = Pnz_11;
   elseif(model(n, 5) == 12)
       Fnz(force:(force+3), 4:6) = Pnz_12_0;
   elseif(model(n, 5) == 21)
       Fnz(force:(force+3), 4:6) = Pnz_21_0;
   elseif(model(n, 5) == 22)
       Fnz(force:(force+3), 4:6) = Pnz_22_0;
   end
   force = force + 4; %every block have 4 normal forces in the z axis
end