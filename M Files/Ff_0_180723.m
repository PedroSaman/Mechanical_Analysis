function Ff = Ff_0_180723(model)
%Find the friction force position in blocks that are in the first layer of the block model in regard to its own coordinate axis.
%input: model:(Block_Number, x, y, z, type, color)
%output: Ff = (Force_Number, Block_Number, Knob_Number, x, y, z, *, 0, 0, 0, 0, 0, 0)
%             *: upper and lower limits of the normal force
% Pfnm_xy = position force of a block with n columns and m rows touching from the xth to the yth knob. Special case to 0 which represents
% that it is touching the entire block. a "_" after the Pf, mean that this position is the negative one.

Pf = force_position(1,1,"ff");
Pf11_0 = Pf(1:4,:);
Pf_11_0 = Pf(5:8,:);

%1x1 Block
%pf1 = [-1.25,     0, 1.5];  pf2 = [    0, -1.25, 1.5];  pf3 = [    0,  1.25, 1.5];  pf4 = [ 1.25,     0, 1.5];
%Pf11_0 = [pf1; pf2; pf3; pf4]; %1x1 blocks friction forces position
%Pf_11_0 = [Pf11_0(1:4, 1:2), -Pf11_0(1:4, 3)]; %Same as above but negative z axis

Pf = force_position(1,2,"ff");
Pf12_1 = Pf(1:4,:);
Pf12_2 = Pf(5:8,:);
Pf12_0 = [Pf12_1(1:2, 1:3); Pf12_1(4, 1:3); Pf12_2(1, 1:3); Pf12_2(3:4,1:3)]; %1x2 blocks friction forces position
Pf_12_0 = [Pf12_0(1:6, 1:2), -Pf12_0(1:6, 3)]; %Same as above but negative z axis

%1x2 Block
%pf1 = [-1.25,    -2, 1.5];  pf2 = [    0, -3.25, 1.5];  pf3 = [    0, -0.75, 1.5];  pf4 = [ 1.25,    -2, 1.5];
%Pf12_1 = [pf1; pf2; pf3; pf4];  %Pf_12_1 = [Pf12_1(1:4, 1:2), -Pf12_1(1:4, 3)];
%pf1 = [-1.25,     2, 1.5];  pf2 = [    0,  0.75, 1.5];  pf3 = [    0,  3.25, 1.5];  pf4 = [ 1.25,     2, 1.5];
%Pf12_2 = [pf1; pf2; pf3; pf4];  %Pf_12_2 = [Pf12_2(1:4, 1:2), -Pf12_2(1:4, 3)];
%Pf12_0 = [Pf12_1(1:2, 1:3); Pf12_1(4, 1:3); Pf12_2(1, 1:3); Pf12_2(3:4,1:3)]; %1x2 blocks friction forces position
%Pf_12_0 = [Pf12_0(1:6, 1:2), -Pf12_0(1:6, 3)]; %Same as above but negative z axis

Pf = force_position(2,1,"ff");
Pf21_1 = Pf(1:4,:);
Pf21_2 = Pf(5:8,:);
Pf21_0 = [Pf21_1(1:3, 1:3); Pf21_2(2:4, 1:3)]; %2x1 blocks friction forces position
Pf_21_0 = [Pf21_0(1:6, 1:2), -Pf21_0(1:6, 3)]; %Same as above but negative z axis

%2x1 Block
%pf1 = [-3.25,     0, 1.5];  pf2 = [   -2, -1.25, 1.5];  pf3 = [   -2,  1.25, 1.5];  pf4 = [-0.75,     0, 1.5];
%Pf21_1 = [pf1; pf2; pf3; pf4];  %Pf_21_1 = [Pf21_1(1:4, 1:2), -Pf21_1(1:4, 3)];
%pf1 = [0.75,     0, 1.5];  pf2 = [   2, -1.25, 1.5];  pf3 = [   2,  1.25, 1.5];  pf4 = [3.25,     0, 1.5];
%Pf21_2 = [pf1; pf2; pf3; pf4];  %Pf_21_2 = [Pf21_2(1:4, 1:2), -Pf21_2(1:4, 3)];
%Pf21_0 = [Pf21_1(1:3, 1:3); Pf21_2(2:4, 1:3)]; %2x1 blocks friction forces position
%Pf_21_0 = [Pf21_0(1:6, 1:2), -Pf21_0(1:6, 3)]; %Same as above but negative z axis

Pf = force_position(2,2,"ff");
Pf22_1 = Pf(1:4,:);
Pf22_2 = Pf(5:8,:);
Pf22_3 = Pf(9:12,:);
Pf22_4 = Pf(13:16,:);
Pf22_0 = [Pf22_1(1:2, 1:3); Pf22_1(4, 1:3); Pf22_2(1, 1:3); Pf22_2(3:4, 1:3); Pf22_3(1:2, 1:3); Pf22_3(4, 1:3); Pf22_4(1, 1:3); Pf22_4(3:4, 1:3)]; %2x2 blocks friction forces position
Pf_22_0 = [Pf22_0(1:12, 1:2), -Pf22_0(1:12, 3)]; %Same as above but negative z axis

%2x2 Block
%pf1 = [-3.25,    -2, 1.5];  pf2 = [   -2, -3.25, 1.5];  pf3 = [   -2, -0.75, 1.5];  pf4 = [-0.75,    -2, 1.5]; 
%Pf22_1 = [pf1; pf2; pf3; pf4];  %Pf_22_1 = [Pf22_1(1:4, 1:2), -Pf22_1(1:4, 3)];
%pf1 = [-3.25,    2, 1.5];  pf2 = [   -2, 0.75, 1.5];  pf3 = [   -2, 3.25, 1.5];  pf4 = [-0.75,    2, 1.5];
%Pf22_2 = [pf1; pf2; pf3; pf4];  %Pf_22_2 = [Pf22_2(1:4, 1:2), -Pf22_2(1:4, 3)];
%pf1 = [0.75,    -2, 1.5];  pf2 = [   2, -3.25, 1.5];  pf3 = [   2, -0.75, 1.5];  pf4 = [3.25,    -2, 1.5];
%Pf22_3 = [pf1; pf2; pf3; pf4];  %Pf_22_3 = [Pf22_3(1:4, 1:2), -Pf22_3(1:4, 3)];
%pf1 = [0.75,    2, 1.5];  pf2 = [   2, 0.75, 1.5];  pf3 = [    2, 3.25, 1.5];  pf4 = [3.25,    2, 1.5];
%Pf22_4 = [pf1; pf2; pf3; pf4];  %Pf_22_4 = [Pf22_4(1:4, 1:2), -Pf22_4(1:4, 3)];
%2x2 blocks friction forces position
%Pf22_0 = [Pf22_1(1:2, 1:3); Pf22_1(4, 1:3); Pf22_2(1, 1:3); Pf22_2(3:4, 1:3); Pf22_3(1:2, 1:3); Pf22_3(4, 1:3); Pf22_4(1, 1:3); Pf22_4(3:4, 1:3)]; 
%Pf_22_0 = [Pf22_0(1:12, 1:2), -Pf22_0(1:12, 3)]; %Same as above but negative z axis

loop = size(model);
count = 0;
for n = 1 : loop
   if(model(n, 4) == 1)
       count = count + 1; %Count the number of blocks in the first stage 
   else
       break;
   end
end

force = 1;
F = zeros(count*24, 13);
F(1:(count*24), 1) = 1: (count*24);  %Force Number
for n = 1 : count %for each block in the 1st layer
   if(model(n, 5) == 11) % 1x1 Block
       F(force:(force+7), 2) = model(n, 1);  %Block Number
       F(force:(force+7), 3) = ones(8, 1);  %Knob Number
       F(force:(force+3), 4:6) = Pf_11_0;  %Cordinate z = -1.5
       F((force+4):(force+7), 4:6) = Pf_11_0;  
       F((force+4):(force+7), 6) = -0.5 * ones(4, 1);  %Cordinate z = -0.5
       F(force:(force+7), 7) = [10; 20; -20; -10; 10; 20; -20; -10]; %setting the upper and lower limits of the normal force
       force = force + 8;
   elseif(model(n, 5) == 12) % 1x2 Block
       F(force:(force+11), 2) = model(n, 1);  %Block Number
       F(force:(force+5), 3) = [1; 1; 1; 2; 2; 2]; % Knob Number
       F((force+6):(force+11), 3) = [1; 1; 1; 2; 2; 2]; % Knob Number
       F(force:(force+5), 4:6) = Pf_12_0;  %Cordinate z = -1.5
       F((force+6):(force+11), 4:6) = Pf_12_0;  
       F((force+6):(force+11), 6) = -0.5 * ones(6, 1);  %Cordinate z = -0.5
       F(force:(force+11), 7) = [10; 20; -10; 10; -20; -10; 10; 20; -10; 10; -20; -10]; %setting the upper and lower limits of the normal force
       force = force + 12;
   elseif(model(n, 5) == 21) % 2x1 Block
       F(force:(force+11), 2) = model(n, 1);  %Block Number
       F(force:(force+5), 3) = [1; 1; 1; 2; 2; 2]; % Knob Number
       F((force+6):(force+11), 3) = [1; 1; 1; 2; 2; 2]; % Knob Number
       F(force:(force+5), 4:6) = Pf_21_0;  %Cordinate z = -1.5
       F((force+6):(force+11), 4:6) = Pf_21_0;  
       F((force+6):(force+11), 6) = -0.5 * ones(6, 1);  %Cordinate z = -0.5
       F(force:(force+11), 7) = [10; 20; -20; 20; -20; -10; 10; 20; -20; 20; -20; -10]; %setting the upper and lower limits of the normal force
       force = force + 12;
   elseif(model(n, 5) == 22) % 2x2 Block
       F(force:(force+23), 2) = model(n, 1);  %Block Number
       F(force:(force+11), 3) = [1; 1; 1; 2; 2; 2; 3; 3; 3; 4; 4; 4]; % Knob Number
       F((force+12):(force+23), 3) = [1; 1; 1; 2; 2; 2; 3; 3; 3; 4; 4; 4]; % Knob Number
       F(force:(force+11), 4:6) = Pf_22_0;  %Cordinate z = -1.5
       F((force+12):(force+23), 4:6) = Pf_22_0;
       F((force+12):(force+23), 6) = -0.5 * ones(12, 1); %Cordinate z = -0.5
       F(force:(force+11), 7) = [10; 20; -10; 10; -20; -10; 10; 20; -10; 10; -20; -10]; %setting the upper and lower limits of the normal force 
       F((force+12):(force+23), 7) = [10; 20; -10; 10; -20; -10; 10; 20; -10; 10; -20; -10];
       force = force + 24; %A 2x2 block have 24 friction force vectors in the force model
   end
end
force = force - 1; %just adjustment, matlab start its vectors as 1.
Ff = F(1:force, :);
