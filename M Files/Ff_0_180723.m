function Ff = Ff_0_180723(model)
%Find the friction force position in blocks that are in the first layer of 
%the block model in regard to its own coordinate axis.
%input: model:(Block_Number, x, y, z, type, color)
%output: Ff = (Force_Number, Block_Number, Knob_Number, x, y, z, *, ...)
%             *: upper and lower limits of the normal force
%Obs: Pfnm_xy = position force of a block with n columns and m rows touching
%from the xth to the yth knob. Special case to 0 which represents that it is
%touching the entire block. a "_" after the Pf, mean that this position is 
%the negative one.

%1x1 block
Pf = force_position(1,1,"ff");
Pf11_0 = [Pf(1:4,:)];
a = size(Pf11_0,1);
Pf_11_0 = [Pf11_0(1:4, 1:2), -Pf11_0(1:4, 3), [11;a;zeros(a-2,1)]];
All_Forces = [All_Forces;Pf_11_0];

%1x2 block
Pf = force_position(1,2,"ff");
Pf12_1 = Pf(1:4,:);
Pf12_2 = Pf(5:8,:);
Pf12_0 = [Pf12_1(1:2, 1:3); Pf12_1(4, 1:3); Pf12_2(1, 1:3); Pf12_2(3:4,1:3)]; %1x2 blocks friction forces position
a = size(Pf12_0,1);
Pf_12_0 = [Pf12_0(1:6, 1:2), -Pf12_0(1:6, 3), [12;a;zeros(a-2,1)]]; %Same as above but negative z axis
All_Forces = [All_Forces;Pf_12_0];

%2x1 block
Pf = force_position(2,1,"ff");
Pf21_1 = Pf(1:4,:);
Pf21_2 = Pf(5:8,:);
Pf21_0 = [Pf21_1(1:3, 1:3); Pf21_2(2:4, 1:3)]; %2x1 blocks friction forces position
a = size(Pf21_0,1);
Pf_21_0 = [Pf21_0(1:6, 1:2), -Pf21_0(1:6, 3), [21;a;zeros(a-2,1)]]; %Same as above but negative z axis
All_Forces = [All_Forces;Pf_21_0];

%2x2 Block
Pf = force_position(2,2,"ff");
Pf22_1 = Pf(1:4,:);
Pf22_2 = Pf(5:8,:);
Pf22_3 = Pf(9:12,:);
Pf22_4 = Pf(13:16,:);
Pf22_0 = [Pf22_1(1:2, 1:3); Pf22_1(4, 1:3); Pf22_2(1, 1:3); Pf22_2(3:4, 1:3); Pf22_3(1:2, 1:3); Pf22_3(4, 1:3); Pf22_4(1, 1:3); Pf22_4(3:4, 1:3)]; %2x2 blocks friction forces position
a = size(Pf22_0,1);
Pf_22_0 = [Pf22_0(1:12, 1:2), -Pf22_0(1:12, 3), [22;a;zeros(a-2,1)]]; %Same as above but negative z axis
All_Forces = [All_Forces;Pf_22_0];

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
max_force_number = 24; %Maximum number of forces that can appear in a block. (for a 2x2 block is 24)
F = zeros(count*max_force_number, 13);
F(1:(count*max_force_number), 1) = 1: (count*max_force_number);  %Force Number
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