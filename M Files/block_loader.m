function All_Forces = block_loader(model)

bc = zeros(9,9); %block checker matrix, support up to 9 b 9 blocks
All_Forces = [];

loop = size(model,1);
for n = 1 : loop
    col = floor(model(n, 5)/10);                             %block number of collums
    row = round(10*((model(n, 5)/10)-floor(model(n, 5)/10))); %block number of rows
    bc(col,row) = 1;
end

if(bc(1,1) == 1) %1x1 block
    Pf = force_position(1,1,"ff");
    Pf11_0 = Pf(1:4,:);
    a = size(Pf11_0,1);
    Pf_11_0 = [Pf11_0(1:4, 1:2), -Pf11_0(1:4, 3), [11;a;zeros(a-2,1)]];
    All_Forces = [All_Forces;Pf_11_0];
end

if(bc(1,2) == 1) %1x2 block
    Pf = force_position(1,2,"ff");
    Pf12_1 = Pf(1:4,:);
    Pf12_2 = Pf(5:8,:);
    Pf12_0 = [Pf12_1(1:2, 1:3); Pf12_1(4, 1:3); Pf12_2(1, 1:3); Pf12_2(3:4,1:3)]; %1x2 blocks friction forces position
    a = size(Pf12_0,1);
    Pf_12_0 = [Pf12_0(1:6, 1:2), -Pf12_0(1:6, 3), [12;a;zeros(a-2,1)]]; %Same as above but negative z axis
    All_Forces = [All_Forces;Pf_12_0];
end

if(bc(2,1) == 1) %2x1 block
    Pf = force_position(2,1,"ff");
    Pf21_1 = Pf(1:4,:);
    Pf21_2 = Pf(5:8,:);
    Pf21_0 = [Pf21_1(1:3, 1:3); Pf21_2(2:4, 1:3)]; %2x1 blocks friction forces position
    a = size(Pf21_0,1);
    Pf_21_0 = [Pf21_0(1:6, 1:2), -Pf21_0(1:6, 3), [21;a;zeros(a-2,1)]]; %Same as above but negative z axis
    All_Forces = [All_Forces;Pf_21_0];
end

if(bc(2,2) == 1) %2x2 Block
    Pf = force_position(2,2,"ff");
    Pf22_1 = Pf(1:4,:);
    Pf22_2 = Pf(5:8,:);
    Pf22_3 = Pf(9:12,:);
    Pf22_4 = Pf(13:16,:);
    Pf22_0 = [Pf22_1(1:2, 1:3); Pf22_1(4, 1:3); Pf22_2(1, 1:3); Pf22_2(3:4, 1:3); Pf22_3(1:2, 1:3); Pf22_3(4, 1:3); Pf22_4(1, 1:3); Pf22_4(3:4, 1:3)]; %2x2 blocks friction forces position
    a = size(Pf22_0,1);
    Pf_22_0 = [Pf22_0(1:12, 1:2), -Pf22_0(1:12, 3), [22;a;zeros(a-2,1)]]; %Same as above but negative z axis
    All_Forces = [All_Forces;Pf_22_0];
end