function FP = forces_position(force_type)
%returns the position of a specified force type for a specified block type
%this is used only to make the functions that calls this one more readable
%input: block_type: 1x1, 1x2, 2x1, 2x2
%       force_type: friction force, X axis normal force, Y axis normal force
%output: FP = a matrix with 3 collums (x,y,z) of a force.
%        this depends on which force and block is passed as argument. For
%        the correct usage, it is necessary a carrefull analysis to
%        separete which force is which in the function that call this one
FP = [];

if(force_type == "ff")
    %1x1 Block
    pf1 = [-1.25,     0, 1.5];  pf2 = [    0, -1.25, 1.5];  pf3 = [    0,  1.25, 1.5];  pf4 = [ 1.25,     0, 1.5];
    Pf11 = [pf1; pf2; pf3; pf4]; %1x1 blocks friction forces position
    Pf_11 = [Pf11(1:4, 1:2), -Pf11(1:4, 3)]; %Same as above but negative z axis
    FP = [FP;Pf_11];
    
    %1X2 Block
    pf1 = [-1.25,    -2, 1.5];  pf2 = [    0, -3.25, 1.5];  pf3 = [    0, -0.75, 1.5];  pf4 = [ 1.25,    -2, 1.5];
    Pf12_1 = [pf1; pf2; pf3; pf4]; %Knob 1 forces position
    pf1 = [-1.25,     2, 1.5];  pf2 = [    0,  0.75, 1.5];  pf3 = [    0,  3.25, 1.5];  pf4 = [ 1.25,     2, 1.5];
    Pf12_2 = [pf1; pf2; pf3; pf4]; %Knob 2 forces position
    Pf12_0 = [Pf12_1(1:2, 1:3); Pf12_1(4, 1:3); Pf12_2(1, 1:3); Pf12_2(3:4,1:3)]; %Remove the forces that doesn't exist
    Pf_12_0 = [Pf12_0(1:6, 1:2), -Pf12_0(1:6, 3)]; %Correct the z axis value
    FP = [FP;Pf12_1;Pf12_2;Pf_12_0];
    
    %2x1 Block
    pf1 = [-3.25,     0, 1.5];  pf2 = [   -2, -1.25, 1.5];  pf3 = [   -2,  1.25, 1.5];  pf4 = [-0.75,     0, 1.5];
    Pf21_1 = [pf1; pf2; pf3; pf4]; %Knob 1 forces position
    pf1 = [0.75,     0, 1.5];  pf2 = [   2, -1.25, 1.5];  pf3 = [   2,  1.25, 1.5];  pf4 = [3.25,     0, 1.5];
    Pf21_2 = [pf1; pf2; pf3; pf4]; %Knob 2 forces position
    Pf21_0 = [Pf21_1(1:3, 1:3); Pf21_2(2:4, 1:3)]; %Remove the forces that doesn't exist
    Pf_21_0 = [Pf21_0(1:6, 1:2), -Pf21_0(1:6, 3)]; %Correct the z axis value
    FP = [FP;Pf21_1;Pf21_2;Pf_21_0];
    
    %2x2 Block
    pf1 = [-3.25,    -2, 1.5];  pf2 = [   -2, -3.25, 1.5];  pf3 = [   -2, -0.75, 1.5];  pf4 = [-0.75,    -2, 1.5];
    Pf22_1 = [pf1; pf2; pf3; pf4]; %Knob 1
    pf1 = [-3.25,    2, 1.5];  pf2 = [   -2, 0.75, 1.5];  pf3 = [   -2, 3.25, 1.5];  pf4 = [-0.75,    2, 1.5];
    Pf22_2 = [pf1; pf2; pf3; pf4]; %Knob 2
    pf1 = [0.75,    -2, 1.5];  pf2 = [   2, -3.25, 1.5];  pf3 = [   2, -0.75, 1.5];  pf4 = [3.25,    -2, 1.5];
    Pf22_3 = [pf1; pf2; pf3; pf4]; %Knob 3
    pf1 = [0.75,    2, 1.5];  pf2 = [   2, 0.75, 1.5];  pf3 = [    2, 3.25, 1.5];  pf4 = [3.25,    2, 1.5];
    Pf22_4 = [pf1; pf2; pf3; pf4]; %Knob 4
    Pf22_0 = [Pf22_1(1:2, 1:3); Pf22_1(4, 1:3); Pf22_2(1, 1:3); Pf22_2(3:4, 1:3); Pf22_3(1:2, 1:3); Pf22_3(4, 1:3); Pf22_4(1, 1:3); Pf22_4(3:4, 1:3)]; %Remove the forces that doesn't exist
    Pf_22_0 = [Pf22_0(1:12, 1:2), -Pf22_0(1:12, 3)]; %Correct the z axis value
    FP = [FP;Pf22_1;Pf22_2;Pf22_3;Pf22_4;Pf_22_0];
elseif(force_type == "fnx")
    % 1x1 Block
    pn1 = [2, -2, -1.5];  pn2 = [2,  2, -1.5];  pn3 = [2, -2,  1.5];  pn4 = [2,  2,  1.5];
    Pnx11 = [pn1; pn2; pn3; pn4];  Pnx_11 = [-Pnx11(1:4, 1), Pnx11(1:4, 2:3)];
    FP = [FP;Pnx11;Pnx_11];
    
    % 1x2 Block
    pn1 = [2, -4, -1.5];  pn2 = [2,  4, -1.5];  pn3 = [2, -4,  1.5];  pn4 = [2,  4,  1.5];
    Pnx12_0 = [pn1; pn2; pn3; pn4];  Pnx_12_0 = [-Pnx12_0(1:4, 1), Pnx12_0(1:4, 2:3)]; 
    pn1 = [2, -4, -1.5];  pn2 = [2,  0, -1.5];  pn3 = [2, -4,  1.5];  pn4 = [2,  0,  1.5];
    Pnx12_1 = [pn1; pn2; pn3; pn4];  Pnx_12_1 = [-Pnx12_1(1:4, 1), Pnx12_1(1:4, 2:3)]; %Only knob 1 X side
    pn1 = [2, 0, -1.5];  pn2 = [2, 4, -1.5];  pn3 = [2, 0,  1.5];  pn4 = [2, 4,  1.5];
    Pnx12_2 = [pn1; pn2; pn3; pn4];  Pnx_12_2 = [-Pnx12_2(1:4, 1), Pnx12_2(1:4, 2:3)]; %Only knob 2 X side
    FP = [FP;Pnx12_0;Pnx_12_0;Pnx12_1;Pnx_12_1;Pnx12_2;Pnx_12_2];
    
    % 2x1 Block
    pn1 = [-4, -2, -1.5];  pn2 = [-4,  2, -1.5];  pn3 = [-4, -2,  1.5];  pn4 = [-4,  2,  1.5];
    Pnx21_1 = [pn1; pn2; pn3; pn4];  Pnx21_2 = [-Pnx21_1(1:4, 1), Pnx21_1(1:4, 2:3)];
    FP = [FP;Pnx21_1;Pnx21_2];
    
    % 2x2 Block
    pn1 = [4, -4, -1.5];  pn2 = [4,  4, -1.5];  pn3 = [4, -4,  1.5];  pn4 = [4,  4,  1.5];
    Pnx22_0 = [pn1; pn2; pn3; pn4];  Pnx_22_0 = [-Pnx22_0(1:4, 1), Pnx22_0(1:4, 2:3)]; %Entire X side
    pn1 = [-4, -4, -1.5];  pn2 = [-4,  0, -1.5];  pn3 = [-4, -4,  1.5];  pn4 = [-4,  0,  1.5];
    Pnx22_1 = [pn1; pn2; pn3; pn4];  Pnx22_3 = [-Pnx22_1(1:4, 1), Pnx22_1(1:4, 2:3)]; %Only knob 1/knob 3 X side
    pn1 = [-4, 0, -1.5];  pn2 = [-4, 4, -1.5];  pn3 = [-4, 0,  1.5];  pn4 = [-4, 4,  1.5];
    Pnx22_2 = [pn1; pn2; pn3; pn4];  Pnx22_4 = [-Pnx22_2(1:4, 1), Pnx22_2(1:4, 2:3)]; %Only knob 2/knob 4 X side
    FP = [FP;Pnx22_0;Pnx_22_0;Pnx22_1;Pnx22_3;Pnx22_2;Pnx22_4];
    
elseif(force_type == "fny")
    % 1x1 Block
    pn1 = [-2, 2, -1.5];  pn2 = [-2, 2,  1.5];  pn3 = [ 2, 2, -1.5];  pn4 = [ 2, 2,  1.5];
    Pny11 = [pn1; pn2; pn3; pn4];  Pny_11 = [Pny11(1:4, 1), -Pny11(1:4, 2), Pny11(1:4, 3)];
    FP = [FP;Pny11;Pny_11];
    
    % 1x2 Block
    pn1 = [-2, -4, -1.5];  pn2 = [-2, -4,  1.5];  pn3 = [ 2, -4, -1.5];  pn4 = [ 2, -4,  1.5];
    Pny12_1 = [pn1; pn2; pn3; pn4];  Pny12_2 = [Pny12_1(1:4, 1), -Pny12_1(1:4, 2), Pny12_1(1:4, 3)];
    FP = [FP;Pny12_1;Pny12_2];
    
    % 2x1 Block
    pn1 = [-4, 2, -1.5];  pn2 = [-4, 2,  1.5];  pn3 = [ 4, 2, -1.5];  pn4 = [ 4, 2,  1.5];
    Pny21_0 = [pn1; pn2; pn3; pn4];  Pny_21_0 = [Pny21_0(1:4, 1), -Pny21_0(1:4, 2), Pny21_0(1:4, 3)]; %Entire Y side
    pn1 = [-4, 2, -1.5];  pn2 = [-4, 2,  1.5];  pn3 = [ 0, 2, -1.5];  pn4 = [ 0, 2,  1.5];
    Pny21_1 = [pn1; pn2; pn3; pn4];  Pny_21_1 = [Pny21_1(1:4, 1), -Pny21_1(1:4, 2), Pny21_1(1:4, 3)]; %Only knob 1 Y side
    pn1 = [0, 2, -1.5];  pn2 = [0, 2,  1.5];  pn3 = [4, 2, -1.5];  pn4 = [4, 2,  1.5];
    Pny21_2 = [pn1; pn2; pn3; pn4];  Pny_21_2 = [Pny21_2(1:4, 1), -Pny21_2(1:4, 2), Pny21_2(1:4, 3)]; %Only knob 2 Y side
    FP = [FP;Pny21_0;Pny_21_0;Pny21_1;Pny_21_1;Pny21_2;Pny_21_2];
    
    % 2x2 Block
    pn1 = [-4, 4, -1.5];  pn2 = [-4, 4,  1.5];  pn3 = [ 4, 4, -1.5];  pn4 = [ 4, 4,  1.5];
    Pny22_0 = [pn1; pn2; pn3; pn4];  Pny_22_0 = [Pny22_0(1:4, 1), -Pny22_0(1:4, 2), Pny22_0(1:4, 3)]; %Entire Y side
    pn1 = [-4, -4, -1.5];  pn2 = [-4, -4,  1.5];  pn3 = [ 0, -4, -1.5];  pn4 = [ 0, -4,  1.5];
    Pny22_1 = [pn1; pn2; pn3; pn4];  Pny22_2 = [Pny22_1(1:4, 1), -Pny22_1(1:4, 2), Pny22_1(1:4, 3)]; %Only knob 1/knob 2 Y side
    pn1 = [0, -4, -1.5];  pn2 = [0, -4,  1.5];  pn3 = [4, -4, -1.5];  pn4 = [4, -4,  1.5];
    Pny22_3 = [pn1; pn2; pn3; pn4];  Pny22_4 = [Pny22_3(1:4, 1), -Pny22_3(1:4, 2), Pny22_3(1:4, 3)]; %Only knob 3/knob 4 Y side
    FP = [FP;Pny22_0;Pny_22_0;Pny22_1;Pny22_2;Pny22_3;Pny22_4];
    
elseif(force_type == "fnz")
    % 1x1 Block
    pn1 = [-2, -2, 1.5];  pn2 = [-2,  2, 1.5];  pn3 = [ 2, -2, 1.5];  pn4 = [ 2,  2, 1.5];
    Pnz11 = [pn1; pn2; pn3; pn4];  Pnz_11 = [Pnz11(1:4, 1:2), -Pnz11(1:4, 3)];
    FP = [FP;Pnz11;Pnz_11];
    
    % 1x2 Block
    pn1 = [-2, -4, 1.5];  pn2 = [-2,  0, 1.5];  pn3 = [ 2, -4, 1.5];  pn4 = [ 2,  0, 1.5];
    Pnz12_1 = [pn1; pn2; pn3; pn4];  Pnz_12_1 = [Pnz12_1(1:4, 1:2), -Pnz12_1(1:4, 3)]; %Knob 1
    pn1 = [-2,  0, 1.5];  pn2 = [-2,  4, 1.5];  pn3 = [ 2,  0, 1.5];  pn4 = [ 2,  4, 1.5];
    Pnz12_2 = [pn1; pn2; pn3; pn4];  Pnz_12_2 = [Pnz12_2(1:4, 1:2), -Pnz12_2(1:4, 3)]; %Knob 2
    Pnz12_0 = [Pnz12_1(1, 1:3); Pnz12_2(2, 1:3); Pnz12_1(3, 1:3); Pnz12_2(4, 1:3)]; %All Knobs connection
    Pnz_12_0 = [Pnz12_0(1:4, 1:2), -Pnz12_0(1:4, 3)];
    FP = [FP;Pnz12_1;Pnz_12_1;Pnz12_2;Pnz_12_2;Pnz12_0;Pnz_12_0];
    
    % 2x1 Block
    pn1 = [-4, -2, 1.5];  pn2 = [-4,  2, 1.5];  pn3 = [ 0, -2, 1.5];  pn4 = [ 0,  2, 1.5];
    Pnz21_1 = [pn1; pn2; pn3; pn4];  Pnz_21_1 = [Pnz21_1(1:4, 1:2), -Pnz21_1(1:4, 3)]; %Knob 1
    pn1 = [0, -2, 1.5];  pn2 = [0,  2, 1.5];  pn3 = [4, -2, 1.5];  pn4 = [4,  2, 1.5];
    Pnz21_2 = [pn1; pn2; pn3; pn4];  Pnz_21_2 = [Pnz21_2(1:4, 1:2), -Pnz21_2(1:4, 3)]; %Knob 1
    Pnz21_0 = [Pnz21_1(1:2, 1:3); Pnz21_2(3:4, 1:3)]; %All Knobs connection
    Pnz_21_0 = [Pnz21_0(1:4, 1:2), -Pnz21_0(1:4, 3)];
    FP = [FP;Pnz21_1;Pnz_21_1;Pnz21_2;Pnz_21_2;Pnz21_0;Pnz_21_0];

    % 2x2 Block
    pn1 = [-4, -4, 1.5];  pn2 = [-4,  0, 1.5];  pn3 = [ 0, -4, 1.5];  pn4 = [ 0,  0, 1.5];
    Pnz22_1 = [pn1; pn2; pn3; pn4];  Pnz_22_1 = [Pnz22_1(1:4, 1:2), -Pnz22_1(1:4, 3)]; %Knob 1
    pn1 = [-4, 0, 1.5];  pn2 = [-4, 4, 1.5];  pn3 = [ 0, 0, 1.5];  pn4 = [ 0, 4, 1.5]; 
    Pnz22_2 = [pn1; pn2; pn3; pn4];  Pnz_22_2 = [Pnz22_2(1:4, 1:2), -Pnz22_2(1:4, 3)]; %Knob 2
    pn1 = [0, -4, 1.5];  pn2 = [0,  0, 1.5];  pn3 = [4, -4, 1.5];  pn4 = [4,  0, 1.5];
    Pnz22_3 = [pn1; pn2; pn3; pn4];  Pnz_22_3 = [Pnz22_3(1:4, 1:2), -Pnz22_3(1:4, 3)]; %Knob 3
    pn1 = [0, 0, 1.5];  pn2 = [0, 4, 1.5];  pn3 = [4, 0, 1.5];  pn4 = [4, 4, 1.5];
    Pnz22_4 = [pn1; pn2; pn3; pn4];  Pnz_22_4 = [Pnz22_4(1:4, 1:2), -Pnz22_4(1:4, 3)]; %Knob 4
    Pnz22_0 = [Pnz22_1(1, 1:3); Pnz22_2(2, 1:3); Pnz22_3(3, 1:3); Pnz22_4(4, 1:3)]; %All Knobs connection
    Pnz_22_0 = [Pnz22_0(1:4, 1:2), -Pnz22_0(1:4, 3)];
    Pnz22_12 = [Pnz22_1(1, 1:3); Pnz22_2(2, 1:3); Pnz22_1(3, 1:3); Pnz22_2(4, 1:3)]; %1 and 2 Knob connection
    Pnz_22_12 = [Pnz22_12(1:4, 1:2), -Pnz22_12(1:4, 3)];
    Pnz22_34 = [Pnz22_3(1, 1:3); Pnz22_4(2, 1:3); Pnz22_3(3, 1:3); Pnz22_4(4, 1:3)]; %3 and 4 Knob connection
    Pnz_22_34 = [Pnz22_34(1:4, 1:2), -Pnz22_34(1:4, 3)];
    Pnz22_13 = [Pnz22_1(1:2, 1:3); Pnz22_3(3:4, 1:3)]; %1 and 3 Knob connection
    Pnz_22_13 = [Pnz22_13(1:4, 1:2), -Pnz22_13(1:4, 3)]; 
    Pnz22_24 = [Pnz22_2(1:2, 1:3); Pnz22_4(3:4, 1:3)]; %2 and 4 Knob connection
    Pnz_22_24 = [Pnz22_24(1:4, 1:2), -Pnz22_24(1:4, 3)];
    FP = [FP;Pnz22_1;Pnz_22_1;Pnz22_2;Pnz_22_2;Pnz22_3;Pnz_22_3;Pnz22_4;Pnz_22_4;Pnz22_0;Pnz_22_0];
    FP = [FP;Pnz22_12;Pnz_22_12;Pnz22_34;Pnz_22_34;Pnz22_13;Pnz_22_13;Pnz22_24;Pnz_22_24];
end