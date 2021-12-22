function Ff = Ff(join)
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
%% Ff = (Force_number, Block_number1, x1, y1, z1, -1, Block_number2, x2, y2, z2, 1)
loop = size(join);
force = 0;
for n = 1 : loop(1);
    if(join(n, 2) == 11)
        force = force + 4;
    else
        force = force + 3;
    end
end
Ff = zeros(force, 11);
Ff( 1 : force, 1) = 1 : force;
Ff(1:force, 6:5:11) = [-ones(force, 1), ones(force, 1)];
count = 1;  knobs = zeros(1, 3);
for n = 1 : loop(1);
    type_1 = join(n, 2);  type_2 = join(n, 5);  knob_1 = join(n, 4);  knob_2 = join(n, 7);
    if(type_1 == 11)
        Ff(count:count+3, 2:5:7) = [join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6)];
        Ff(count:count+3, 3:5) = Pf_11;
        Ff(count:count+3, 5) = 0.5;
        if(type_2 == 11)
            Ff(count:count+3, 8:10) = Pf11;
        elseif(type_2 == 12)
            if(knob_2 == 1)
                Ff(count:count+3, 8:10) = Pf12_1;
            elseif(knob_2 == 2)
                Ff(count:count+3, 8:10) = Pf12_2;
            end
        elseif(type_2 == 21)
            if(knob_2 == 1)
                Ff(count:count+3, 8:10) = Pf21_1;
            elseif(knob_2 == 2)
                Ff(count:count+3, 8:10) = Pf21_2;
            end
        else
            if(knob_2 == 1)
                Ff(count:count+3, 8:10) = Pf22_1;
            elseif(knob_2 == 2)
                Ff(count:count+3, 8:10) = Pf22_2;
            elseif(knob_2 == 3)
                Ff(count:count+3, 8:10) = Pf22_3;
            elseif(knob_2 == 4)
                Ff(count:count+3, 8:10) = Pf22_4;
            end
        end
        Ff(count:count+3, 10) = 3.5;
        count = count + 4;
    else
        Ff(count:count+2, 2:5:7) = [join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6)];
        if(type_1 == 12)
            if(knob_1 == 1)
                 Ff(count:count+2, 3:5) = Pf_12_0(1:3, 1:3);
                 knobs = [1, 2, 4];
            elseif(knob_1 == 2)
                 Ff(count:count+2, 3:5) = Pf_12_0(4:6, 1:3);
                 knobs = [1, 3, 4];
            end
        elseif(type_1 == 21)
            if(knob_1 == 1)
                 Ff(count:count+2, 3:5) = Pf_21_0(1:3, 1:3);
                 knobs = [1, 2, 3];
            elseif(knob_1 == 2)
                 Ff(count:count+2, 3:5) = Pf_21_0(4:6, 1:3);
                 knobs = [2, 3, 4];
            end
        else
            if(knob_1 == 1)
                 Ff(count:count+2, 3:5) = Pf_22_0(1:3, 1:3);
                 knobs = [1, 2, 4];
            elseif(knob_1 == 2)
                 Ff(count:count+2, 3:5) = Pf_22_0(4:6, 1:3);
                 knobs = [1, 3, 4];
            elseif(knob_1 == 3)
                 Ff(count:count+2, 3:5) = Pf_22_0(7:9, 1:3);
                 knobs = [1, 2, 4];
            elseif(knob_1 == 4)
                 Ff(count:count+2, 3:5) = Pf_22_0(10:12, 1:3);
                 knobs = [1, 3, 4];
            end
        end
        if(type_2 == 12)
            if(knob_2 == 1)
                Ff(count:count+2, 8:10) = [Pf12_1(knobs(1), 1:3); Pf12_1(knobs(2), 1:3); Pf12_1(knobs(3), 1:3)];
            elseif(knob_2 == 2)
                Ff(count:count+2, 8:10) = [Pf12_2(knobs(1), 1:3); Pf12_2(knobs(2), 1:3); Pf12_2(knobs(3), 1:3)];
            end
        elseif(type_2 == 21)
            if(knob_2 == 1)
                Ff(count:count+2, 8:10) = [Pf21_1(knobs(1), 1:3); Pf21_1(knobs(2), 1:3); Pf21_1(knobs(3), 1:3)];
            elseif(knob_2 == 2)
                Ff(count:count+2, 8:10) = [Pf21_2(knobs(1), 1:3); Pf21_2(knobs(2), 1:3); Pf21_2(knobs(3), 1:3)];
            end
        else
            if(knob_2 == 1)
                Ff(count:count+2, 8:10) = [Pf22_1(knobs(1), 1:3); Pf22_1(knobs(2), 1:3); Pf22_1(knobs(3), 1:3)];
            elseif(knob_2 == 2)
                Ff(count:count+2, 8:10) = [Pf22_2(knobs(1), 1:3); Pf22_2(knobs(2), 1:3); Pf22_2(knobs(3), 1:3)];
            elseif(knob_2 == 3)
                Ff(count:count+2, 8:10) = [Pf22_3(knobs(1), 1:3); Pf22_3(knobs(2), 1:3); Pf22_3(knobs(3), 1:3)];
            elseif(knob_2 == 4)
                Ff(count:count+2, 8:10) = [Pf22_4(knobs(1), 1:3); Pf22_4(knobs(2), 1:3); Pf22_4(knobs(3), 1:3)];
            end
        end
        Ff(count:count+2, 5) = 0.5;
        Ff(count:count+2, 10) = 3.5;
        count = count + 3;
    end
end