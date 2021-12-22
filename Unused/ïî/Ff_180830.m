function Ff = Ff_180830(join)  %組立途中の最後の押し込むブロック用
%% constant number
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
for n = 1 : loop(1);  %接続凸部各組の摩擦力の数をカウント
    if(join(n, 2) == 11)
        force = force + 8;   
    else
        force = force + 6;
    end
end
Ff = zeros(force, 13);
Ff( 1 : force, 1) = 1 : force;  %Force Number
Ff(1:force, 6:5:11) = [-ones(force, 1), ones(force, 1)];  %力の向き(1：上段は負，2：下段は正)
count = 1;  knobs = zeros(1, 3);
for n = 1 : loop(1);
    type_1 = join(n, 2);  type_2 = join(n, 5);  knob_1 = join(n, 4);  knob_2 = join(n, 7);
    if(type_1 == 11)
        Ff(count:count+3, 2:6:8) = [join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6)];  %Block Number
        Ff(count:count+3, 3:6:9) = [join(n, 4:3:7); join(n, 4:3:7); join(n, 4:3:7); join(n, 4:3:7)];  %Knob Number
        Ff(count:count+3, 4:6) = Pf_11;  %座標1(z = -1.5)
        %座標2(z = 2.5)
        if(type_2 == 11)
            Ff(count:count+3, 10:12) = Pf11;
        elseif(type_2 == 12)
            if(knob_2 == 1)
                Ff(count:count+3, 10:12) = Pf12_1;
            elseif(knob_2 == 2)
                Ff(count:count+3, 10:12) = Pf12_2;
            end
        elseif(type_2 == 21)
            if(knob_2 == 1)
                Ff(count:count+3, 10:12) = Pf21_1;
            elseif(knob_2 == 2)
                Ff(count:count+3, 10:12) = Pf21_2;
            end
        else
            if(knob_2 == 1)
                Ff(count:count+3, 10:12) = Pf22_1;
            elseif(knob_2 == 2)
                Ff(count:count+3, 10:12) = Pf22_2;
            elseif(knob_2 == 3)
                Ff(count:count+3, 10:12) = Pf22_3;
            elseif(knob_2 == 4)
                Ff(count:count+3, 10:12) = Pf22_4;
            end
        end
        Ff(count:count+3, 12) = 2.5;
        Ff(count+4:count+7, 2:11) =Ff(count:count+3, 2:11);  %追加するFf
        Ff(count+4:count+7, 6) = -0.5;  %座標1(z = -0.5)
        Ff(count+4:count+7, 12) = 3.5;  %座標2(z = 3.5)
        Ff(force:(force+3), 7) = [10; 20; -20; -10]; %%垂直抗力の上下限値設定用
        Ff((force+4):(force+7), 7) = [10; 20; -20; -10]; %%垂直抗力の上下限値設定用
        count = count + 8;
    else
        Ff(count:count+2, 2:6:8) = [join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6)];  %Block Number
        Ff(count:count+2, 3:6:9) = [join(n, 4:3:7); join(n, 4:3:7); join(n, 4:3:7)];  %Knob Number
        if(type_1 == 12)
            if(knob_1 == 1)
                 Ff(count:count+2, 4:6) = Pf_12_0(1:3, 1:3);
                 knobs = [1, 2, 4];
            elseif(knob_1 == 2)
                 Ff(count:count+2, 4:6) = Pf_12_0(4:6, 1:3);
                 knobs = [1, 3, 4];
            end
        elseif(type_1 == 21)
            if(knob_1 == 1)
                 Ff(count:count+2, 4:6) = Pf_21_0(1:3, 1:3);
                 knobs = [1, 2, 3];
            elseif(knob_1 == 2)
                 Ff(count:count+2, 4:6) = Pf_21_0(4:6, 1:3);
                 knobs = [2, 3, 4];
            end
        else
            if(knob_1 == 1)
                 Ff(count:count+2, 4:6) = Pf_22_0(1:3, 1:3);
                 knobs = [1, 2, 4];
            elseif(knob_1 == 2)
                 Ff(count:count+2, 4:6) = Pf_22_0(4:6, 1:3);
                 knobs = [1, 3, 4];
            elseif(knob_1 == 3)
                 Ff(count:count+2, 4:6) = Pf_22_0(7:9, 1:3);
                 knobs = [1, 2, 4];
            elseif(knob_1 == 4)
                 Ff(count:count+2, 4:6) = Pf_22_0(10:12, 1:3);
                 knobs = [1, 3, 4];
            end
        end
        if(type_2 == 12)
            if(knob_2 == 1)
                Ff(count:count+2, 10:12) = [Pf12_1(knobs(1), 1:3); Pf12_1(knobs(2), 1:3); Pf12_1(knobs(3), 1:3)];
            elseif(knob_2 == 2)
                Ff(count:count+2, 10:12) = [Pf12_2(knobs(1), 1:3); Pf12_2(knobs(2), 1:3); Pf12_2(knobs(3), 1:3)];
            end
        elseif(type_2 == 21)
            if(knob_2 == 1)
                Ff(count:count+2, 10:12) = [Pf21_1(knobs(1), 1:3); Pf21_1(knobs(2), 1:3); Pf21_1(knobs(3), 1:3)];
            elseif(knob_2 == 2)
                Ff(count:count+2, 10:12) = [Pf21_2(knobs(1), 1:3); Pf21_2(knobs(2), 1:3); Pf21_2(knobs(3), 1:3)];
            end
        else
            if(knob_2 == 1)
                Ff(count:count+2, 10:12) = [Pf22_1(knobs(1), 1:3); Pf22_1(knobs(2), 1:3); Pf22_1(knobs(3), 1:3)];
            elseif(knob_2 == 2)
                Ff(count:count+2, 10:12) = [Pf22_2(knobs(1), 1:3); Pf22_2(knobs(2), 1:3); Pf22_2(knobs(3), 1:3)];
            elseif(knob_2 == 3)
                Ff(count:count+2, 10:12) = [Pf22_3(knobs(1), 1:3); Pf22_3(knobs(2), 1:3); Pf22_3(knobs(3), 1:3)];
            elseif(knob_2 == 4)
                Ff(count:count+2, 10:12) = [Pf22_4(knobs(1), 1:3); Pf22_4(knobs(2), 1:3); Pf22_4(knobs(3), 1:3)];
            end
        end
        Ff(count:count+2, 12) = 2.5;
        Ff(count+3:count+5, 2:11) =Ff(count:count+2, 2:11);  %追加するFf
        Ff(count+3:count+5, 6) = -0.5;  %座標1(z = -0.5)
        Ff(count+3:count+5, 12) = 3.5;  %座標2(z = 3.5)
        %垂直抗力の上下限値設定用
        for i = 1 : 3
            if(knobs(i) == 1)
                Ff((count-1+i):3:(count+2+i), 7) = 10;
                %Ff(count+2+i, 6) = 10;
            elseif(knobs(i) == 2)
                Ff((count-1+i):3:(count+2+i), 7) = 20;
                %Ff(count+2+i, 6) = 20;
            elseif(knobs(i) == 3)
                Ff((count-1+i):3:(count+2+i), 7) = -20;
                %Ff(count+2+i, 6) = -20;
            elseif(knobs(i) == 4)
                Ff((count-1+i):3:(count+2+i), 7) = -10;
                %Ff(count+2+i, 6) = -10;
            end
        end
        %
        count = count + 6;
    end
end