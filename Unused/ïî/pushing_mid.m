function beq = pushing_mid(model, N)
% 押し込まれるブロックの摩擦力
% constant number
g = 9.81; T=70*g;
% 1*1
pf1 = [-1.25,     0, 1.5];  pf2 = [    0, -1.25, 1.5];  pf3 = [    0,  1.25, 1.5];  pf4 = [ 1.25,     0, 1.5];
Pf11 = [pf1; pf2; pf3; pf4];
%1*2
pf1 = [-1.25,    -2, 1.5];  pf2 = [    0, -3.25, 1.5];  pf3 = [    0, -0.75, 1.5];  pf4 = [ 1.25,    -2, 1.5];
Pf12_1 = [pf1; pf2; pf3; pf4];
pf1 = [-1.25,     2, 1.5];  pf2 = [    0,  0.75, 1.5];  pf3 = [    0,  3.25, 1.5];  pf4 = [ 1.25,     2, 1.5];
Pf12_2 = [pf1; pf2; pf3; pf4];
%2*1
pf1 = [-3.25,     0, 1.5];  pf2 = [   -2, -1.25, 1.5];  pf3 = [   -2,  1.25, 1.5];  pf4 = [-0.75,     0, 1.5];
Pf21_1 = [pf1; pf2; pf3; pf4];
pf1 = [0.75,     0, 1.5];  pf2 = [   2, -1.25, 1.5];  pf3 = [   2,  1.25, 1.5];  pf4 = [3.25,     0, 1.5];
Pf21_2 = [pf1; pf2; pf3; pf4];
%2*2
pf1 = [-3.25,    -2, 1.5];  pf2 = [   -2, -3.25, 1.5];  pf3 = [   -2, -0.75, 1.5];  pf4 = [-0.75,    -2, 1.5];
Pf22_1 = [pf1; pf2; pf3; pf4];
pf1 = [-3.25,    2, 1.5];  pf2 = [   -2, 0.75, 1.5];  pf3 = [   -2, 3.25, 1.5];  pf4 = [-0.75,    2, 1.5];
Pf22_2 = [pf1; pf2; pf3; pf4];
pf1 = [0.75,    -2, 1.5];  pf2 = [   2, -3.25, 1.5];  pf3 = [   2, -0.75, 1.5];  pf4 = [3.25,    -2, 1.5];
Pf22_3 = [pf1; pf2; pf3; pf4];
pf1 = [0.75,    2, 1.5];  pf2 = [   2, 0.75, 1.5];  pf3 = [    2 3.25, 1.5];  pf4 = [3.25,    2, 1.5];
Pf22_4 = [pf1; pf2; pf3; pf4];

%押し込むブロックの情報
model_pushing = model(N, 1:5);
knob_pushing = knob(model_pushing);
z_max = model_pushing(3);
%押し込まれるブロックの情報
knob_push = knob_line(model, z_max - 1);
%押し込まれる凸部を求める
knob_push = [knob_push; knob_pushing];
join_push = join(knob_push, z_max);
%押し込み力のかかるブロック数 block_number, 凸部数 knob_number(1) を求める
block_number = 0;  knob_number = size(join_push);
b = zeros(knob_number(1));
for n = 1 : knob_number(1)
    if(n == 1)
       block_number = 1;
       b(1) = join_push(1, 6);
    else
        check = 0;
        for m = 1 : (n - 1);
           if(join_push(n, 6) == b(m))
              check = -1; 
           end
        end
        if(check == 0)
            block_number = block_number + 1;
            b(block_number) = join_push(n, 6);
        end
    end
end
% beq を求める
beq = zeros(block_number, 7);
for m = 1 : block_number
    beq(m, 1) = b(m);
    block = b(m); 
    Mx = 0; My = 0;
    for n = 1 : knob_number(1)
        type_u = join_push(n, 2);
        knob_u = join_push(n, 4);
        if(type_u == 11)
            T_push = 4;
            knobs = [1, 2, 3, 4];
        elseif(type_u == 12)
            T_push = 3;
            if(knob_u == 1) 
                knobs = [1, 2, 4];
            elseif(knob_u == 2)
                knobs = [1, 3, 4];
            end
        elseif(type_u == 21)
            T_push = 3;
            if(knob_u == 1) 
                knobs = [1, 2, 3];
            elseif(knob_u == 2)
                knobs = [2, 3, 4];
            end
        else
            T_push = 2;
            if(knob_u == 1) 
                knobs = [1, 2];
            elseif(knob_u == 2)
                knobs = [1, 3];
            elseif(knob_u == 3) 
                knobs = [2, 4];
            elseif(knob_u == 4)
                knobs = [3, 4];
            end
        end
        if(join_push(n, 6) ==  block)
            type_l = join_push(n, 5);
            knob_l = join_push(n,7);
            if(type_l == 11)
                for l = 1 : T_push
                    Mx = Mx + Pf11(knobs(l), 2) * T / T_push;
                    My = My - Pf11(knobs(l), 1) * T / T_push;
                end
            elseif(type_l == 12)
                if(knob_l == 1)
                    for l = 1 : T_push
                        Mx = Mx + Pf12_1(knobs(l), 2) * T / T_push;
                        My = My - Pf12_1(knobs(l), 1) * T / T_push;
                    end
                elseif(knob_l == 2)
                    for l = 1 : T_push
                        Mx = Mx + Pf12_2(knobs(l), 2) * T / T_push;
                        My = My - Pf12_2(knobs(l), 1) * T / T_push;
                    end
                end
            elseif(type_l == 21)
                if(knob_l == 1)
                    for l = 1 : T_push
                        Mx = Mx + Pf21_1(knobs(l), 2) * T / T_push;
                        My = My - Pf21_1(knobs(l), 1) * T / T_push;
                    end
                elseif(knob_l == 2)
                    for l = 1 : T_push
                        Mx = Mx + Pf21_2(knobs(l), 2) * T / T_push;
                        My = My - Pf21_2(knobs(l), 1) * T / T_push;
                    end
                end
            else
                if(knob_l == 1)
                    for l = 1 : T_push
                        Mx = Mx + Pf22_1(knobs(l), 2) * T / T_push;
                        My = My - Pf22_1(knobs(l), 1) * T / T_push;
                    end
                elseif(knob_l == 2)
                    for l = 1 : T_push
                        Mx = Mx + Pf22_2(knobs(l), 2) * T / T_push;
                        My = My - Pf22_2(knobs(l), 1) * T / T_push;
                    end
                elseif(knob_l == 3)
                    for l = 1 : T_push
                        Mx = Mx + Pf22_3(knobs(l), 2) * T / T_push;
                        My = My - Pf22_3(knobs(l), 1) * T / T_push;
                    end
                elseif(knob_l == 4)
                    for l = 1 : T_push
                        Mx = Mx + Pf22_4(knobs(l), 2) * T / T_push;
                        My = My - Pf22_4(knobs(l), 1) * T / T_push;
                    end
                end
            end
            beq(m, 4:6) = [beq(m, 4) + T, beq(m, 5) + Mx, beq(m, 6) + My];
            Mx = 0;  My = 0;
        end
    end
end