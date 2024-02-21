function block_counter(model)
    m_size = size(model);
    block_11 = zeros(1,7);
    block_12 = zeros(1,7);
    block_22 = zeros(1,7);
    block_13 = zeros(1,7);
    block_14 = zeros(1,7);
    block_23 = zeros(1,7);
    block_24 = zeros(1,7);
    block_28 = zeros(1,7);
    support = 0;

    for i = 1 : m_size
        if(model(i,9) == 1)
            support = support + 1;
        elseif(model(i,5) == 1 && model(i,6) == 1)
            block_11(model(i,8)+1) = block_11(model(i,8)+1) + 1;
        elseif(model(i,5) == 2 && model(i,6) == 2)
            block_22(model(i,8)+1) = block_22(model(i,8)+1) + 1;
        elseif((model(i,5) == 1 && model(i,6) == 2)||(model(i,6) == 1 && model(i,5) == 2))
            block_12(model(i,8)+1) = block_12(model(i,8)+1) + 1;
        elseif((model(i,5) == 1 && model(i,6) == 3)||(model(i,6) == 1 && model(i,5) == 3))
            block_13(model(i,8)+1) = block_13(model(i,8)+1) + 1;
        elseif((model(i,5) == 1 && model(i,6) == 4)||(model(i,6) == 1 && model(i,5) == 4))
            block_14(model(i,8)+1) = block_14(model(i,8)+1) + 1;
        elseif((model(i,5) == 2 && model(i,6) == 4)||(model(i,6) == 2 && model(i,5) == 4))
            block_24(model(i,8)+1) = block_24(model(i,8)+1) + 1;
        elseif((model(i,5) == 2 && model(i,6) == 3)||(model(i,6) == 2 && model(i,5) == 3))
            block_23(model(i,8)+1) = block_23(model(i,8)+1) + 1;
        elseif((model(i,5) == 2 && model(i,6) == 8)||(model(i,6) == 2 && model(i,5) == 8))
            block_28(model(i,8)+1) = block_28(model(i,8)+1) + 1;
        else
            1;
        end
    end
    total = sum(block_11) + sum(block_12) + sum(block_22) + sum(block_13) + sum(block_14) + sum(block_23) + sum(block_24) + sum(block_28);
    fprintf("For this plan it is necessary the following blocks.\n");
    fprintf("Total Number: %d\n",total);
    fprintf("Support Blocks: %d\n",support);
    fprintf("1x1 blocks: white: %d, red: %d, orange: %d, yellow: %d, green: %d, blue: %d, black: %d\n",block_11(1),block_11(2),block_11(3),block_11(4),block_11(5),block_11(6),block_11(7));
    fprintf("1x2 blocks: white: %d, red: %d, orange: %d, yellow: %d, green: %d, blue: %d, black: %d\n",block_12(1),block_12(2),block_12(3),block_12(4),block_12(5),block_12(6),block_12(7));
    fprintf("2x2 blocks: white: %d, red: %d, orange: %d, yellow: %d, green: %d, blue: %d, black: %d\n",block_22(1),block_22(2),block_22(3),block_22(4),block_22(5),block_22(6),block_22(7));
    fprintf("1x3 blocks: white: %d, red: %d, orange: %d, yellow: %d, green: %d, blue: %d, black: %d\n",block_13(1),block_13(2),block_13(3),block_13(4),block_13(5),block_13(6),block_13(7));
    fprintf("1x4 blocks: white: %d, red: %d, orange: %d, yellow: %d, green: %d, blue: %d, black: %d\n",block_14(1),block_14(2),block_14(3),block_14(4),block_14(5),block_14(6),block_14(7));
    fprintf("2x3 blocks: white: %d, red: %d, orange: %d, yellow: %d, green: %d, blue: %d, black: %d\n",block_23(1),block_23(2),block_23(3),block_23(4),block_23(5),block_23(6),block_23(7));
    fprintf("2x4 blocks: white: %d, red: %d, orange: %d, yellow: %d, green: %d, blue: %d, black: %d\n",block_24(1),block_24(2),block_24(3),block_24(4),block_24(5),block_24(6),block_24(7));
    fprintf("2x8 blocks: white: %d, red: %d, orange: %d, yellow: %d, green: %d, blue: %d, black: %d\n",block_28(1),block_28(2),block_28(3),block_28(4),block_28(5),block_28(6),block_28(7));    
end