function [plan,ins_blk_n,pilar_number] = insert_support_blocks(model,support_coordinate,insert_block)
    %Insert support pillars in the positions given by 'support_coordinate' while testing the stability via the stability judge.
    %It is needed a strategy to choose the 'support_coordinate' positions order. For now it is alternated the block to insert the pillar
    %from the ones possible.
    %input: model: current plan.
    %       support_coordinate:
    %       insert_block: being inserted block number.
    %output: plan: new assembly plan with support pillars inserted.
    %        ins_blk_n: number of blocks inserted.
    %        pilar_number: number of pillars inserted.

    %TODO: inconsistency when no number of support pillars make the
    %structure stable. Look into this
    
    base_size = max([max(model(:,3)),max(model(:,2))]) + 8; %Maximum value for X and Y + 8 (biggest size of a block)
    z_max = model(insert_block,4);
    model_size = insert_block;
    map = zeros(base_size,base_size,z_max);
    j = 1;
    %Create the input model map to verify where there are space to insert the support block
    for i = 1 : z_max
        while(model(j,4) == i)
            [col,row] = col_row_converter(model(j,5));
            for k = model(j,2) : model(j,2)+col-1 %Column iterator
                for l = model(j,3) : model(j,3)+row-1 %Row iterator
                    map(base_size-l+1,k,i) = model(j,1);
                end
            end
            j = j + 1;
            if(j > model_size)
               break;
            end
        end
    end
    
    ins_blk_n = 0;
    pilar_number = 0;
    plan = model;
    
    %While there is more support pillar possible coordinates
    while(~isempty(support_coordinate))
        m = size(support_coordinate,1);
        blk_number = 0;
        while(m >=1)
            if(support_coordinate(m,4) ~= blk_number)
                X = support_coordinate(m,1);
                Y = support_coordinate(m,2);
                Z = support_coordinate(m,3);
                blk_number = support_coordinate(m,4);
                pilar_number = pilar_number + 1;
                j=1;
                for i = 1 : Z
                    if(map(base_size-Y+1,X,i) == 0) %There is space to insert support block in the i layer
                        while(plan(j,4) <= i) %Search for the last block with Z = i in model. This is to insert this support block as the last block in the layer
                            j = j + 1;
                            if(j > size(plan,1))
                                break;
                            end
                        end
                        plan = [plan(1:j-1,:); [0,X,Y,i,11,-1,1]; plan(j:end,:)];
                        ins_blk_n = ins_blk_n + 1;
                    end
                end
                support_coordinate(m,:) = [];
                plan(:,1) = 1:size(plan,1);
                [planner_output] = Planner_Stability_Judge(plan(1:insert_block + ins_blk_n,:));
                if(strcmp(planner_output,'safe')||strcmp(planner_output,'100safe')) %If the insertion is stable, there is no need to check the rest
                    return;
                end
            end
            m = m - 1;
        end
    end
end