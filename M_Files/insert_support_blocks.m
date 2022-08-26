function [plan,ins_blk_n] = insert_support_blocks(model,support_coordinate,insert_block)

    base_size = max([max(model(:,3)),max(model(:,2))]) + 8;
    z_max = model(end,4); % Height
    model_size = size(model,1);
    map = zeros(base_size,base_size,z_max);
    j = 1;
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
    plan = model;
    
    while(~isempty(support_coordinate))
        m = size(support_coordinate,1);
        blk_number = 0;
        while(m >=1)
            if(support_coordinate(m,4) ~= blk_number)
                X = support_coordinate(m,1);
                Y = support_coordinate(m,2);
                Z = support_coordinate(m,3);
                blk_number = support_coordinate(m,4);
                j=1;
                for i = 1 : Z
                    if(map(X,Y,i) == 0) %There is space to insert support block in the i layer
                        while(plan(j,4) <= i) %Search for the last block with Z = i in model. This is to insert this support block as the last block in the layer
                            j = j + 1;
                            if(j > size(plan,1))
                                break;
                            end
                        end
                        plan = [plan(1:j-1,:); [0,X,Y,i,11,-1]; plan(j:end,:)];
                        ins_blk_n = ins_blk_n + 1;
                    end
                end
                support_coordinate(m,:) = [];
                plan = [(1:size(plan,1))' , plan(1:end,2:end)];
                [planner_output] = Planner_Stability_Judge(plan(1:insert_block + ins_blk_n,:));
                if(strcmp(planner_output,'safe'))
                    return;
                end
            end
            m = m - 1;
        end
    end
end