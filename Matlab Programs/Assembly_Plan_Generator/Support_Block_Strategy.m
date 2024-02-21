function [plan,inserted_block_number] = Support_Block_Strategy(model,block_number)
% Implement the support block strategy. This function receives the block
% that is being inserted, then call the function to search the possible
% blocks to have support blocks inserted, then decides in which available 
% knob of each block the support pilar will be inserted and finaly calls the
% function that insert in the plan the support bricks
%
% input: model:(block_number ,x, y, z, block_type, color) is the planning until now
%       block number: is the block being inserted
% output: plan: (block_number ,x, y, z, block_type, color). Is the plan updated 
%        inserted_block_number: how many support blocks was added. -1 is error 
    
    knobs = knob(model,0); % Knob information (entire block model)
    insert_block = block_number;
    [knob_map,block_number] = Search_Blocks(knobs,model,block_number); % Search for blocks to add support
    
    if(isempty(block_number)) %Impossible to insert support blocks
        inserted_block_number = -1; %Return error
        plan = model;
        return;
    end
    
    possible_blocks = size(block_number,1);
    support_coordinates = []; % (X,Y,Z,BlockNumber)
    
    j = possible_blocks;
    %For each possible support pillar position 
    while(j >= 1) 
        [col,row] = col_row_converter(model(block_number(j),5));
        
        tentative_positions = zeros(size(knob_map,2),4);
        tentative_positions(:,4) = block_number(j);
        tentative_positions(:,3) = model(block_number(j),4) - 1;
        
        k = size(knob_map,2);
        while(k >= 1)
            if(knob_map(j,k) == -1 || knob_map(j,k) == 1) %If this knob is not free, or do not exists.
                tentative_positions(k,:) = []; %Erase this tentative position.
            elseif(knob_map(j,k) == 0) %This knob is free.
                support_knob = k;
                
                %Convertion from block knob number to cartesian position.
                if(col >= row)
                    if(row > 1)
                        if(mod(support_knob,2) == 0)
                            tentative_positions(k,2) = model(block_number(j),3) + 1;
                        else
                            tentative_positions(k,2) = model(block_number(j),3);
                        end
                    else
                        tentative_positions(k,2) = model(block_number(j),3);
                    end
                    tentative_positions(k,1) = model(block_number(j),2) + ceil(support_knob/row) - 1;
                else
                    if(row > col)
                        if(support_knob/row <= 1)
                            tentative_positions(k,1) = model(block_number(j),2);
                        else
                            tentative_positions(k,1) = model(block_number(j),2) + 1;
                        end
                        if(support_knob > row)
                            tentative_positions(k,2) = model(block_number(j),3) + support_knob - row - 1;
                        else
                            tentative_positions(k,2) = model(block_number(j),3) + support_knob - 1;
                        end
                    end
                end

            end
            k = k - 1;
        end
        support_coordinates = [tentative_positions;support_coordinates]; % Concatenate the tentative with the support coordinates.
        j = j - 1;
    end

    [plan,inserted_block_number,pillars_inserted] = insert_support_blocks(model,support_coordinates,insert_block); % Input is a matrix with each row being a cartesian position of each pilar.
    [plan,removed_blocks_number] = Remove_Redundancies(plan,insert_block+inserted_block_number,pillars_inserted); % Remove pillars not necessary.
    inserted_block_number = inserted_block_number - removed_blocks_number; % Update the inserted block number with the removed pillars.
end