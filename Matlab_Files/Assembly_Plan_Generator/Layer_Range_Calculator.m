function [range] = Layer_Range_Calculator(model)
% Given the model in the input, determines the first and last blocks of
% each layer.
%
% input: model:(block_number ,x, y, z, block_type, color)
% output: range:[first_block_layer_n,last_block_layer_n]

    z_max = model(end,4); % Structure height
    model_size = size(model,1);
    range = zeros(z_max,2);
    initial_index = 1;
    j = 1;
    for i = 1:z_max % For every layer
        while(model(j,4) == i) % While same layer
            if(j == model_size)
                j = j + 1;
                break;
            end
            j = j + 1;
        end
        final_index = j - 1; % Correct count
        range(i,:) = [initial_index,final_index]; 
        initial_index = final_index + 1; % The next layer first block will be the next block in model.
    end
end