function [range] = Layer_Range_Calculator(model)
% Given the model in the input, determines the first and last blocks of
% each layer.
%
% input: model:(block_number ,x, y, z, block_type, color)
% output: range:[first_block_layer_n,last_block_layer_n,layer_volume]

    z_max = model(end,4); % Structure height
    model_size = size(model,1);
    range = zeros(z_max,3);
    initial_index = 1;
    j = 1;
    for i = 1:z_max % For every layer
        volume = 0;
        while(model(j,4) == i) % While same layer
            x = floor(model(j,5)/10);
            y = model(j,5) - 10*x;
            volume = volume + x*y;
            if(j == model_size)
                j = j + 1;
                break;
            end
            j = j + 1;
        end
        final_index = j - 1; % Correct count
        range(i,:) = [initial_index,final_index,volume];
        initial_index = final_index + 1; % The next layer first block will be the next block in model.
    end
end