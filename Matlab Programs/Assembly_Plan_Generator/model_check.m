function check = model_check(model)
%Verify if the input model has: unavailable blocks or if multiples blocks
%are occupying the same space
%input:  model:(block_number, x, y, z, Block_type, color)
%        M:(Mass of each available block)
%output: -2 if any space is occupied by two or 
%               more blocks, 1 if no problems
%OBS: the base_length and base_width are necessary to create the matrix to
%map the blocks position. Those values can increase as much as it is
%desirable.

base_size = max([max(model(:,3)),max(model(:,2))]) + 8;
z_max = model(end,4); % Height
model_size = size(model,1);
map = zeros(base_size,base_size,z_max);
density_map = zeros(base_size,base_size);
j = 1;
for i = 1 : z_max
    while(model(j,4) == i)
        [col,row] = col_row_converter(model(j,5));
        for k = model(j,2) : model(j,2)+col-1 %Column iterator
            for l = model(j,3) : model(j,3)+row-1 %Row iterator
                if(map(base_size-l+1,k,i) ~= 0) %This Cartesian space already has a block
                    check = -2;
                    return;
                end
                map(base_size-l+1,k,i) = model(j,1);
                density_map(base_size-l+1,k) = density_map(base_size-l+1,k) + 1;
            end
        end
        j = j + 1;
        if(j > model_size)
           break; 
        end
    end
end
check = 1;