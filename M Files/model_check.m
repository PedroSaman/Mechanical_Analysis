function check = model_check(model,M)
%Verify if the input model has: unavailable blocks or if multiples blocks
%are occupying the same space
%input:  model:(block_number, x, y, z, Block_type, color)
%        M:(Mass of each available block)
%output: check: -1 if not available, -2 if any space is occupied by two or 
%               more blocks, 1 if no problems
%OBS: the base_length and base_width are necessary to create the matrix to
%map the blocks position. Those values can increase as much as it is
%desirable.

base_length = 300; % Max length of the base
base_width = 300; % Max width of the base
z_max = model(end,4); % Height
model_size = size(model,1); 
map = zeros(base_length,base_width,z_max);
density_map = zeros(base_length,base_width);
j = 1;
check_size = 0;
for i = 1 : z_max
    while(model(j,4) == i)
        for m = 1 : size(M,1)
            if(model(j,5) == M(m,1))
               check_size = 1;
               break;
            end
        end
        if(check_size == 0) %This block type is not available in the lab
            check = -1;
            return;
        end
        [col,row] = col_row_converter(model(j,5));
        for k = model(j,2) : model(j,2)+col-1 %Column iterator
            for l = model(j,3) : model(j,3)+row-1 %Row iterator
                if(map(base_width-l+1,k,i) == 1) %This Cartesian space already has a block
                    check = -2;
                    return;
                end
                map(base_width-l+1,k,i) = 1;
                density_map(base_width-l+1,k) = density_map(base_width-l+1,k) + 1;
            end
        end
        j = j + 1;
        if(j > model_size)
           break; 
        end
    end
end
check = 1;