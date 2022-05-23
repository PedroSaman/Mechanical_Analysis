function check = model_check(model,M)

base_length = 300;
base_width = 300;
z_max = lines(model);
model_size = size(model,1);
j = 1;
check_size = 0;
map = zeros(base_length,base_width,z_max);
density_map = zeros(base_length,base_width);
for i = 1 : z_max
    while(model(j,4) == i)
        for m = 1 : size(M,1)
            if(model(j,5) == M(m,1))
               check_size = 1;
               break;
            end
        end
        if(check_size == 0) %This block type is not available in the lab
            %check = -1;
            %return;
        end
        [col,row] = col_row_converter(model(j,5));
        for k = model(j,2) : model(j,2)+col-1 %Column iterator
            for l = model(j,3) : model(j,3)+row-1 %Row iterator
                if(map(base_width-l+1,k,i) == 1)
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