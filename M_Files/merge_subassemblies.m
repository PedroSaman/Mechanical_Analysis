function merged_model = merge_subassemblies(submodel1,submodel2)
%myFun - Description
%
% Syntax: output = myFun(input)
%
% Long description
    merged_model = [];

    new_size = size(submodel1,1) + size(submodel2,1);
    
    displacement = max(submodel1(:,2)) + 2;
    
    submodel2(:,2) = submodel2(:,2) + displacement;
    
    size_sub1 = submodel1(end,1);
    size_sub2 = submodel2(end,1);
    max_z2 = submodel2(end,4);
    
    sub1_i = 1;
    sub2_i = 1;
    for i = 1:max_z2
        if(sub1_i <= size_sub1)
            while(submodel1(sub1_i,4) == i)
                merged_model = [merged_model; submodel1(sub1_i,:)];
                sub1_i = sub1_i + 1;
                if(sub1_i > size_sub1)
                    break;
                end
            end
        end
        while(submodel2(sub2_i,4) == i)
            merged_model = [merged_model; submodel2(sub2_i,:)];
            sub2_i = sub2_i + 1;
            if(sub2_i > size_sub2)
                break;
            end
        end
        
    end
    merged_model(:,1) = 1:size(merged_model);
end