function [submodel1,submodel2] = Separate_Subassemblies(model,division_layer) 
%
%input: model:
%       division_layer: last layer of the first subassembly
    
    for i = 1:size(model,1)
        if(model(i,4) > division_layer)
            break;
        end
    end
    
    submodel1 = model(1:i-1,:);
    submodel2 = model(i:end,:);
    %Clean subassembly1 (take out the non necessary support blocks)
    i = max(submodel1(:,1));
    while(i>=1)
        if(submodel1(i,6) == 98)
            submodel1(i,:) = [];
            submodel1(:,1) = 1:size(submodel1);
        end
        i = i - 1;
    end
   
    %Prepare subassembly2 to be iterated in New_Planner function (change Z values)
    submodel2(:,4) = submodel2(:,4) - division_layer;
    submodel2(:,1) = 1:size(submodel2);
    
end