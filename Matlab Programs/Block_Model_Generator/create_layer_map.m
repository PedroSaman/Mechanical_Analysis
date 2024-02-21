function [layer_map, voxel_model_layer] = create_layer_map(voxel_model_layer)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    layer_map = -ones(max(voxel_model_layer(:,2)),max(voxel_model_layer(:,1)));
    number_of_rows = size(layer_map,1);
    voxel_vector_size = size(voxel_model_layer,1);
    
    for i = 1 : voxel_vector_size
        layer_map(voxel_model_layer(i,2),voxel_model_layer(i,1)) = 0;
        voxel_model_layer(i,2) = number_of_rows - voxel_model_layer(i,2) + 1;
    end
    
    % Correct the layer map to make visual sense. This is corrected in the
    % end to be correct in regard to the mechanical analysis
    for i = 1 : floor(number_of_rows/2)
        aux = layer_map(i,:);
        layer_map(i,:) = layer_map(number_of_rows-i+1,:);
        layer_map(number_of_rows-i+1,:) = aux;
    end
    
end