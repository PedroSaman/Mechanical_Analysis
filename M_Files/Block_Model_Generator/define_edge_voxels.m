function output = define_edge_voxels(voxel_model_layer)
%UNTITLED4 Summary of this function goes here
%   Develop an algorithm that searches for all voxels in the edge of the
%   voxel_model_layer and change the 3rd column to 0 if on edge and -1 if
%   not (maybe differenciate the type of edge voxel (to which side the
%   edge is?)
%   output: for each row 1: occupied, 0: empty 
%                                       4th column: voxel to the right
%                                       5th column: voxel to the left
%                                       6th column: voxel below
%                                       7th column: voxel above
%                                       8th column: number of voxels around
    
    voxel_vector_size = size(voxel_model_layer,1);
    output = [voxel_model_layer,zeros(voxel_vector_size,5)];
    
    for i = 1 : voxel_vector_size
        
        for j = 1 : voxel_vector_size
            if    ((voxel_model_layer(i,1) == voxel_model_layer(j,1) + 1) && (voxel_model_layer(i,2) == voxel_model_layer(j,2)))
                %there is something to the left
                output(i,4) = 1;
                output(i,8) = output(i,8) + 1;
            elseif((voxel_model_layer(i,1) == voxel_model_layer(j,1) - 1) && (voxel_model_layer(i,2) == voxel_model_layer(j,2)))
                %there is something to the right
                output(i,5) = 1;
                output(i,8) = output(i,8) + 1;
            elseif((voxel_model_layer(i,2) == voxel_model_layer(j,2) - 1) && (voxel_model_layer(i,1) == voxel_model_layer(j,1)))
                %there is something above
                output(i,6) = 1;
                output(i,8) = output(i,8) + 1;
            elseif((voxel_model_layer(i,2) == voxel_model_layer(j,2) + 1) && (voxel_model_layer(i,1) == voxel_model_layer(j,1)))
                %there is something below
                output(i,7) = 1;
                output(i,8) = output(i,8) + 1;
            end
        end
    end
    output = sortrows(output,8); %Sort to put first the voxels that has fewer other voxels around it
    output(:,4:8) = [];
end

