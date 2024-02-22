function output = voxels_priority_list(voxel_model_layer)
%UNTITLED4 Summary of this function goes here
%   Develop an algorithm that searches for all voxels in the edge of the
%   voxel_model_layer and change the 3rd column to 0 if on edge and -1 if
%   not (maybe differenciate the type of edge voxel (to which side the
%   edge is?)
%   output: for each row 1: occupied, 0: empty 
%                                       1th column: voxel X value
%                                       2th column: voxel Y value
%                                       3th column: voxel Z value
%                                       4th column: voxel to the right
%                                       5th column: voxel to the left
%                                       6th column: voxel below
%                                       7th column: voxel above
%                                       8th column: number of voxels around
    
    voxel_vector_size = size(voxel_model_layer,1);
    output = [voxel_model_layer,zeros(voxel_vector_size,5)];
    
    %Count how many voxels each voxel has around it from 0 to 4
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
    
    %Create a matrix for each set of voxel (0 to 4 neighbooring voxels) and
    %then randomize the voxels position inside it. This is to avoid
    %creating block layers with stacks of blocks without other connections.
    count = ones(5,1);
    output0 = zeros(size(output,1),8);
    output1 = zeros(size(output,1),8);
    output2 = zeros(size(output,1),8);
    output3 = zeros(size(output,1),8);
    output4 = zeros(size(output,1),8);
    for i = 1:size(output,1)
        if(output(i,8)==0)
            output0(count(1),:) = output(i,:);
            count(1) = count(1) + 1;
        elseif(output(i,8)==1)
            output1(count(2),:) = output(i,:);
            count(2) = count(2) + 1;
        elseif(output(i,8)==2)
            output2(count(3),:) = output(i,:);
            count(3) = count(3) + 1;
        elseif(output(i,8)==3)
            output3(count(4),:) = output(i,:);
            count(4) = count(4) + 1;
        elseif(output(i,8)==4)
            output4(count(5),:) = output(i,:);
            count(5) = count(5) + 1;
        end
    end
    
    
    output0(count(1):end,:) = [];
    output1(count(2):end,:) = [];
    output2(count(3):end,:) = [];
    output3(count(4):end,:) = [];
    output4(count(5):end,:) = [];
    R0 = randperm(size(output0,1));
    R1 = randperm(size(output1,1));
    R2 = randperm(size(output2,1));
    R3 = randperm(size(output3,1));
    R4 = randperm(size(output4,1));
    output0 = output0(R0,:);
    output1 = output1(R1,:);
    output2 = output2(R2,:);
    output3 = output3(R3,:);
    output4 = output4(R4,:);
    
    %as one more measure to avoid stacks of blocks, every 3 layers give
    %priority to voxels 0->1->4->3->2. 
    %if(output(1,3) == 3 || output(1,3) == 6) %removed for testing
    if(0)
        output = [output0;output1;output4;output3;output2];
    else
        output = [output0;output1;output2;output3;output4];
    end
   output(:,4:8) = [];
end