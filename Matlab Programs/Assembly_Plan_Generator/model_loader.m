function block_model = model_loader(file_path)
% Given the desired csv filename and path file, load the data in the 
% correct format.
% Input: file_path: text file name with its directory. Must be in Kohama
% Voxel Converter output format.
% Output: block_model:(Block_Number, x, y, z, type, color). All blocks in
% the block model.
    
    model_original = load(file_path);
    k = size(model_original,1);
    block_model(:,1) = 1:k;
    block_model(:,2:4) = model_original(:,1:3) + 1;
    block_model(:,5) = model_original(:,4)*10 + model_original(:,5);
    block_model(:,6) = model_original(:,7);
    block_model(:,7) = 1;
end