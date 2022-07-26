function [new_data] = block_model_loader(filename)

old_data = importdata(filename);
len = size(old_data,1);
new_data = zeros(len,4);

for i = 1:len
    new_data(i,1:3) = old_data(i,1:3) + 1;
    new_data(i,4) = 10*old_data(i,4) + old_data(i,5);
end