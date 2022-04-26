function y = putcolor(model)
%Put up colors to the model loaded, this function is manual used to make
%the blocks in the future simulations more recognizable. Also add to the
%model matrix a column with the block number.
%input: model:(x, y, z, block_type)
%output: y:(BlockNo., x, y, z, type, color)

y = model;
k = size(y);
number = 1:1:k(1); 
for n = 1 : k(1)
    if(y(n, 4) == 22)
        y(n, 5) = 1;
    elseif(y(n, 4) == 12)
        y(n, 5) = 1;
    elseif(y(n, 4) == 21)
        y(n, 5) = 1;
    else
        y(n, 5) = 1;
    end
end
y = [number.' y];