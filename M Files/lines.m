function max = lines(model)
%Find the height of the block model
%input: model:(BlockNo., x, y, z, type, color)
%output: max:(maximum z value in the model)

loop = size(model);
max = 1;
for n = 1 : loop(1)
    if(model(n, 4) > max)
        max = model(n, 4);
    end
end