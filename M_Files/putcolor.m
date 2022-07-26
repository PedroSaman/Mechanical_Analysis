function y = putcolor(model)
%Put up colors and enumerate the model blocks.
%input: model:(x, y, z, block_type)
%output: y:(BlockNo., x, y, z, type, color)

k = size(model,1);
y = zeros(k,6);
y(1:k,2:5) = model(1:k,1:4);
for n = 1 : k
    if(model(n, 4) == 11)
        y(n, 6) = 1;
    elseif(model(n, 4) == 12)
        y(n, 6) = 1;
    elseif(model(n, 4) == 21)
        y(n, 6) = 1;
    elseif(model(n, 4) == 22)
        y(n, 6) = 1;
    elseif(model(n, 4) == 13)
        y(n, 6) = 1;
    elseif(model(n, 4) == 31)
        y(n, 6) = 1;
    elseif(model(n, 4) == 14)
        y(n, 6) = 1;
    elseif(model(n, 4) == 41)
        y(n, 6) = 1;
    elseif(model(n, 4) == 24)
        y(n, 6) = 1;
    elseif(model(n, 4) == 42)
        y(n, 6) = 1;
    elseif(model(n, 4) == 28)
        y(n, 6) = 1;
    elseif(model(n, 4) == 82)
        y(n, 6) = 1;
    else
        y(n, 6) = 1;
    end
    y(n,1) = n;
end