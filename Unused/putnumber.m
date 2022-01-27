function y = putnumber(model)
%ブロックモデルデータに番号を与える
y = model;
k = size(y);
number = 1:1:k(1); 
y = [number.' y];