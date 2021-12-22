function y = putcolor(model)
%�u���b�N���f���f�[�^�ɐF����^����
%22 => 1, 12 => 2, 21 => 3, 11 => 4
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