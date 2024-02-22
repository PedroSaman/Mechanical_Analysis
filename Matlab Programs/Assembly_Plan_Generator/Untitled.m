s = size(a,1);
count = s;
while(count>0)
    if(a(count,2)==4)
        if(a(count,3) == 26)
            if(a(count,9)==1)
                a(count,:) = [];
            end
        end
    end
    count = count - 1;
end
