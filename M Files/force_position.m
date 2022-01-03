function FP = force_position(col,row,force_type)
%Return the position of a specified force type for a specified block type
%this is used only to make the functions that calls this one more readable
%input: : row: number of rows of the block desired
%         col: number of collums of the block desired
%         force_type:friction force (ff), X axis normal force (fnx),
%                    Y axis normal force (fny) or Z axis normal force (fnz)
%output: FP = a matrix with 3 collums (x,y,z) of a force.
%        this depends on which force and block is passed as argument. For
%        the correct usage, it is necessary a carrefull analysis to
%        separete which force is which in the function that call this one

if(force_type == "fnx")
    force_index = 1; %force index in FP matrix
    sum_row=row*(1+row)/2; %number of terms (optimization)
    FP = zeros(sum_row,3); %preallocation (optimization)
    
    for i=0:row %first of the force pair
        for j=1:row %second of the force pair
            if(j <= i) %if the second force is behind the first one
                continue;
            end
            %force position, already with the origin correction
            f1 = 4*[col/2,i-row/2];
            f2 = 4*[col/2,j-row/2];
            
            %Adding z axis value to f
            FP(force_index:force_index+3,:) = [[f1,-1.5];[f2,-1.5];[f1,1.5];[f2,1.5]];
            force_index = force_index + 4;
        end
    end  
elseif(force_type == "fny")
    force_index = 1; %force index in FP matrix
    sum_col=col*(1+col)/2; %number of terms (optimization)
    FP = zeros(sum_col,3); %preallocation (optimization)
    
    for i=0:col %first of the force pair
        for j=1:col %second of the force pair
            if(j <= i) %if the second force is behind the first one
                continue;
            end
            %force position, already with the origin correction
            f1 = 4*[i-col/2,row/2];
            f2 = 4*[j-col/2,row/2];
            
            %Adding z axis value to f
            FP(force_index:force_index+3,:) = [[f1,-1.5];[f2,-1.5];[f1,1.5];[f2,1.5]];
            force_index = force_index + 4;
        end
    end
elseif(force_type == "fnz")
    force_index = 1; %force index in FP matrix
    sum_row=row*(1+row)/2; %number of terms in Y axis (optimization)
    sum_col=col*(1+col)/2; %number of terms in X axis (optimization)
    FP = zeros(sum_row*sum_col,3); %preallocation (optimization)

    for i=1:col %increases the block size in the X axis
        for j=1:row %increases the block size in the Y axis
            for l=0:col %increases the starting point in the X axis
                if(i+l > col) %if this block does not fit in the original block, break
                    break;
                end
                for k=0:row %increases the starting point in the Y axis
                    if(k+j > row) %if this block does not fit in the original block, break
                        break;
                    end
                    %force position
                    f1 = [l,k];
                    f2 = [l,k+j];
                    f3 = [l+i,k];
                    f4 = [l+i,j+k];

                    %origin correction
                    f1 = 4*[f1(1)-col/2,f1(2)-row/2];
                    f2 = 4*[f2(1)-col/2,f2(2)-row/2];
                    f3 = 4*[f3(1)-col/2,f3(2)-row/2];
                    f4 = 4*[f4(1)-col/2,f4(2)-row/2];

                    %Adding z axis value to f
                    FP(force_index:force_index+3,:) = [[f1,1.5];[f2,1.5];[f3,1.5];[f4,1.5]];
                    force_index = force_index + 4;
                end
            end
        end
    end
elseif(force_type == "ff")
    %Need to think about this case
else
    FP = [];
end