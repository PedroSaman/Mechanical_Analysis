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

if(force_type == "ff" || force_type == "fnx" || force_type == "fny" || force_type == "fnz")
    %filler
end

if(force_type == "fnx") %Fnx
    
    k=1; %auxiliary variable (optimization)
    m=1; %auxiliary variable (optimization)
    sum_row=row*(1+row)/2; %number of terms (optimization)
    FPx = zeros(sum_row,3); %preallocation
    
    for i = (-row/2):(row/2 - 1)
        for j = (-row/2 + k):row/2
           f1 = [4*col/2, 4*i];
           f2 = [4*col/2, 4*j];
           FPx(m:m+3,:) = [[f1,-1.5];[f2,-1.5];[f1,1.5];[f2,1.5]];
           m = m + 4;
        end
        k = k + 1;
    end
    FP = [FPx;[-FPx(:,1),FPx(:,2),FPx(:,3)]];
    
elseif(force_type == "fny") %Fny
    
    k=1; %auxiliary variable (optimization)
    m=1; %auxiliary variable (optimization)
    sum_col=col*(1+col)/2; %number of terms (optimization)
    FPy = zeros(sum_col,3); %preallocation (optimization)
    
    for i = (-col/2):(col/2 - 1)
        for j = (-col/2 + k):col/2
           f1 = [4*i, 4*row/2];
           f2 = [4*j, 4*row/2];
           FPy(m:m+3,:) = [[f1,-1.5];[f2,-1.5];[f1,1.5];[f2,1.5]];
           m = m + 4;
        end
        k = k + 1;
    end
    FP = [FPy;[FPy(:,1),-FPy(:,2),FPy(:,3)]];

elseif(force_type == "fnz") %funciona, vou fazer igual para a logica do fnx e fny
    force_index = 1;
    sum_row=row*(1+row)/2;
    sum_col=col*(1+col)/2;
    FPz = zeros(sum_row*sum_col,3);

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

                    %Adding to f and z value
                    FPz(force_index:force_index+3,:) = [[f1,1.5];[f2,1.5];[f3,1.5];[f4,1.5]];
                    force_index = force_index + 4;
                end
            end
        end
    end
    FP = [FPz;[FPz(:,1),FPz(:,2),-FPz(:,3)]];
    
end