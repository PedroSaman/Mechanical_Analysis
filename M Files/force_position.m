function FP = force_position(col,row,force_type)
%Return the position of a specified force type for a specified block type.
%This function should work for every n by 2, 2 by m, 1 by n, m by 1 or 
%1 by 1 block.
%It is used only to make the functions that calls this one more readable.
%input: : row: number of rows of the desired block
%         col: number of columns of the desired block
%         force_type: friction force (ff), X axis normal force (fnx),
%                     Y axis normal force (fny) or Z axis normal force (fnz)
%output: FP = a matrix with 3 columns with the coordiante (x,y,z) of a force.
%        this depends on which force and block is passed as argument. For
%        the correct usage, it is necessary a carrefull analysis to
%        separete which force is which in the function that call this one

if(force_type == "fnx")
    force_index = 1; %force index in FP matrix
    sum_row=row*(1+row)/2; %number of terms (optimization)
    FP = zeros(sum_row,4); %preallocation (optimization)
    
    for i=0:row %first of the force pair
        for j=1:row %second of the force pair
            if(j <= i) %if the second force is behind the first one
                continue;
            end
            %force position with the origin corrected
            f1 = 4*[col/2,i-row/2];
            f2 = 4*[col/2,j-row/2];
            
            %Adding z axis value to f
            FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+j];[f1,1.5,0];[f2,1.5,0]];
            force_index = force_index + 4;
        end
    end  
elseif(force_type == "fny")
    force_index = 1; %force index in FP matrix
    sum_col=col*(1+col)/2; %number of terms (optimization)
    FP = zeros(sum_col,4); %preallocation (optimization)
    
    for i=0:col %first of the force pair
        for j=1:col %second of the force pair
            if(j <= i) %if the second force is behind the first one
                continue;
            end
            %force position with the origin corrected
            f1 = 4*[i-col/2,row/2];
            f2 = 4*[j-col/2,row/2];
            
            %Adding z axis value to f [10*col+row;10*i+j;0;0]
            FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+j];[f1,1.5,0];[f2,1.5,0]];
            force_index = force_index + 4;
        end
    end
elseif(force_type == "fnz")
    force_index = 1; %force index in FP matrix
    sum_row=row*(1+row)/2; %number of terms in Y axis (optimization)
    sum_col=col*(1+col)/2; %number of terms in X axis (optimization)
    FP = zeros(2*sum_row*sum_col,4); %preallocation (optimization)

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
                    FP(force_index:force_index+3,:) = [[f1,1.5,10*col+row];[f2,1.5,10*(l+1)+i+l];[f3,1.5,10*(k+1)+j+k];[f4,1.5,0]];
                    force_index = force_index + 4;
                end
            end
        end
    end
elseif(force_type == "ff")
    r = 1.25; %knob radius
    b = 2; %half block size
    force_index = 1; %force index in FP matrix
    pf_index = 1; %force index in pf matrix
    
    %These force vector Pf are have the forces excluding the ones that
    %doesnt exist in the block.
    if(row == 1 && col == 1)
        Pf = zeros(4,3);
    elseif(row>=col)
        Pf = zeros((2*3+(row-2)*2)*col,3);
    elseif(col>row)
        Pf = zeros((2*3+(col-2)*2)*row,3);
    end
    FP = zeros(4*row*col + size(Pf,1) ,3);
    
    for i=0:(col-1) %for every column
        for j=0:(row-1) %for every row
            pos = 4*[i-col/2,j-row/2]; %correct the origin
            f1 = [pos(1)+b-r,pos(2)+b]; %knob friction force number one
            f2 = [pos(1)+b,pos(2)+b-r]; %knob friction force number two
            f3 = [pos(1)+b,pos(2)+b+r]; %knob friction force number three
            f4 = [pos(1)+b+r,pos(2)+b]; %knob friction force number four
            FP(force_index:force_index+3,:) = [[f1,1.5];[f2,1.5];[f3,1.5];[f4,1.5]];
            force_index = force_index + 4;
            
            if(row==1 && col==1) %Special case for 1x1 blocks
                Pf(pf_index:pf_index+3,:) = [[f1,-1.5];[f2,-1.5];[f3,-1.5];[f4,-1.5]];
                pf_index = pf_index + 4;
            elseif(row>=col) %if the block is taller than wide
                if(j==0) %if columns first knob
                    Pf(pf_index:pf_index+2,:) = [[f1,-1.5];[f2,-1.5];[f4,-1.5]];
                    pf_index = pf_index + 3;
                elseif(j==row-1) %if columns last knob
                    Pf(pf_index:pf_index+2,:) = [[f1,-1.5];[f3,-1.5];[f4,-1.5]];
                    pf_index = pf_index + 3;
                else %if columns inside knob
                    Pf(pf_index:pf_index+1,:) = [[f1,-1.5];[f4,-1.5]];
                    pf_index = pf_index + 2;
                end
            elseif(col>row) %if the block is wider than tall
                if(i == 0) %if rows first knob
                    Pf(pf_index:pf_index+2,:) = [[f1,-1.5];[f2,-1.5];[f3,-1.5]];
                    pf_index = pf_index + 3;
                elseif(i == col-1) %if rows last knob
                    Pf(pf_index:pf_index+2,:) = [[f2,-1.5];[f3,-1.5];[f4,-1.5]];
                    pf_index = pf_index + 3;
                else %if rows inside knob
                    Pf(pf_index:pf_index+1,:) = [[f2,-1.5];[f3,-1.5]];
                    pf_index = pf_index + 2;
                end
            end
        end
    end
    FP(force_index:end,:) = Pf;
else
    FP = [];
end