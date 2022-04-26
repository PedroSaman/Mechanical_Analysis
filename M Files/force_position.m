function FP = force_position(col,row,force_type)
%Return the position of a specified force type for a specified block type.
%This function should work for every n by 2, 2 by m, 1 by n, m by 1 or 
%1 by 1 block.
%It is used only to make the functions that calls this one more readable.
%input: : row: number of rows of the desired block
%         col: number of columns of the desired block
%         force_type: friction force (ff), X axis normal force (fnx),
%                     Y axis normal force (fny) or Z axis normal force (fnz)
%                     friction force complete (ffc).
%output: FP = a matrix with 4 columns with the coordiante (x,y,z) of a force
%        and a column with relevant information. More details about the output
%        in each if below.

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
elseif(force_type == "ff") %friction force (excluding the necessary forces)
    r = 1.25; %knob radius
    b = 2; %half block size
    pf_index = 1; %force index in pf matrix
    
    %That force vector Pf have the forces excluding the ones that doesnt 
    %exist in the block (only the ones that actually appears in the knobs)
    if(row == 1 && col == 1)
        Pf = zeros(4,4);
    elseif(row>=col)
        Pf = zeros((2*3+(row-2)*2)*col,4);
    elseif(col>row)
        Pf = zeros((2*3+(col-2)*2)*row,4);
    end
    
    for i=0:(col-1) %for every column
        for j=0:(row-1) %for every row
            pos = 4*[i-col/2,j-row/2]; %correct the origin
            f1 = [pos(1)+b-r,pos(2)+b]; %knob friction force number one
            f2 = [pos(1)+b,pos(2)+b-r]; %knob friction force number two
            f3 = [pos(1)+b,pos(2)+b+r]; %knob friction force number three
            f4 = [pos(1)+b+r,pos(2)+b]; %knob friction force number four
            
            %The output: first 3 collumns in each row is the coordinates of
            %a force, the forth column is a special information relevant
            %to each force: 1st column = block type, 2nd column = knob
            %number, 3rd column = how many forces exists in this knob, 4th
            %column = which force was excluded.
            if(row==1 && col==1) %Special case for 1x1 blocks
                Pf(pf_index:pf_index+3,:) = [[f1,1.5,11];[f2,1.5,11];[f3,1.5,4];[f4,1.5,0]];
                pf_index = pf_index + 4;
            elseif(row>=col) %if the block is taller than wide
                if(j==0) %if columns first knob
                    Pf(pf_index:pf_index+3,:) = [[f1,1.5,10*col+row];[f2,1.5,10*(i+1)+(j+1)];[f4,1.5,3];[0,0,0,3]];
                    pf_index = pf_index + 4;
                elseif(j==row-1) %if columns last knob
                    Pf(pf_index:pf_index+3,:) = [[f1,1.5,10*col+row];[f3,1.5,10*(i+1)+(j+1)];[f4,1.5,3];[0,0,0,2]];
                    pf_index = pf_index + 4;
                else %if columns inside knob
                    Pf(pf_index:pf_index+3,:) = [[f1,1.5,10*col+row];[f4,1.5,10*(i+1)+(j+1)];[0,0,0,2];[0,0,2,3]];
                    pf_index = pf_index + 4;
                end
            elseif(col>row) %if the block is wider than tall
                if(i == 0) %if rows first knob
                    Pf(pf_index:pf_index+3,:) = [[f1,1.5,10*col+row];[f2,1.5,10*(i+1)+(j+1)];[f3,1.5,3];[0,0,0,4]];
                    pf_index = pf_index + 4;
                elseif(i == col-1) %if rows last knob
                    Pf(pf_index:pf_index+3,:) = [[f2,1.5,10*col+row];[f3,1.5,10*(i+1)+(j+1)];[f4,1.5,3];[0,0,0,1]];
                    pf_index = pf_index + 4;
                else %if rows inside knob
                    Pf(pf_index:pf_index+3,:) = [[f2,1.5,10*col+row];[f3,1.5,10*(i+1)+(j+1)];[0,0,0,2];[0,0,1,4]];
                    pf_index = pf_index + 4;
                end
            end
        end
    end
    FP = Pf;
elseif(force_type == "ffc") %complete friction force matrix
    r = 1.25; %knob radius
    b = 2; %half block size
    force_index = 1; %force index in FP matrix
    
    %These force vector Pf are have the forces excluding the ones that
    %doesnt exist in the block.
    FP = zeros(4*row*col,4);
    
    for i=0:(col-1) %for every column
        for j=0:(row-1) %for every row
            pos = 4*[i-col/2,j-row/2]; %correct the origin
            f1 = [pos(1)+b-r,pos(2)+b]; %knob friction force number one
            f2 = [pos(1)+b,pos(2)+b-r]; %knob friction force number two
            f3 = [pos(1)+b,pos(2)+b+r]; %knob friction force number three
            f4 = [pos(1)+b+r,pos(2)+b]; %knob friction force number four
            
            %The output: first 3 collumns in each row is the coordinates of
            %a force, the forth column is a special information relevant
            %to each force: 1st row = block type, 2nd row = knob number, 
            %3rd row = the number of a knob that would need to be excluded,
            %4th row = the number of a knob that would need to be excluded.
            if(row==1 && col==1) %Special case for 1x1 blocks
                FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+(j+1)];[f3,-1.5,0];[f4,-1.5,0]];
            elseif(row>=col) %if the block is taller than wide
                if(j==0) %if columns first knob
                    FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+(j+1)];[f3,-1.5,3];[f4,-1.5,0]];
                elseif(j==row-1) %if columns last knob
                    FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+(j+1)];[f3,-1.5,2];[f4,-1.5,0]];
                else %if columns inside knob
                    FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+(j+1)];[f3,-1.5,2];[f4,-1.5,3]];
                end
            elseif(col>row) %if the block is wider than tall
                if(i == 0) %if rows first knob
                    FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+(j+1)];[f3,-1.5,4];[f4,-1.5,0]];
                elseif(i == col-1) %if rows last knob
                    FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+(j+1)];[f3,-1.5,1];[f4,-1.5,0]];
                else %if rows inside knob
                    FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+(j+1)];[f3,-1.5,1];[f4,-1.5,4]];
                end
            end
            force_index = force_index + 4;
        end
    end
else
    FP = [];
end