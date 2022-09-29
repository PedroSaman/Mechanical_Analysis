function FP = force_position(col,row,force_type)
%Return the position of a specified force type for a specified block type.
%input: : row: number of rows of the desired block
%         col: number of columns of the desired block
%         force_type: friction force (ff), X axis normal force (fnx),
%                     Y axis normal force (fny) or Z axis normal force (fnz)
%                     friction force complete (ffc).
%output: FP = a matrix with 4 columns with the coordinate (x,y,z) of a force
%        and a column with relevant information. More details about the output
%        in each if below.
 
if(force_type == "fnx")
    %When two blocks as touching through the YZ face, it appear 4 forces
    %pointing in the X axis. One in each extremity of the faces in contact.
    %It is necessary to search for the two that have different Y value and
    %then duplicate it changing the Z value to find the other 2.
    
    force_index = 1; % Force index in FP matrix
    sum_row=row*(1+row)/2; % Number of terms
    FP = zeros(sum_row,4);
    for i=0:row % First of the force pair
        for j=1:row % Second of the force pair
            if(j <= i) % If the second force is behind the first one
                continue;
            end
            %Force position with the origin corrected
            f1 = 4*[col/2,i-row/2];
            f2 = 4*[col/2,j-row/2];
            
            %Adding z axis value to f
            FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+j];[f1,1.5,0];[f2,1.5,0]];
            force_index = force_index + 4;
        end
    end  
elseif(force_type == "fny")
    %When two blocks as touching through the XZ face, it appear 4 forces
    %pointing in the Y axis. One in each extremity of the faces in contact.
    %It is necessary to search for the two that have different X value and
    %then duplicate it changing the Z value to find the other 2.
    
    force_index = 1; % Force index in FP matrix
    sum_col=col*(1+col)/2; % Number of terms
    FP = zeros(sum_col,4);
    for i=0:col % First of the force pair
        for j=1:col % Second of the force pair
            if(j <= i) % If the second force is behind the first one
                continue;
            end
            % Force position with the origin corrected
            f1 = 4*[i-col/2,row/2];
            f2 = 4*[j-col/2,row/2];
            
            %Adding z axis value to f
            FP(force_index:force_index+3,:) = [[f1,-1.5,10*col+row];[f2,-1.5,10*(i+1)+j];[f1,1.5,0];[f2,1.5,0]];
            force_index = force_index + 4;
        end
    end
elseif(force_type == "fnz")
    %When two blocks as touching through the XY face, it appear 4 forces
    %pointing in the Z axis. One in each extremity of the faces in contact.
    %It is necessary to search for all 4 at the same time.
    
    force_index = 1; % Force index in FP matrix
    sum_row=row*(1+row)/2; % Number of terms in Y axis
    sum_col=col*(1+col)/2; % Number of terms in X axis
    FP = zeros(2*sum_row*sum_col,4);
    for i=1:col % Increases the block size in the X axis
        for j=1:row % Increases the block size in the Y axis
            for l=0:col % Increases the starting point in the X axis
                if(i+l > col) % If this block does not fit in the original block, break
                    break;
                end
                for k=0:row % Increases the starting point in the Y axis
                    if(k+j > row) % If this block does not fit in the original block, break
                        break;
                    end
                    
                    %Forces position
                    f1 = [l,k];
                    f2 = [l,k+j];
                    f3 = [l+i,k];
                    f4 = [l+i,j+k];
 
                    % Origin correction
                    f1 = 4*[f1(1)-col/2,f1(2)-row/2];
                    f2 = 4*[f2(1)-col/2,f2(2)-row/2];
                    f3 = 4*[f3(1)-col/2,f3(2)-row/2];
                    f4 = 4*[f4(1)-col/2,f4(2)-row/2];
 
                    % Adding z axis value to f
                    FP(force_index:force_index+3,:) = [[f1,1.5,10*col+row];[f2,1.5,10*(l+1)+i+l];[f3,1.5,10*(k+1)+j+k];[f4,1.5,0]];
                    force_index = force_index + 4;
                end
            end
        end
    end
elseif(force_type == "ff") % Friction force (excluding some forces)
    r = 1.25; % Knob radius
    b = 2; % Half block size
    pf_index = 1; % Force index in pf matrix
    z = -0.5;
    
    %That force vector Pf have the forces excluding the ones that doesn’t 
    %exist in the block (only the ones that actually appears in the knobs)
    if(row == 1 && col == 1)
        Pf = zeros(4,4);
    elseif(row>=col)
        Pf = zeros((2*3+(row-2)*2)*col,4);
    elseif(col>row)
        Pf = zeros((2*3+(col-2)*2)*row,4);
    end
    
    for i=0:(col-1) % For every column
        for j=0:(row-1) % For every row
            pos = 4*[i-col/2,j-row/2]; % Correct the origin
            f1 = [pos(1)+b-r,pos(2)+b]; % Knob friction force number one
            f2 = [pos(1)+b,pos(2)+b-r]; % Knob friction force number two
            f3 = [pos(1)+b,pos(2)+b+r]; % Knob friction force number three
            f4 = [pos(1)+b+r,pos(2)+b]; % Knob friction force number four
            
            %The output: first 3 columns in each row is the coordinates of
            %a force, the forth column is a special information relevant
            %to each force: 1st column = block type, 2nd column = knob
            %number, 3rd column = how many forces exists in this knob, 4th
            %column = which force was excluded.
            
            if(row==1 && col==1) % Special case for 1x1 blocks
                Pf(pf_index:pf_index+3,:) = [[f1,z,11];[f2,z,11];[f3,z,4];[f4,z,0]];
                pf_index = pf_index + 4;
            elseif(row>=col) % If the block is taller than wide
                if(j==0) % If columns first knob
                    Pf(pf_index:pf_index+3,:) = [[f1,z,10*col+row];[f2,z,10*(i+1)+(j+1)];[f4,z,3];[0,0,0,3]];
                    pf_index = pf_index + 4;
                elseif(j==row-1) % If columns last knob
                    Pf(pf_index:pf_index+3,:) = [[f1,z,10*col+row];[f3,z,10*(i+1)+(j+1)];[f4,z,3];[0,0,0,2]];
                    pf_index = pf_index + 4;
                else % If columns inside knob
                    Pf(pf_index:pf_index+3,:) = [[f1,z,10*col+row];[f4,z,10*(i+1)+(j+1)];[0,0,0,2];[0,0,2,3]];
                    pf_index = pf_index + 4;
                end
            elseif(col>row) % If the block is wider than tall
                if(i == 0) % If rows first knob
                    Pf(pf_index:pf_index+3,:) = [[f1,z,10*col+row];[f2,z,10*(i+1)+(j+1)];[f3,z,3];[0,0,0,4]];
                    pf_index = pf_index + 4;
                elseif(i == col-1) % If rows last knob
                    Pf(pf_index:pf_index+3,:) = [[f2,z,10*col+row];[f3,z,10*(i+1)+(j+1)];[f4,z,3];[0,0,0,1]];
                    pf_index = pf_index + 4;
                else % If rows inside knob
                    Pf(pf_index:pf_index+3,:) = [[f2,z,10*col+row];[f3,z,10*(i+1)+(j+1)];[0,0,0,2];[0,0,1,4]];
                    pf_index = pf_index + 4;
                end
            end
        end
    end
    FP = Pf;
elseif(force_type == "ffc") % Complete friction force matrix
    r = 1.25; % Knob radius
    b = 2; % Half block size
    force_index = 1; % Force index in FP matrix
    z = 2.5;
    
    %These force vector Pf are have the forces excluding the ones that
    %doesnt exist in the block.
    FP = zeros(4*row*col,4);
    
    for i=0:(col-1) % For every column
        for j=0:(row-1) % For every row
            pos = 4*[i-col/2,j-row/2]; % Correct the origin
            f1 = [pos(1)+b-r,pos(2)+b]; % Knob friction force number one
            f2 = [pos(1)+b,pos(2)+b-r]; % Knob friction force number two
            f3 = [pos(1)+b,pos(2)+b+r]; % Knob friction force number three
            f4 = [pos(1)+b+r,pos(2)+b]; % Knob friction force number four
            
            %The output: first 3 columns in each row is the coordinates of
            %a force, the forth column is a special information relevant
            %to each force: 1st row = block type, 2nd row = knob number, 
            %3rd row = the number of a knob that would need to be excluded,
            %4th row = the number of a knob that would need to be excluded.
            
            if(row==1 && col==1) % Special case for 1x1 blocks
                FP(force_index:force_index+3,:) = [[f1,z,10*col+row];[f2,z,10*(i+1)+(j+1)];[f3,z,0];[f4,z,0]];
            elseif(row>=col) % If the block is taller than wide
                if(j==0) % If columns first knob
                    FP(force_index:force_index+3,:) = [[f1,z,10*col+row];[f2,z,10*(i+1)+(j+1)];[f3,z,3];[f4,z,0]];
                elseif(j==row-1) % If columns last knob
                    FP(force_index:force_index+3,:) = [[f1,z,10*col+row];[f2,z,10*(i+1)+(j+1)];[f3,z,2];[f4,z,0]];
                else % If columns inside knob
                    FP(force_index:force_index+3,:) = [[f1,z,10*col+row];[f2,z,10*(i+1)+(j+1)];[f3,z,2];[f4,z,3]];
                end
            elseif(col>row) % If the block is wider than tall
                if(i == 0) % If rows first knob
                    FP(force_index:force_index+3,:) = [[f1,z,10*col+row];[f2,z,10*(i+1)+(j+1)];[f3,z,4];[f4,z,0]];
                elseif(i == col-1) % If rows last knob
                    FP(force_index:force_index+3,:) = [[f1,z,10*col+row];[f2,z,10*(i+1)+(j+1)];[f3,z,1];[f4,z,0]];
                else % If rows inside knob
                    FP(force_index:force_index+3,:) = [[f1,z,10*col+row];[f2,z,10*(i+1)+(j+1)];[f3,z,1];[f4,z,4]];
                end
            end
            force_index = force_index + 4;
        end
    end
else
    FP = [];
end