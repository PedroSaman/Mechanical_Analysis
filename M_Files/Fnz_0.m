function Fnz = Fnz_0(model)
%Compute the first layer blocks Z axis normal forces position
%input: model:(Block_Number, x, y, z, type, color)
%output: Fnz:(Force_Number, Block_Number, 0, x, y, z, 1, 0, 0, 0, 0, 0, 0)

loop = size(model,1);
count = 0;
All_Forces = [];
bc = zeros(9,9);
for n = 1 : loop
   if(model(n, 4) == 1)
       count = count + 1; % Count the number of blocks in the first stage
       [col,row] = col_row_converter(model(n, 5)); % Block number of columns and rows
       if(bc(col,row) == 0) % If this is the 1st time this block type appears
           bc(col,row) = 1;
           Pf = force_position(col,row,"fnz");
           Pnz = Pf(end-3:end,1:3); % Get only the last 4 forces from Pf
           Pnz_neg = [Pnz(1:4, 1:2), -Pnz(1:4, 3)];
           All_Forces = [All_Forces;Pnz_neg,[10*col+row;zeros(3,1)]];
       end
   else
       break;
   end
end

Fnz = zeros(count*4, 13);
Fnz(1:count*4, 1) = 1: count*4; % Force Number
Fnz(1:count*4, 7) = 1;
force = 1;
for n = 1 : count % For each block in the 1st layer
    Fnz(force:(force+3), 2) = model(n, 1); % Block number
    i = 1;
    while(i < size(All_Forces,1)) % Iterate the All_Forces matrix to find the current block force coordinate information
        if(All_Forces(i,4) == model(n, 5))
            Fnz(force:(force+3), 4:6) = All_Forces(i:i+3,1:3);
            break;
        end
        i = i + 4; % Iterate to the next force in All_Forces matrix
    end
    force = force + 4; % Every block have 4 normal forces in the z axis
end