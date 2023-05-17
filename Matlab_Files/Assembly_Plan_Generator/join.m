function z = join(knobs, line)
%Find the pair of knobs that are snapped together.
%input: knobs: (x,y,z,block_type,color,block_number,knob_index)
%       line = (layer number)
%output: z = (*,upper_block_type,upper_block_number,upper_knob_index,lower_block_type,lower_block_number,lower_knob_index)
%        *: Number of other pair of snapped together knobs that are from the same block as the first pair
%OBS: The order in the output vector is in regard to the upper block numeration

loop = size(knobs,1);
lower = zeros(loop, 7);
upper = zeros(loop, 7);
upper_line = line;
lower_line = line - 1;
count_l = 1;  count_u = 1;
for n = 1 : loop % Count and save which knob is from the lower and which are from the upper layer
   if(knobs(n, 3) == lower_line) % If the nth knob is in the lower layer
      lower(count_l, 1:7) = knobs(n, 1:7); % Save this knob information in the lower matrix
      count_l = count_l + 1;
   elseif(knobs(n, 3) == upper_line) % If the nth knob is in the upper layer
       upper(count_u, 1:7) = knobs(n, 1:7); % Save this knob information in the upper matrix
       count_u = count_u + 1;
   elseif(knobs(n, 3) > upper_line) % If the line layer is over, break
       break
   end
end
 
count = 1;
join = zeros(count_u, 7); %(1,upper_block_type,upper_block_number,upper_knob_index,lower_block_type,lower_block_number,lower_knob_index)
for n = 1 : (count_l - 1) % For every upper knob found
    for m = 1 : (count_u - 1) % For every lower knob found
        if((lower(n, 1) == upper(m, 1)) && (lower(n, 2) == upper(m, 2))) %if x and y axis value from the upper knob is equal from the lower knob
            join(count,  1:7) = [1, upper(m, 4), upper(m, 6:7), lower(n, 4), lower(n, 6:7)];
            count = count + 1;
            break
        end
    end
end
 
y = join(1:(count - 1), 1:7);
z = join(1:(count - 1), 1:7);
c = 1;
max_knob_number = 81; % (9x9 block have 81 knobs)
block_number = zeros(max_knob_number, 1); % This vector stores all knobs connections that is also from the same block pair as the current one
 
for n = 1 : (count-1) % For each line of y matrix
   if(y(n, 1) == 1)
       block_number(1) = n;
       same_block = 1;
       for m = n + 1 : (count - 1) % Search which knob pairs are from the same block as the y(n,:)
          if((y(n, 3) == y(m, 3)) && (y(n, 6) == y(m, 6))) % If both the lower block and upper block are the same for n and m
              same_block = same_block + 1;
              block_number(same_block) = m;
          end
       end
       for l = 1 : same_block
           y(block_number(l), 1) = same_block; % Save in the matrix how many knob pairs share the same block pair
           z(c,:) = y(block_number(l),:); % Organize the vector
           c = c + 1;
       end
   end
end