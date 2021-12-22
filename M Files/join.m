function y = join(knobs, line)
%Find which knobs are snaped togheter. The order in the output vector is in regard to the upper block numeration
%input: knobs: information of all knobs in the block model. (x,y,z,block_type,color,block_number,knob_index)
%       line = (number of the current layer)
%output: y = (*,upper_block_type,upper_block_number,upper_knob_index,lower_block_type,lower_block_number,lower_knob_index)
%        *: Number of other pair of snaped togheter knobs that are from the same block as the first pair

%count and save which knob is from the lower and the upper layer
loop = size(knobs);
lower = zeros(loop(1), 7);
upper = zeros(loop(1), 7);
upper_line = line;
lower_line = line - 1;
count_l = 1;  count_u = 1;
for n = 1 : loop(1)
   if(knobs(n, 3) == lower_line) %if the this knob,(n), is in the lower layer
      lower(count_l, 1:7) = knobs(n, 1:7); %save this knob infomation in the lower matrix
      count_l = count_l + 1;
   elseif(knobs(n, 3) == upper_line) %if the this knob,(n), is in the upper layer
       upper(count_u, 1:7) = knobs(n, 1:7); %save this knob infomation in the upper matrix
       count_u = count_u + 1;
   elseif(knobs(n, 3) > upper_line) %if the line layer is over, break the for
       break
   end
end

count = 1;
join = zeros(count_u, 7); %(1,upper_block_type,upper_block_number,upper_knob_index,lower_block_type,lower_block_number,lower_knob_index)
for n = 1 : (count_u - 1) %for every upper knob found
    for m = 1 : (count_l - 1) %for every lower knob found
        if((upper(n, 1) == lower(m, 1)) && (upper(n, 2) == lower(m, 2)) ) %if x and y axis value from the upper knob is equal from the lower knob
            join(count,  1:7) = [1, upper(n, 4), upper(n, 6:7), lower(m, 4), lower(m, 6:7)];
            count = count + 1;
            break
        end
    end
end
y = join(1:(count - 1), 1:7);
block_number = zeros(4, 1);
for n = 1 : (count-1) %for each line of y matrix
   if(y(n, 1) == 1)
       block_number(1) = n;
       same_block = 1;
       for m = n + 1 : (count - 1)
          if( (y(n, 3) == y(m, 3)) && (y(n, 6) == y(m, 6))) %
              same_block = same_block + 1;
              block_number(same_block) = m;
          end
       end
       if(same_block > 1)
           for l = 1 : same_block
               y(block_number(l), 1) = same_block;
           end
       end
   end
end