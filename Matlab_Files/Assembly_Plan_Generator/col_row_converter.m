function [col,row] = col_row_converter(block_type)
%Compute the number of columns and rows a block has given its block type
%input: block_type
%output: col:(number of columns of the block)
%        row:(number of rows of the block)

    col = floor(block_type/10); % Block number of columns
    row = round(10*((block_type/10)-floor(block_type/10))); % Block number of rows
end