function [col,row] = col_row_converter(block_type)
%Find out the number of collums and rows a block have givven it block type
%This function works only for up to 9x9 blocks
%input: block_type
%output: col:(number of collums of the block)
%        row:(number of rows of the block)

    col = floor(block_type/10);                        %block number of collums
    row = round(10*((block_type/10)-floor(block_type/10)));%block number of rows
end