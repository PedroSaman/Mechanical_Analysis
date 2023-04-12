function plan_formatation(plan,file_location)
%myFun - Description
%
% Syntax: output = myFun(input)
%
% Long description

% #AssemblyArea,X,Y,Z,SizeX,SizeY,SizeZ,ColorIndex,IsSupport,CanPress,ShiftX,ShiftY

    csv_plan = ones(size(plan,1),12);

    for i = 1:size(plan,1)
        csv_plan(i,1) = 0;
        csv_plan(i,2:3) = plan(i,2:3) - 1;
        csv_plan(i,4) = plan(i,4);
        [csv_plan(i,5),csv_plan(i,6)] = col_row_converter(plan(i,5));

        if(plan(i,6) ~= 99)
            csv_plan(i,8) = plan(i,6);
            csv_plan(i,9) = 0;
        else
            csv_plan(i,8) = 0;
        end

        csv_plan(i,11) = 0; %for now = 0
        csv_plan(i,12) = 0; %for now = 0
        %{
            ShiftX,ShiftY =  

            if (rightNeightbors.Any(p => this.Occupy(p))) shiftX--;
            if (leftNeightbors.Any(p => this.Occupy(p))) shiftX++;
            if (frontNeightbors.Any(p => this.Occupy(p))) shiftY--;
            if (backNeightbors.Any(p => this.Occupy(p))) shiftY++;
        %} 
    end
    csvwrite(strrep(file_location, 'txt' , 'csv' ), csv_plan);
end