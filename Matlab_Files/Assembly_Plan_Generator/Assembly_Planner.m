function Assembly_Planner(file_name)
%Assembly Planner with mechanical analysis stability judge main function.
%This function creates debug files with the incomplete plans for every
%completed layer. If successeful, the assembly plan will be saved in the 
%"../../Block_Printer_System/Models/plan/" path.
%
%input: filename: only the file name, without file type
%OBS: This function assumes that the .txt file "filename" is located in 
% "../../Block_Printer_System/Models/BlockModel/". If a new place is desired,
% change the "directory" variable.

    %% Read the block model
    directory = "../../Block_Printer_System/Models/BlockModel/";
    incomplete_plans = convertStringsToChars("../../Block_Printer_System/Models/plan/Incomplete_Plans/incomplete_" + file_name + "/");
    mkdir(incomplete_plans);
    block_model = model_loader(directory + file_name + ".txt"); % Load model and adapt it to the Stability Judge format
    fprintf("\nStarted %s file Assembly Planning\n\nOriginal block model has %d blocks\n",file_name,size(block_model,1));
    if(model_check(block_model)~=1)
        fprintf("model is NG\n");
        return;
    end
    
    %% Run Assembly Planner
    tic
    [plan,output] = NewPlanner(block_model,file_name,[0,0]);
    toc
    %% Save output
    if(~strcmp(output,'ok'))
        fprintf("\nAssembly planning of %s failed\n\n",file_name);
        plan_formatation(plan,file_name+"fail",0);
    else
        local = "../../Block_Printer_System/Models/plan/";
        block_counter(plan_formatation(plan,local + file_name + ".txt",1));
        fprintf("\nModel completed. Plan available in %s\n",local + file_name + ".csv");
    end
end