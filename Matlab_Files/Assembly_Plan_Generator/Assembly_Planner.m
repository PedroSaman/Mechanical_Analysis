function Assembly_Planner(filename)
%Assembly Planner with mechanical analysis stability judge main function.
%This function creates debug files with the incomplete plans for every
%completed layer. If successeful, the assembly plan will be saved in the 
%"../../Block_Printer_System/Models/plan/" path.
%
%input: filename: only the file name, without file type
%OBS: This function assumes that the .txt file "filename" is located in 
% "../../Block_Printer_System/Models/BlockModel/". If a new place is desired,
% change the "directory" variable.

    tic
    %% Read the block model
    directory = "../../Block_Printer_System/Models/BlockModel/";
    incomplete_plans = convertStringsToChars("../../Block_Printer_System/Models/plan/Incomplete_Plans/incomplete_" + filename + "/");
    mkdir(incomplete_plans);
    block_model = model_loader(directory + filename + ".txt"); % Load model and adapt it to the Stability Judge format
    fprintf("\nStarted %s file Assembly Planning\n\nOriginal block model has %d blocks\n",filename,size(block_model,1));
    if(model_check(block_model)~=1)
        fprintf("model is NG\n");
        return;
    end
    
    %% Run Assembly Planner
    [plan,output] = NewPlanner(block_model,filename,[0,0]);
    
    %% Save output
    if(~strcmp(output,'ok'))
        fprintf("\nAssembly planning of %s failed\n\n",filename);
        plan_formatation(plan,filename+"fail");
    else
        local = "../../Block_Printer_System/Models/plan/";
        plan_formatation(plan,local + filename + ".txt");
        fprintf("\nModel completed. Plan available in %s\n",local + filename + ".txt");
    end
    toc
end