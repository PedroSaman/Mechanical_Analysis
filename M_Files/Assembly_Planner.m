function Assembly_Planner(filename)
    % This function assumes that the .txt file "filename" is located in 
    % "../Block_Printer_System/Models/BlockModel/"

    tic
    directory = "../Block_Printer_System/Models/BlockModel/";
    incomplete_plans = convertStringsToChars("../Block_Printer_System/Models/plan/Incomplete_Plans/incomplete_" + filename + "/");
    mkdir(incomplete_plans);
    block_model = model_loader(directory + filename + ".txt"); % Load model and adapt it to the Stability Judge format
    
    fprintf("\nStarted %s file Assembly Planning\n\nOriginal block model has %d blocks\n",filename,size(block_model,1));
    if(model_check(block_model)~=1)
        fprintf("model is NG\n");
        return;
    end
    
    [plan,output] = NewPlanner(block_model,filename,[0,0]);
    
    if(~strcmp(output,'ok'))
        fprintf("\nAssembly planning of %s failed\n\n",filename);
        plan_formatation(plan,filename+"fail");
    else
        local = "../Block_Printer_System/Models/plan/";
        plan_formatation(plan,local + filename + ".txt");
        fprintf("\nModel completed. Plan available in %s\n",local + filename + ".txt");
    end
    toc
end