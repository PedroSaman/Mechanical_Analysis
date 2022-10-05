function Assembly_Planner(filename)

    tic
    block_model = model_loader(filename); % Load model and adapt it to the Stability Judge format
    
    fprintf("\nStarted %s file Assembly Planning\n\nOriginal block model has %d blocks\n",filename,size(block_model,1));
    
    [plan,output] = NewPlanner(block_model);
    
    if(~strcmp(output,'ok'))
        fprintf("\nAssembly planning of %s failed\n\n",filename);
        plan_formatation(plan,filename+"fail");
    else
        plan_formatation(plan,filename);
    end
    toc
end