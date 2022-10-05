function [model,range] = model_loader(filename)
    
    model_original = load(filename);
    k = size(model_original,1);
    model(:,1) = 1:k;
    model(:,2:4) = model_original(:,1:3) + 1;
    model(:,5) = model_original(:,4)*10 + model_original(:,5);
    model(:,6) = model_original(:,7);
    
end