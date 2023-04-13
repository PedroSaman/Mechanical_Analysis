function [fncoutput] = Planner_Stability_Judge(model)
    %% Stability_Judg_pushing.m
    % This script will take the datfile specified in "filename" and compute
    % where each force would appear if the assembled structure is static minus
    % the last block. It also compute how the insertion of the last block in
    % the model disturbs the static balance. After that, a linear programing 
    % problem will try to minimize the minimum capacity in the model.
    % Input: Filename(is the location of a datfile in the correct format)
                     % dat file format: each line is one block. All coordinates
                     % are related to the FIRST knob of the block. Being it:
                     % (x, y, z, block type). The dat file MUST be sorted to be
                     % increasing in the value of Z from 1 to z_max.
    % Output: CapacityMargin(CM): The stability judge.
    % OBS: T - A*x will show how much force is overflowing (case negative) or 
    % still can be applied (case positive) for each knob in the model.

    %% Constant numbers
    %tic
    g = 9.8;
    T = 151*g; % Maximum static friction force of one set of convex part
    M = [11,17/448;12,1.39/20;21,1.39/20;13,17/175;31,17/175;14,1.03/8;41,1.03/8;22,8.1/64;24,3.9/16;42,3.9/16;28,11/24;82,11/24]; % Mass of each registered block
    good_margin = T*0.4; % Arbitrary minimum value for stability

    %% Load the model and search for structural problems
    check = model_check(model); % Verify the datfile model
    if(check == -1 ) 
        %fprintf('This model has a block that is not currently available in the laboratory. \n');
        fncoutput = 'This input has blocks not available';
        return;
    elseif(check == -2)
        %fprintf('This model has multiples blocks occupying the same spot. \n');
        fncoutput = 'This input has blocks occupying the same space';
        return;
    elseif(check == -3)
        %fprintf('This model has blocks floating. \n');
        fncoutput = 'This input has blocks floating';
        return;
    end

    %% Model, knobs and pushing force information
    N = size(model,1); % Total block number
    b_push = pushing(model, N, T); % Forces and Torques caused by the last block insertion
    if(isempty(b_push))
        %fprintf('The block being inserted has no contact forces \n');
        fncoutput = 'The block being inserted has no contact forces';
        return;
    end
    N = N - 1; % Ignore the last block in the usual stability judge.
    model = model(1:N,:); % Exclude the last block from the model.
    z_max = model(end,4); % Structure height
    knobs = knob(model,0); % Knob information (entire block model)

    %% Find the forces acting on the model
    %For the first layer
    %knob_i = knob(model, 1); % Layer knob information
    %adjoin_i = adjoin(knob_i); % Layer adjoin blocks information
    F_f = Ff_0_180723(model); % First layer friction forces
    F_nz = Fnz_0(model); % First layer z axis normal forces
    %F_nx = Fnx(adjoin_i); % Layer X axis normal forces 
    %F_ny = Fny(adjoin_i); % Layer Y axis normal forces

    %For second layer onwards 
    for n = 2 : z_max 
        %knob_i = knob(model, n); % Layer knob information
        %adjoin_i = adjoin(knob_i); % Layer adjoin blocks information
        join_i = join(knobs, n); % Connected knobs information

        Ff_i = Ff_180723(join_i); % Layer friction forces
        Fnz_i = Fnz(join_i);   % Layer z axis normal forces
        %Fnx_i = Fnx(adjoin_i); % Layer X axis normal forces
        %Fny_i = Fny(adjoin_i); % Layer Y axis normal forces

        % Add the computed forces to the final forces matrix
        F_f = [F_f; Ff_i];
        F_nz = [F_nz; Fnz_i];
        %F_nx = [F_nx; Fnx_i];
        %F_ny = [F_ny; Fny_i];
    end

    % Count the number of forces
    force_f = size(F_f,1); % Friction force number
    force_nz = size(F_nz,1); % Z axis normal force number
    %force_nx = 0;%size(F_nx,1); % X axis normal force number
    %force_ny = size(F_ny,1); % Y axis normal force number
    F = [F_f; F_nz];%; F_nx; F_ny];
    force = force_f + force_nz ;%+ force_nx + force_ny; % Total number of forces

    %% Force value bounds
    %Lower Bounds
    lb = -Inf(3*force+1, 1); % Number of forces times 3 (X, Y, Z) + Capacity
    lb(3:3:3*force_f) = 0; % F_f Z axis lower bound value is 0
    lb(3*force_f+1:3*force) = 0; % F_n lower bound value in all 3 axis is 0

    %Upper bounds
    ub = Inf(3*force+1, 1); % Number of forces times 3 (X, Y, Z) + Capacity
    ub(3*force_f + 1:3:3*(force_f+force_nz) - 2) = 0; % Fnz X axis upper bound value is 0
    ub(3*force_f + 2:3:3*(force_f+force_nz) - 1) = 0; % Fnz Y axis upper bound value is 0
    %ub(3*(force_f+force_nz) + 2:3:3*(force_f+force_nz+force_nx) - 1) = 0; % Fnx Y axis upper bound value is 0
    %ub(3*(force_f+force_nz) + 3:3:3*(force_f+force_nz+force_nx)) = 0; % Fnx Z axis upper bound value is 0
    %ub(3*(force_f+force_nz+force_nx) + 1:3:3*force - 2) = 0; % Fny X axis upper bound value is 0
    %ub(3*(force_f+force_nz+force_nx) + 3:3:3*force) = 0; %Fny Z axis upper bound value is 0

    % Friction force X and Y axis bounds (Fn_line) and orientation correction
    for i = 1 : force
        if((F(i, 7) == -1))
            % Ff X axis points to -infinite and does not have component in Y axis 
            lb(3*i-1) =  0;   ub(3*i-2 : 3*i-1) = [0, 0]; %no force in Y axis and in X axis bounds are [-inf,0]
        elseif((F(i, 7) == -2)) 
            % Ff does not have component in X axis and Y axis points to -infinite 
            lb(3*i-2) =  0;   ub(3*i-2 : 3*i-1) = [0, 0]; %no force in X axis and in Y axis bounds are [-inf,0]
        elseif((F(i, 7) == 2))
            % Ff does not have component in X axis and Y axis points to +infinite
            lb(3*i-2 : 3*i-1) = [0, 0];   ub(3*i-2) =  0; %no force in X axis and in Y axis bounds are [0,+inf]
        elseif((F(i, 7) == 1))
            % Ff X axis points to +infinite and does not have component in Y axis
            lb(3*i-2 : 3*i-1) = [0, 0];   ub(3*i-1) = 0; %no force in Y axis and in X axis bounds are [0,+inf]
        elseif((F(i, 7) == -3))
            ub(3*i) = 0.1*T;
        end
        if(i<=force_f)
            if(F(i,8) == 0) % If the force is from the first layer
                F(i, 7) = -1;
            else
                F(i, 7:6:13) = [-1, 1];
            end
        end
    end

    % Fn_line upper limit setting (Only the inner Fn_line from the nx2/2xn blocks)
    %{
    for i = 1 : force_f
        if(F(i, 4) == -0.75)
            lb(3*i-2) = -T;
        elseif(F(i, 4) == 0.75)
            ub(3*i-2) = T;
        elseif(F(i, 5) == -0.75)
            lb(3*i-1) = -T;
        elseif(F(i, 5) == 0.75)
            ub(3*i-1) = T;
        end
    end
    %}
    
    %% Linear programming problem
    [A,b] = A_b_matrices_assembler(F,force_f,model,T); % Linear Inequalities
    [Aeq,beq] = Aeq_beq_matrices_assembler(F,N,force,model,M); % Linear equalities
    f = sparse(3*force+1, 1); % Evaluate function
    f((3*force + 1), 1) = -1; % Change the problem to minimization

    % Add pushing force and momentum to beq 
    N_push = size(b_push,1); % Number of blocks to which pushing force is applied
    for n = 1 : N_push
        pbn = b_push(n, 1);
        b_push(n, 4) = b_push(n, 4) + beq(6*pbn-3); % Pushing force in Z (Fz) + gravity 
        b_dummy = b_push(n, 2:7).';
        beq(6*pbn-5 : 6*pbn) = b_dummy;
    end
    options = optimoptions('linprog','Display','none');
    [x,~,~,~] = linprog(f,A,b,Aeq,beq,lb,ub,options); % Solve the problem

    %% Final output messages
    if(~isempty(x)) % If there were a solution
        %fprintf('Solution found\n');
        %XX =[x(1:3:3*force-2), x(2:3:3*force-1), x(3:3:3*force)]; % All forces (x,y,z) 
        CM = x(3*force+1); %Capacity CM 
        if(CM >= good_margin)
            fncoutput = 'safe';
            %fprintf('Stability with good security margin. CM = %.4f \n',CM);
        else
            fncoutput = 'not safe';
            %fprintf('Stability can not be guaranteed. CM = %.4f \n',CM);
        end
    else
        fncoutput = 'no solution';
        %fprintf('No feasible solution found \n');
    end
    %CM = 100*round(CM/T,4); % percentage of the maximum value
    %fprintf('Time elapsed: %.2d \n',toc);
end