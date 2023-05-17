function [Aeq,beq] = Equality_Matrices_Assembler(F,N,force,model,M)
%Create Aeq and beq matrices (equalities) of the linear programing problem 
%(Aeq.x = beq).
%
%input: F: The force vector (all forces acting in the model).
%       N: Model block number
%       force: Total number of forces
%       model: Input model
%       M: Block mass matrix
%output: Aeq: A matrix 
%        beq: B matrix

    g = 9.8;
    Aeq = spalloc(6*N, 3*force+1,9*force+3);
    beq = zeros(6*N, 1);
    for n = 1 : N   % For each block in the model
        K_i = spalloc(3, 3*force+1,3*force+1); % Upper portion of the "Wj" matrix from the formulation
        PN_i = spalloc(3*force + 1,3*force + 1,3*force + 1); % The "A" matrix from the formulation. (A = diag(A1,A2,...,An))
        P_i = spalloc(3, 3*force + 1,6*force + 2); % Bottom portion of the "Wj" matrix from the formulation 
        % The Zero matrix inside A, case the kth force does not appear in the block is not in A but in Wj.
        for m = 1 : force 
            if(F(m, 2) == n) % If the current force first block is the current one
                K_i(1:3, (3*m - 2):3*m) = sparse(eye(3)); % Force Balance 
                p_i = F(m, 4:6); % The "pk" vectors from the formulation. Start point coordinates
                P_i(1:3,(3*(m-1)+1):3*m) = sparse([0, -p_i(3), p_i(2); p_i(3), 0, -p_i(1); -p_i(2), p_i(1), 0]);
                if(F(m, 7) == -1) % Correct the direction of force 
                    PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = sparse(-eye(3));
                else
                    PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = sparse(eye(3));
                end
            elseif(F(m, 8) == n) % If the current force second block is the current one
                K_i(1:3, (3*m - 2):3*m) = sparse(eye(3)); % Force Balance
                p_i = F(m, 10:12); % The "pk" vectors from the formulation. Start point coordinates
                P_i(1:3,(3*(m-1)+1):3*m) = sparse([0, -p_i(3), p_i(2); p_i(3), 0, -p_i(1); -p_i(2), p_i(1), 0]);
                if(F(m, 13) == -1) % Correct the direction of force
                    PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = sparse(-eye(3));
                else
                    PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = sparse(eye(3));
                end                
            end
        end
        Aeq((6*n - 5) : 6*n, 1 : (3*force + 1)) = [K_i; P_i] * PN_i; %[W1*A1;W2*A2,...,Wn*An]
        %Mass changes depending on block type [b1;b2;...;bn]
        [col,row] = col_row_converter(model(n, 5));
        mass = col*row*2/60; % If the block does not have a registered mass

        for i = 1:size(M) % Search for the registered mass
            if(model(n, 5) == M(i,1))
                mass =  M(i,2);
                break;
            end
        end
        beq((6*n - 5) : 6*n, 1) = [0; 0; mass*g; 0; 0; 0];
    end
end