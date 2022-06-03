function [Aeq,beq] = Aeq_beq_matrices_assembler(F,N,force,model,M)

g = 9.8;
Aeq = zeros(6*N, 3*force+1);
beq = zeros(6*N, 1);
for n = 1 : N   %for each block in the model
    K_i = zeros(3, 3*force+1); %This is the upper portion of the "Wj" matrix from the formulation
    p_i = zeros(force, 3); %This is all the "pk" vectors from the formulation into one matrix
    PN_i = eye(3*force + 1); %This is the "A" matrix from the formulation. (A = diag(A1,A2,...,An))
                             %Here the Zero matrix inside A, case the kth force does not appear in 
                             %the block is not in A but in Wj.
    for m = 1 : force 
       if(F(m, 2) == n) %if the current force first block is the current one
           K_i(1:3, (3*m - 2):3*m) = eye(3); %Force Balance 
           p_i(m, 1 : 3) = F(m, 4:6); %Start point coordinates 
           if(F(m, 7) == -1) %Correct the direction of force 
               PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = -eye(3);
           end
        elseif(F(m, 8) == n) %if the current force second block is the current one
           K_i(1:3, (3*m - 2):3*m) = eye(3); %Force Balance
           p_i(m, 1 : 3) = F(m, 10:12); %Start point coordinates
           if(F(m, 13) == -1) %Correct the direction of force
               PN_i((3*m - 2) : 3*m, (3*m - 2) : 3*m) = -eye(3);
           end
       end
    end
    P_i = zeros(3, 3*force + 1); %This is the botton portion of the "Wj" matrix from the formulation 
    for l = 3 : 3 : 3*force
        P_i(1:3, (l-2):l) = [0, -p_i(l/3, 3), p_i(l/3, 2); p_i(l/3, 3), 0, -p_i(l/3, 1); -p_i(l/3, 2), p_i(l/3, 1), 0];
    end
    Aeq((6*n - 5) : 6*n, 1 : (3*force + 1)) = [K_i; P_i] * PN_i; %[W1*A1;W2*A2,...,Wn*An]
    %Mass changes depending on block type [b1;b2;...;bn]
    [col,row] = col_row_converter(model(n, 5));
    mass = col*row*2/60; %if the block does not have a registered mass
    
    for i = 1:size(M) %Search for the registered mass
        if(model(n, 5) == M(i,1))
           mass =  M(i,2);
           break;
        end
    end
    
    beq((6*n - 5) : 6*n, 1) = [0; 0; mass*g; 0; 0; 0]; %[(0,0,M1*g,0,0,0);...;(0,0,Mn*g,0,0,0)]
end