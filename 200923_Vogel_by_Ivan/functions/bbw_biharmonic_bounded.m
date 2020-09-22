function W = bbw_biharmonic_bounded(V, F, b, bc)
    % BIHARMONIC_BOUNDED Compute biharmonic bounded coordinates, using quadratic
    % optimizer
    %
    % W = biharmonic_bounded(V, F, b, bc, type, pou)
    % W = biharmonic_bounded(V, F, b, bc, type, pou, low, up)
    %
    % Inputs:
    %  V  list of vertex positions
    %  F  list of face indices, for 3D F is #F by 4, for 2D F is #F by 3
    %  b  list of boundary vertices
    %  bc list of boundary conditions, size(boundary) by # handles matrix of
    %    boundary conditions where bc(:, i) are the boundary conditions for the 
    %    ith handle
    %
    % Outputs:
    %  W  weights, # vertices by # handles matrix of weights
    %
    % Copyright 2011, Alec Jacobson (jacobson@inf.ethz.ch)
    %
    % See also: boundary_conditions

    % number of vertices
    n = size(V, 1);
    % number of handles
    m = size(bc, 2);

    % Build discrete laplacian and mass matrices used by all handles' solves
    if(size(F, 2) == 4)
        fprintf('Solving over volume...\n');
        L = bbw_cotmatrix3(V, F);
        M = bbw_massmatrix3(V, F, 'barycentric');
    else
        L = bbw_cotmatrix(V, F);
        M = bbw_massmatrix(V, F, 'voronoi');
    end

    % default bounds
    low = 0;
    up = 1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SET UP SOLVER
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    param = optimset( ...
                    'TolFun', 1e-16, ...
                    'Algorithm', 'interior-point-convex', ...
                    ... % 'Algorithm', 'active-set', ...
                    'MaxIter', 1000, ...
                    'Display', 'off');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SET UP PROBLEM AND SOLVE
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    disp('<starting BBW>');
    
    % build quadratic coefficient matrix (bilaplacian operator)
    Q = L*(M\L);
    
    % set bounds
    ux = up.*ones(n, 1);
    lx = low.*ones(n, 1);

    % allocate space for weights
    W = zeros(n, m);
    t = 0;
    
    % loop over handles
    for i = 1:m
        tic;
        fprintf('Point %d / %d ', i, size(b, 2));
        % enforce boundary conditions via lower and upper bounds
        % lx(b) = bc(:, i);
        % ux(b) = bc(:, i);
        Aeq = speye(n, n);
        Aeq = Aeq(b, :);
        [x, fval, err] = quadprog(Q, zeros(n, 1), [], [], Aeq, bc(:, i), lx, ux, [], param);
%         if(err ~= 1)
%             fprintf(['\nERROR ('  num2str(err) ', ' num2str(fval) '):' ...
%                      ' solution may be inaccurate...\n']);
%         end
      % set weights to solution in weight matrix
      W(:, i) = x(1:n);
      fprintf('(Lap time: %0.2gs)\n', toc);
      t = t + toc;
    end
    fprintf('Total elapsed time: %0.1gs\n', t);
    fprintf('Average time per handle: %0.1gs\n\n', t/m);
end
