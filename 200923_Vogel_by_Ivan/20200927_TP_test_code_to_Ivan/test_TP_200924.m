
load('A.mat')
load('B.mat')
load('demand.mat')
load('supply.mat')

% small scale testing
tic
Sol = Transportation_code(A, supply, demand);
toc


% original scale testing
tic
supply2 = ones(3946,1);
demand2 = ones(1,3946);

Sol2 = Transportation_code(B, supply2, demand2);
toc

    
    