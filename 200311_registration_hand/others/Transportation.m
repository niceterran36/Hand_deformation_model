% Copyright 2020, Benjamin Wolpert, All rights reserved

% The semicolon (;) at the end of a line suppresses console output. Many of the
% outputs have been suppressed to clean up the console. Much of the
% code is sub-optimal. Remove the semicolons to understand how the code
% operates and the decisions I made.
% Due to the procedural nature of these algorithms, I've left the
% functions fairly long with the hopes of increasing follow-along-ability.
%
% Note: for those who don't know, MATLAB indices begin at 1, not 0;
function [] = Transportation()

% When solving simple transportation problems by hand, we display both the
% cost matrix and the allocations in a single tableau. To allow for easier
% programming, we represent the same data using indentically dimensioned
% matarics. An allocation at index i,j of the allocation matrix is at cost
% stored at index i,j of the cost matrix.

% Sample problem
% A = [ 4 6 8 14 13; 11 13 11 10 8; 14 4 10 7 13; 9 7 11 16 8; 12 8 6 14 9];%30
% supply = [50; 70; 30; 50; 45];
% demand = [25 35 45 105 35];

% New problem
%A = [1 5 10000; 2 20 222; 3 20 555; 4 1 555; 5 3 1; 3 3 2];
% A = [1 2 3 4 5 3; 5 20 20 1 3 3; 10000 222 555 555 1 2];
% supply = [1; 1; 1];
% demand = [1 1 1 1 1 1];

% Why not
%A = magic(40);
%A = randi(100,40);
%supply = [20;30;20;25;10;40;10;30;50;75;40;20;50;30;25;40;10;50;60;75;20;30;20;25;10;40;10;30;50;75;40;20;50;30;25;40;10;50;60;75];
%demand = [40,35,25,50,20,40,30,30,50,60,20,50,35,25,40,30,50,20,10,50,40,35,25,50,20,40,30,30,50,60,20,50,35,25,40,30,50,20,10,50];

vogelResult = full(vogels(A,supply,demand)) %returns back allocation matrix, a basic feasible solution, from Vogel's Method
vogelCost = sum(sum(A.*vogelResult)) %calculates the total cost of the initial basic feasible solution
optimalSolution = modi(A, vogelResult, supply, demand) %optimization function
A
fprintf("Optimized cost = %d\n",sum(sum(A.*optimalSolution)))
end


% This function performs Vogel's method on a transportation problem. For
% now, it only works when supply == demand.
function [AVals] = vogels(A,supply,demand)
rows = length(supply);
cols = length(demand);
AVals = sparse(rows,cols); %allocation matrix
vertDif = zeros(rows,1); %vector corresponding to the minimum difference of the rows
horizDif = zeros(1,cols); %vector corresponding to the minimum different of the colums

while 1
    % Break/end condition:
    %   Supply>0 is a boolean array with same size as the supply vector
    %   with 0 at index i if the value at i is greater than 0, and 1 otherwise.
    %   If the sum of all the entries is one, that means there's only one
    %   value left.
    if sum(supply > 0) == 1
        [~, index] = max(supply > 0); % get the index of the 1 remaining value
        AVals(index,:) = AVals(index,:) + demand; %add the remaining values in the demand vector to corresponding row in the answer matrix
        break
    end
    
    if sum(demand > 0) == 1
        [~,index] = max(demand > 0);
        AVals(:,index) = AVals(:,index) + supply;
        break
    end
    
    %calculates the mincost for reach row
    for p = 1:rows
        x = mink(A(p,:),2); %x gets the smallest two values in that row
        if any(isnan(x(:))) %check x for NaN values
            vertDif(p) = 0; %if found, default to the first of the two, guaranteed to be non-nan number
        else
            x = abs( int64(x(1)) - int64(x(2))); %calulcate the difference of the two
            vertDif(p) = x;
        end
    end
    %calculates the min cost for each column
    for q = 1:cols
        x = mink(A(:,q),2);
        if any(isnan(x(:)))
            horizDif(q) = 0;
        else
            x = abs( int64(x(1)) - int64(x(2)));
            horizDif(q) = x;
        end
    end
    
    [maxVert, vertIndex] = max(vertDif); % gets the value and index of the smallest cost of the rows
    [maxHoriz, horizIndex] = max(horizDif); %gets the value and index of the smallest cost of the columns
    
    %if the smallest of the two value is of the rows
    if maxVert > maxHoriz
        total = supply(vertIndex); %the resource value associated with that index of the supply vector
        [~, minCostIndex] = min(A(vertIndex,:)); % of the tableau, this gets the minimum cost spot on the specified row and its index
        least = min(total,demand(minCostIndex)); % gets the smaller of the supply and demand vector index positions that correspond to the min cost spot of the tableau
        AVals(vertIndex,minCostIndex) = least; %put the least in the answer tableau / allocate those resources
        supply(vertIndex) = supply(vertIndex)-least; %subtract that amount from the supply vector as those resources have been allocated
        demand(minCostIndex) = demand(minCostIndex)-least; % subtract that amount from the demand vector as those resources have been allocated
        
        %check for new zeros in the demand or supply vector after the
        %deductions and set its corresponding row/column in the tableau to
        %nan so not to be used for future calculations
        if demand(minCostIndex) == 0
            A(:,minCostIndex) = nan;
        end
        if supply(vertIndex) == 0
            A(vertIndex,:) = nan;
        end
        
    else
        total = demand(horizIndex);
        [~, minCostIndex] = min(A(:,horizIndex));
        
        least = min(total, supply(minCostIndex));
        AVals(minCostIndex,horizIndex) = least;
        supply(minCostIndex) = supply(minCostIndex)-least;
        demand(horizIndex) = demand(horizIndex)-least;
        
        if demand(horizIndex) == 0
            A(:,horizIndex) = nan;
        end
        if supply(minCostIndex) == 0
            A(minCostIndex,:) = nan;
        end
    end
end
end

%This function iterates and optimizes the basic feasible solution
%calculated by Vogel's method. It will only perform correctly on a basic
%feasible solution. A feasible solution is not enough.
function [result] = modi(A, result, supply, demand)

while 1
    % If the number of basic variables after Vogel's is less than the number
    % of rows + the number of columns (in our original cost matrix) - 1, our
    % solution is degenerate.
    % Note: Vogel's will never produce more than this number of allocations.
    %
    % To resolve degeneracy, we must make an artificial allocation in the
    % LEAST cost entry equivalent in our allocation matrix that does NOT
    % form a loop. Here, we are sorting the non-basic variables, which
    % correspond to unallocated entries in the allocation matrix, and
    % calling our loop-finding function on the ascending list until we find
    % our index. The artifical allocation is technically larger
    % than 0 but functions as 0 during the matlab operations used here.
    % However, it changes the precision of matrix.
    while nnz(result) < length(supply)+length(demand)-1
        [nbasic] = find(~result);
        [~, order] = sort(A(nbasic));%sort from smallest to largest
        for i = 1:length(nbasic)
            [row,col] = ind2sub([length(supply) length(demand)], nbasic(order(i)));
            isLoop = loop(result,row,col,row,col,0,0,0); %returns loop, if any
            if isempty(isLoop) %if empty, then no loop.
                result(row, col) = typecast(uint32(1),'single');
                break;
            end
        end
    end
    
    numOfBasic = nnz(result);%Number of basic variables
    [colID, rowID] = find(result');%Returns the row and column index of basic variables
    columns = size(result,2);
    rows = size(result,1);
    
    %This is the coefficient matrix of our system of linear equations.
    sysLin = sparse(numOfBasic, (columns+rows) );
    
    for index=1:size(colID)
        %sup-optimal indexing of sparse matrix but number of loops is minimal
        sysLin(index,rowID(index)) = 1;
        sysLin(index,rows+colID(index)) = 1;
    end
    
    % We will solve for a general system of linear equations
    tr = A';
    b = tr(result' ~= 0);% the vector of costs that correspond to unallocated entries. Non-basic variables.
    x = mldivide(sysLin,b);%solves for our x vector
    
    [newColID, newRowID] = find(~result'); %non-basic entry indices
    nonBasic = tr(result' == 0);%non-basic variables
    
    % Note: solveForNonBasic holds the coefficients for the NON-BASIC
    % ENTRIES of the z row(bottom row) of our simplex table. From these coefficients we
    % will determine if we must perform another iteration in simplex, and if
    % we do, what the entering variable(most negative) will be.
    solveForNonBasic = nonBasic-( x(newRowID) + x(rows+newColID) );
    [minVal, minIndex] = min(solveForNonBasic);
    
    %If the smallest basic variable is positive, our solution is optimal
    %and we stop.
    if minVal >= 0
        break
    end
    
    % If we've made it here, our solution is not optimal. We must do an
    % iteration using the most negative of our z-row coefficients and
    % using it as our entering variable.
    rowIndexOfMin = newRowID(minIndex);
    colIndexOfMin = newColID(minIndex);
    foundLoop = loop(result,rowIndexOfMin,colIndexOfMin,rowIndexOfMin,colIndexOfMin,0,0,0);
    loopVals = sub2ind(size(result),foundLoop(1:2:end),foundLoop(2:2:end));
    loopVals = loopVals(1:end-1);
    addVals = loopVals(1:2:end);
    subVals = loopVals(2:2:end);
    
    [minLoopVal] = min(result(subVals));
    result(addVals) = result(addVals)+minLoopVal;
    result(subVals) = result(subVals)-minLoopVal;
end
end

% This function will find a loop, if one exists, at a given entry in a
% matrix. Loops are valid if they contain vertices only at allocated entries
% and there exists no row or column with three vertices. No diagonal lines.
%{
                Valid Loops
    ____        __          __
   |    |      |  |        |  |
   |    |      |__|__      |  |__
    ----          |  |     |     |
                   --       -----

                Invalid Loops
             ___ ___         ___
                |   |        \  |
                |   |         \ |
                 ---           \|
%}
% If a loop is found, it will return a (very poorly formatted) list of
% indices. If a loop is not found, it retuns an empty list.
% This algorithm uses a variation of recursive Depth-First Search (dfs),
% recursing back immediately when a loop is found.
% In this function, neighbors are defined as allocated indices within the
% same row or column as the current index.
%
% Note: the variable name bfs stands for "Basic Feasible Solution."
function [loopList] = loop(bfs,targetRow,targetCol,rowIndex,colIndex,prevRow,prevCol,count)

loopList = [];
if count > 2 % Return condition. This is met when we are at the index we started.
    if rowIndex == targetRow && colIndex == targetCol
        loopList = [rowIndex colIndex];
        return
    end
end

% Because a single row or column may not have three vertices, every time we
% check a neighbor, because that neighbor is guaranteed to be in the same
% column or row, we must nan that entire row during the current recursion
% stack so we don't choose another index on that row/col. The change does
% not last the return call and is undone when recursing back after dead
% ends.
if count ~= 0
    if rowIndex == prevRow
        bfs(rowIndex,:) = nan;
    elseif colIndex == prevCol
        bfs(:,colIndex) = nan;
    end
    bfs(targetRow,targetCol) = -1;
end

if count == 1 %The second time checking for neighbors will always include the original index.
    bfs(targetRow,targetCol) = 0;%So we will zero it out just for this frame
end
save = bfs(rowIndex,colIndex); %save the value at the current index
bfs(rowIndex,colIndex) = 0; % we set the current index to zero so it doesn't show as a neighbor to itself

[row] = find(bfs(rowIndex,:) ~= 0 & ~isnan(bfs(rowIndex,:)));%row neighbors, can be empty
[col] = find(bfs(:,colIndex) ~= 0 & ~isnan(bfs(:,colIndex)));%column nieghbors, can be empty

if isempty(row) %if we have no row neighbors
    rowNeighbors = []; %explicitly set list to empty
else
    rowNeighbors = [ones(length(row),1)*rowIndex, row'];%this is janky but it works. It just organizes the indices
end
if isempty(col)%if we have no column neighbors
    colNeighbors = []; %explicitly set list to empty
else
    colNeighbors = [col, ones(length(col),1)*colIndex];%still janky
end

neighbors = [rowNeighbors;colNeighbors]; %Our full nieghbor list, represented using a matrix

if count > 2 && ~isempty(neighbors)
    if ismember([targetRow targetCol],neighbors,'rows')
        loopList = loop(bfs,targetRow,targetCol,targetRow,targetCol,rowIndex,colIndex,count+1);
        loopList(end+1) = rowIndex; %add current row to list
        loopList(end+1) = colIndex; %add current column to list
        return
    end
end
sizeM = size(neighbors,1);%number of rows
if sizeM == 0
    return
end

%This loop drives the recursion on the neighbors list.
for index=1:sizeM
    bfs(rowIndex,colIndex) = save;
    loopList = loop(bfs,targetRow,targetCol,neighbors(index,1),neighbors(index,2),rowIndex,colIndex,count+1);
    if ~isempty(loopList)
        loopList(end+1) = rowIndex;
        loopList(end+1) = colIndex;
        return
    end
end
end
