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