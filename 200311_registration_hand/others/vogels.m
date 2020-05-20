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
            vertDif(p) = -1; %if found, default to the first of the two, guaranteed to be non-nan number
        else
            x = abs( int64(x(1)) - int64(x(2))); %calulcate the difference of the two
            vertDif(p) = x;
        end
    end
    %calculates the min cost for each column
    for q = 1:cols
        x = mink(A(:,q),2);
        if any(isnan(x(:)))
            horizDif(q) = -1;
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