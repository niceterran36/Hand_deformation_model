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