

%segment = 18 15 12 9
%centers = 4 8 12 16
Idx = [1:6984]'

keep = ismember(mesh.assignment, 5); % 18 15 12 9
[V, F] = filter_vertices(mesh.vertices, mesh.faces, keep);
A = Idx(keep)

center = mesh.centers(4,:)

n = size(V, 1); 

delta = V - repmat(center(1, :), n, 1);
distances = sum(delta .^ 2, 2);
distances = sqrt(distances);
[~, j] = min(distances);

keep2 = distances < 15
A = A(keep2)
[Ve, Fe] = filter_vertices(V, F, keep2);