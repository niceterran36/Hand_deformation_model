A = vi_Larm
%B = LMs_Luparm

figure()
axis equal
hold on
scatter3(points.vertices(:, 1), points.vertices(:, 2), points.vertices(:, 3), '.', 'MarkerEdgeColor',[0.9, 0.9, 0.9]);
scatter3(A(:, 1), A(:, 2), A(:, 3), '*', 'MarkerEdgeColor', [22/255, 181/255, 240/255]);
%scatter3(B(:, 1), B(:, 2), B(:, 3), '.', 'MarkerEdgeColor', [239/255, 61/255, 145/155]);
hold off