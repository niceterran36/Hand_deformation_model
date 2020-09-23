function pairs = compute_correspondences_new(mesh_vertices, mesh_normals, points_vertices, points_normals);

    n = size(vertices, 1); % vertices --> mesh_vertices·Î º¯°æ
    m = size(points.vertices, 1);
    pairs = zeros(n, 2);

    % cut points_vertices with boundary 
    
    centroid = mean(vertices);
    Idx_points = [1:size(points.vertices,1)]';
    
    delta = points.vertices - repmat(centroid, m, 1);
    distances = sqrt(sum(delta .^ 2, 2));
    % distances(distances>30) = 0
    LIX = distances < 30;
    points_v_candidate = points.vertices(LIX,:)
    points_v_candidate(:,4) = Idx_points(LIX)
    points_n_candidates = points.normals(LIX,:)
    
      
    for i = 1 : m    
        [~, j] = min(distances);
        pairs(i, 1) = i;
        pairs(i, 2) = j;
    end



end 

figure()
axis equal
axis off
hold on
scatter3(vertices(:,1),vertices(:,2),vertices(:,3),'.', 'MarkerEdgeColor',[144/255, 228/255, 255/255]);
scatter3(points_candidate(:,1),points_candidate(:,2),points_candidate(:,3),'.', 'MarkerEdgeColor',[220/255, 220/255, 220/255]);
scatter3(centroid(1,1),centroid(1,2),centroid(1,3),'*','MarkerEdgeColor',[255/255, 0/255, 0/255]);
view(-195,0);
hold off