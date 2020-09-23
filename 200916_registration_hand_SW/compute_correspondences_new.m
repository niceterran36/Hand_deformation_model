function pairs = compute_correspondences_new(mesh_vertices, mesh_normals, points_vertices, points_normals);

    n = size(vertices, 1); % vertices --> mesh_vertices�� ����
    m = size(points.vertices, 1);
    pairs = zeros(n, 2);

    % cut points_vertices with boundary 
    
    centroid = mean(vertices);
    Idx_points = [1:size(points.vertices,1)]';
    
    delta = points.vertices - repmat(centroid, m, 1);
    distances = sqrt(sum(delta .^ 2, 2));
    % distances(distances>30) = 0
    LIX = distances < 30;
    points_v_candidate = points.vertices(LIX,:);
    points_v_candidate(:,4) = Idx_points(LIX);
    points_n_candidates = points.normals(LIX,:);
    m = size(points_v_candidate,1);
    
    A = zeros(m,m);
    supply = ones(m,1);
    demand = ones(1,m);
    
    normals = normalizerow(normals);
    points_n_candidates = normalizerow(points_n_candidates);    
      
    for i = 1 : n % for template_vertices

       for j = 1 : m % for scan points

            cos_angle = normals(i, :) * points_n_candidates(j, :)';
            distance = norm(vertices(i, :) - points_v_candidate(j, 1:3));

            if cos_angle < 0
               cost = 1000;
            else 
               cost = distance;
            end 

            A(j,i) = cost;
            
       end 

    end
    
    Sol = Transportation_code(A, supply, demand);
    
end 

% figure()
% axis equal
% axis off
% hold on
% view(-195,0);
% scatter3(vertices(:,1),vertices(:,2),vertices(:,3),'.', 'MarkerEdgeColor',[144/255, 228/255, 255/255]);
% scatter3(points_candidate(:,1),points_candidate(:,2),points_candidate(:,3),'.', 'MarkerEdgeColor',[220/255, 220/255, 220/255]);
% scatter3(centroid(1,1),centroid(1,2),centroid(1,3),'*','MarkerEdgeColor',[255/255, 0/255, 0/255]);
% hold off