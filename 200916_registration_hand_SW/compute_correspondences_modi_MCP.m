function pairs = compute_correspondences_modi_MCP(mesh_vertices, mesh_normals, points_vertices, points_normals, distance_threshold,cos_angle_threshold)
%COMPUTE_CORRESPONDENCES For each mesh vertex, find closest point in cloud
    
% Define missing arguments
    if nargin < 5
         distance_threshold = 30;
    end
    if nargin < 6
        cos_angle_threshold = cos(30 * pi / 180);
    end

    n = size(mesh_vertices, 1); % vertices --> mesh_vertices·Î º¯°æ
    m = size(points_vertices, 1);
    pairs = zeros(n, 2);

    % cut points_vertices with boundary 
    
    centroid = mean(mesh_vertices);
    Idx_points = [1:size(points_vertices,1)]';
    
    delta = points_vertices - repmat(centroid, m, 1);
    distance = sqrt(sum(delta .^ 2, 2));
    % distances(distances>30) = 0
    LIX = distance < distance_threshold;
    
    points_v_candidate = points_vertices(LIX,:);
    points_v_candidate(:,4) = Idx_points(LIX);
    points_n_candidate = points_normals(LIX,:);
    m = size(points_v_candidate,1);
    
    mesh_normals = normalizerow(mesh_normals);
    points_n_candidate = normalizerow(points_n_candidate); 

    % For each mesh vertex, find closest point
        
    cos_angle = zeros(m,1);
    for i = 1 : n % for template_vertices
        for j = 1 : m % for scan points
        cos_angle(j,1) = mesh_normals(i, :) * points_n_candidate(j, :)';
        end
        
        LIX_angle = cos_angle > cos_angle_threshold;
        points_v_candidate =  points_v_candidate(LIX_angle,:);
        m = size(points_v_candidate,1);
        
        delta =  points_v_candidate(:,1:3) - repmat(mesh_vertices(i, :), m, 1);
        distances_delta = sqrt(sum(delta .^ 2, 2));
        [row, col] = min(distances_delta);
        
        pairs(i, 1) = i;
        pairs(i, 2) = points_v_candidate(col,4); 
        
        clear LIX_angle delta distances_delta col
        
        points_v_candidate = points_vertices(LIX,:);
        points_v_candidate(:,4) = Idx_points(LIX);
        points_n_candidate = points_normals(LIX,:);
        m = size(points_v_candidate,1);
    end

%     keep = true(n, 1);
%     for i = 1 : n
%           p = pairs(i, 1);
%           q = pairs(i, 2);
%           cos_angle_candidate = mesh_normals(p, :) * points_normals(q, :)';
%           if cos_angle_candidate < cos_angle_threshold
%             keep(i) = false;
%           end
%     end
%     pairs = pairs(keep, :);
    
        % Filter bad pairs
   
%     % TODO vectorize this
%     
%     for i = 1 : n
%         p = pairs(i, 1);
%         q = pairs(i, 2);
%         
%         % Check if distance is too large
%         distance = norm(mesh_vertices(p, :) - points_vertices(q, :));
%         if distance > distance_threshold
%              keep(i) = false;
%          end
%         
%         % Check if normals differ
%         cos_angle = mesh_normals(p, :) * points_normals(q, :)';
%         if cos_angle < cos_angle_threshold
%             keep(i) = false;
%         end
%         
%     end
%     pairs = pairs(keep, :);

end
