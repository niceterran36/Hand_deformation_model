function pairs = correspondences_rigid(mesh_vertices, mesh_normals, points_vertices, points_normals, distance_threshold, cos_angle_threshold)

    if nargin < 5
         distance_threshold = 30;
    end
    if nargin < 6
        cos_angle_threshold = cos(60 * pi / 180);
    end

    n = size(mesh_vertices, 1); 
    m = size(points_vertices, 1);
    pairs = zeros(n, 2);

    % cut points_vertices with boundary 
    
    centroid = mean(mesh_vertices);
    Idx_points = [1:size(points_vertices,1)]'; 
    
    delta = points_vertices - repmat(centroid, m, 1);
    distance = sqrt(sum(delta .^ 2, 2));
    LIX = distance < distance_threshold;
   
    points_v_candidate = points_vertices(LIX,:);
    points_v_candidate(:,4) = Idx_points(LIX);
    points_vidx_candidate = points_v_candidate(:,4);
    points_n_candidate = points_normals(LIX,:);
    m = size(points_v_candidate,1);
    
    mesh_normals = normalizerow(mesh_normals);
    points_n_candidate = normalizerow(points_n_candidate); 

    % random option
    %Rad = round(rand(m,1)*1000);        
    
    Rad_idx = zeros(500,1);
    for ii = 1:500
        Rad_idx(ii) = randi(m);
    end
    m = 500;

    points_v_r_candidate = points_v_candidate(Rad_idx,1:3);
    points_n_r_candidate = points_n_candidate(Rad_idx,1:3);
    points_vidx_r_candidate = points_vidx_candidate(Rad_idx);

    B = zeros(m,m);
    supply = ones(m,1);
    demand = ones(1,m);
    
    
          
    for i = 1 : n % for template_vertices

       for j = 1 : m % for scan points
            cos_angle_i = mesh_normals(i, :) * points_n_r_candidate(j, :)';
            distance_i = norm(mesh_vertices(i, :) - points_v_r_candidate(j, 1:3));

            if cos_angle_i < cos(30 * pi / 180)
               cost = 1000;
            elseif distance_i > 25
               cost = 100;
            else 
               cost = distance_i;
            end 
            B(j,i) = cost;            
       end 

    end
    tic
    Sol = Transportation_code(B, supply, demand);
    Sol(:,n+1:end) = [];
    toc

    for i = 1:n
        pairs(i, 1) = i;
        C = Sol(:,i);
        xx = C == 1;
        pairs(i, 2) = points_vidx_r_candidate(xx); 
        clear C
    end 
    
    keep = true(n, 1);
    for i = 1 : n
          p = pairs(i, 1);
          q = pairs(i, 2);
          
        % Check if distance is too large
        distance = norm(mesh_vertices(p, :) - points_vertices(q, :));
        if distance > distance_threshold/2
           keep(i) = false;
        end

    end
    pairs = pairs(keep, :);

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
