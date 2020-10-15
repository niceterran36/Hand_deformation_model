function transform = compute_transformation(mesh_vertices, points_vertices, points_normals, pairs)
%COMPUTE_TRANSFORMATION Compute incremental transformation to match point cloud

    % Check arguments
    transform = eye(4);
    n = size(pairs, 1);
    if n == 0
        return;
    end
    
    % Define derivative rotation matrix (linear approximation for small angles)
    Rx = [0, 0, 0; 0, 0, -1; 0, 1, 0]; 
    Ry = [0, 0, 1; 0, 0, 0; -1, 0, 0];
    Rz = [0, -1, 0; 1, 0, 0; 0, 0, 0]; 
    
    % Build jacobian matrix and right-hand term
    J = zeros(n, 7);
    b = zeros(n, 1);
    for i = 1 : n
        p = mesh_vertices(pairs(i, 1), :); % template vertex
        q = points_vertices(pairs(i, 2), :); % target scan vertex
        n = points_normals(pairs(i, 2), :); % target scan normals
        
        b(i) = n * (p - q)'; % vector difference & normal ���� ����
        
        J(i, 1) = n * (Rx * p'); % x-angle factor
        J(i, 2) = n * (Ry * p'); % y-angle factor
        J(i, 3) = n * (Rz * p'); % z-angle factor
        J(i, 4 : 6) = n; % translation factor
        J(i, 7) = 2 * n * p'; % scale factor
    end
    
    % Create and solve linear system: ����� ������ ����
    JtJ = J' * J;
    Jtb = J' * b;
    result = JtJ \ -Jtb; %\�� ����� ���� �Լ�, inv(JtJ)�� ����
    
    % Extract transform matrix - 4 x 4 form���� scale, rotation, translation
    % ��
    scale = (result(7) + 1) ^ 2;
    rotation = ...
        rotation_matrix(result(1), [1, 0, 0]) * ...
        rotation_matrix(result(2), [0, 1, 0]) * ...
        rotation_matrix(result(3), [0, 0, 1]);
    translation = result(4 : 6)';
    transform = [diag(scale) * eye(3), translation'; 0, 0, 0, 1] * rotation;

end

