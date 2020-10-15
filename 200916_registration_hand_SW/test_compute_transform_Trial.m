
mesh_vertices = vertices;
points_vertices = points.vertices;
points_normals = points.normals;

transform = eye(4)
n = size(pairs, 1)

% Define derivative rotation matrix (linear approximation for small angles)
% 편미분 방정식
Rx = [0, 0, 0; 0, 0, -1; 0, 1, 0] % x 편미분
Ry = [0, 0, 1; 0, 0, 0; -1, 0, 0] % y 편미분
Rz = [0, -1, 0; 1, 0, 0; 0, 0, 0] % z 편미분

J = zeros(n, 7) % 1:3 for x, y, z 미분, 4:6 for translation, 7 for scale factor
b = zeros(n, 1) % 1:3 for x, y, z 미분, 4:6 for translation, 7 for scale factor

% question: J와 b를 왜 구분하는지 모르겠음
% J: normal과 mesh_vertices간 관계 파악
% b: normal과 mesh_vertices - scan_points간 관계 파악

% question: mesh_vertices의 normal과 scan_points의 normal을 mapping 하는게 낫지 않은지?


for i = 1 : n
    p = mesh_vertices(pairs(i, 1), :)
    q = points_vertices(pairs(i, 2), :)
    n = points_normals(pairs(i, 2), :)

    b(i) = n * (p - q)'; % b(i)가 cos(theta)를 의도한 것이라면 p-q vector의 normalization 필요

    J(i, 1) = n * (Rx * p');
    J(i, 2) = n * (Ry * p');
    J(i, 3) = n * (Rz * p');
    J(i, 4 : 6) = n;
    J(i, 7) = 2 * n * p'; % scale factor 왜 이렇게 구하는지 모르겠음

end

% Create and solve linear system - 장수영 교수님께 문의
JtJ = J' * J;
Jtb = J' * b;
result = JtJ \ -Jtb; % \연산은 JtJ의 역행렬을 의미 JtJ \ = inv(JtJ)

% Extract transform matrix
scale = (result(7) + 1) ^ 2; % scale factor 왜 이렇게 구하는지 모르겠음
rotation = ...
    rotation_matrix(result(1), [1, 0, 0]) * ...
    rotation_matrix(result(2), [0, 1, 0]) * ...
    rotation_matrix(result(3), [0, 0, 1]);
translation = result(4 : 6)';
transform = [diag(scale) * eye(3), translation'; 0, 0, 0, 1] * rotation;