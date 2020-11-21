function M = mean_dist_tester(mesh, points, FRP_dorsal_segment, assignment_new)

vertices = mesh.vertices;
faces = mesh.faces;
normals = per_vertex_normals(vertices, faces);
keep = ismember(assignment_new, FRP_dorsal_segment);
[vertices, faces] = filter_vertices(vertices, faces, keep);
normals = normals(keep, :);
pairs = compute_correspondences_new(vertices, normals, points.vertices, points.normals,20,cos(30*pi/180));

dist_array = [];
verticesA = vertices; verticesB = points.vertices;
for i = 1:size(pairs,1)
    dist = norm(verticesA(pairs(i,1),:)-verticesB(pairs(i,2),:));
    dist_array = [dist_array; dist];
end 
M = sum(dist_array)/size(pairs,1);
fprintf('mean distance = %.2f\n',M);

end