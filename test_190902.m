addpath(genpath('external'));

[T,N,B,I] = per_vertex_frames(V,F,varargin)
N = per_vertex_normals(V,F);

figure(1)
scatter3(A(:,1), A(:,2), A(:,3), '.', 'MarkerEdgeColor',[0.3, 0.3, 0.3])
scatter3(A(1,1), A(1,2), A(1,3), '*', 'MarkerEdgeColor',[1, 0, 0])
plot3(

mesh = load('hy_mesh.mat');
A = mesh.mesh.vertices;
B = mesh.mesh.faces;
dt = delaunayTriangulation(A)
[Tfb, Xfb] = freeBoundary(dt)
tr = triangulation(Tfb,Xfb)
vn = vertexNormal(tr)

V1 = vertexNormal(A)
V = vertexNormal(TR,ID)

TR = 
ID = 

%      tr = triangulation(Tfb,Xfb); - Xfb : points, Tfb : index of triangle
%      vn = vertexNormal(tr);

%   Example 3:
%       % Direct query of a 3D Delaunay triangulation created using
%       % delaunayTriangulation. Compute the free boundary as in Example 2
%       Points = rand(50,3);
%       dt = delaunayTriangulation(Points);
%       [tri, Xb] = freeBoundary(dt);
%       %Plot the surface mesh
%       trisurf(tri, Xb(:,1), Xb(:,2), Xb(:,3), 'FaceColor', 'cyan','FaceAlpha', 0.8);


        points_normals = normals(vertices, faces);
        points_normals = normalizerow(points_normals(indices, :));
        
