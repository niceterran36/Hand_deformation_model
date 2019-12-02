clc
clear all;
addpath functions;
addpath(genpath('external'));

%% Data import (mesh, CoR)
[V, F, FB, H] = function_loading_ply_file('hand_meshmodel_190730.ply');
load('centers_new.mat');
% 1~4: little, 5~8: ring, 9~12: middle, 13~16: index, 17~20: thumb, 
% 22: wrist center
% centers_new 4th column = finger parts (5: little, 4: ring, 3: middle, 2:
% index, 1: thumb, 0: palm
% centers)new 5th column = hierarchy index

% bone segment generation (Things to do, save bone hierarchy)
load('bone_segment.mat')

%% Hierarchy setting






%%  vertex seperation

for vertexIdx = 1:size(V,1);
    
    F2 = F;
    LI = F2 == vertexIdx;
    [row2, col2] = find(LI);
    F2 = F2(row2,:);
    for i = 1:size(F2,1)*3
        v(i,1) = F2(i);
    end   
    nv = vertexIdx;
    for i = 1:size(v,1)
        if v(i) ~= nv
           nv = [nv v(i)]; % nv: List of neighbor vertices of target with 1-node
        end
    end
    normals = getNormals(V, F);
    normals_F = normals(row2,:);
    normals_F = sum(normals_F);
    normals_F = normals_F/norm(normals_F);
    vni = normals_F; % weighted normal vector of vi

    Sj = zeros(21,4); Sj(1:21,1) = [1:21]';

    for segment=1:21
    vt = V(vertexIdx,:);
    jna = A(S(segment,1),:); jnb = A(S(segment,2),:); 
    vt_jna = vt - jna;
    jnb_jna = jnb - jna;
    delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2;
    Sj(segment,2) = delta;

    av = vt-jna; 
    bv = jnb-jna; 
    cv = dot(av,bv)/norm(bv);
    dv1 = sqrt(norm(av)^2 - norm(cv)^2);
    dv = av - dot(av,bv)/norm(bv)^2 * bv; % projection vector
    dv2 = norm(dv);
    Sj(segment,3) = dv2;
    dv = dv./dv2;
    cosTH = dot(dv,vni)/(norm(dv)*norm(vni));
    Sj(segment,4) = cosTH;
    end
    
end

