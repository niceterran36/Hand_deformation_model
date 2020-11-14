function [V_tr, CP2] = segment_scale_finger(vertices,faces,CP1,CPs1)
% jCOR1 = fixed COR, jCOR2 = transformed COR
% V = vertices
% F = faces
% CP1: template CoR groups
% CPs1: scan's CoR groups
% target_length = amount of how much stretch the mesh in terms of the link
% length 

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);

vector1s = CPs1(2,:) - CPs1(1,:);
vector2s = CPs1(3,:) - CPs1(2,:);

target_length1 = norm(vector1s);
target_length2 = norm(vector2s);

scale_factor1 = target_length1/norm(vector1);
scale_factor2 = target_length2/norm(vector2);

vector1_n = scale_factor1*vector1;
vector2_n = scale_factor2*vector2;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
CP2 = [CP1(1,:); C2_n; C3_n];

[b,bc] = bbw_boundary_conditions(vertices, faces, CP1);
W = bbw_biharmonic_bounded(vertices, faces, b, bc);
W = W./repmat(sum(W,2),1,size(W,2));
Diff = CP2 - CP1;
V_tr = bbw_simple_deform(vertices, faces, CP1, W, Diff);

end 