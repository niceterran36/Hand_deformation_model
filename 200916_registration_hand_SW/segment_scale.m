function [V_tr, jCOR1_tr, jCOR2_tr] = segment_scale(V,F,jCOR1,jCOR2,target_length)
% jCOR1 = fixed COR, jCOR2 = transformed COR
% V = vertices
% F = faces
% target_length = amount of how much stretch the mesh in terms of the link
% length 

% Things to Do: whole hand segment adjustment

LM1 = [jCOR1; jCOR2];
C1 = jCOR1; C2 = jCOR2;
vector = C2-C1;
scale_factor = target_length/norm(vector);
vector = scale_factor*vector;
C2_n = C1+vector;
vector2 = C2_n - C1;
% fprintf()
norm(vector2)

LM2 = [C1; C2_n];
jCOR1_tr = C1;
jCOR2_tr = C2_n;

[b,bc] = bbw_boundary_conditions(V,F,LM1);
W = bbw_biharmonic_bounded(V,F,b,bc);
W = W./repmat(sum(W,2),1,size(W,2));

Diff = LM2 - LM1;
V_tr = bbw_simple_deform(V,F,LM1,W,Diff);

end 