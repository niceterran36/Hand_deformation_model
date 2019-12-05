clc
clear all;
addpath functions;
addpath(genpath('external'));

%% Data import (mesh, CoR)
[V, F, FB, H] = function_loading_ply_file('hand_meshmodel_190730.ply');
load('centers.mat');
% 1~4: little, 5~8: ring, 9~12: middle, 13~16: index, 17~20: thumb, 
% 22: wrist center
A = centers;
C = V;

% bone segment generation (Things to do, save bone hierarchy)
load('bone_segment.mat')
V_idx = [1:size(V,1)]';
V2 = V;
VLI = [];

%% Visualization of hand
vertexIdx = 42;

A = centers;
figure() % point cloud 3D plotting
hold on
axis equal
plot3(A(:,1),A(:,2),A(:,3),'b*')
plot3(A(1:4,1), A(1:4,2), A(1:4,3),'k-')
plot3(A(5:8,1), A(5:8,2), A(5:8,3),'k-')
plot3(A(9:12,1), A(9:12,2), A(9:12,3),'k-')
plot3(A(13:16,1), A(13:16,2), A(13:16,3),'k-')
plot3(A(17:20,1), A(17:20,2), A(17:20,3),'k-')
plot3(A([4 22],1),A([4 22],2),A([4 22],3),'b-')
plot3(A([8 22],1),A([8 22],2),A([8 22],3),'b-')
plot3(A([12 22],1),A([12 22],2),A([12 22],3),'b-')
plot3(A([16 22],1),A([16 22],2),A([16 22],3),'b-')
plot3(A([20 22],1),A([20 22],2),A([20 22],3),'b-')
plot3(A([22 27],1),A([22 27],2),A([22 27],3),'k-')
scatter3(V(:,1),V(:,2),V(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
scatter3(V(vertexIdx,1),V(vertexIdx,2),V(vertexIdx,3), 'o', 'MarkerEdgeColor',[255/255, 0/255, 0/255])
hold off

C = V;
figure(1) % point cloud 3D plotting
hold on
axis equal
scatter3(C(:,1),C(:,2),C(:,3),'.', 'MarkerEdgeColor',[217/255, 217/255, 217/255])
B = centers;
scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
hold off

%% Test code for vertex vi

% jna = centers(3,:);
% jnb = centers(2,:);
% %vt = [-69.7 -22.5 60.9]; % after the segment 
% vt = [-61.20 -20.35 44.06]; % between the segment
% %vt = [-47.85 -0.99 -10.44] % before the segment
% vt_jna = vt - jna;
% jnb_jna = jnb - jna;
% delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2;
% 
% % distance of projection vector from vi to segment 
% % vt: target vertex 
% av = vt-jna; bv = jnb-jna; cv = dot(av,bv)/norm(bv); % vector define
% dv1 = sqrt(norm(av)^2 - norm(cv)^2); % distance by trigonometric functions
% dv = av - dot(av,bv)/norm(bv)^2 * bv; % distance by vector & norm 
% dv2 = norm(dv);
% 
% % neighboring vertex info in the FACE column
% % vertexIdx = 12;
% vertexIdx = 1281;
% 
% F2 = F;
% LI = F2 == vertexIdx;
% [row, col] = find(LI);
% F2 = F2(row,:);
% 
% for i = 1:size(F2,1)*3
%     v(i,1) = F2(i);
% end
%     
% nv = vertexIdx; % target vertex
% for i = 1:size(v,1)
%     if v(i) ~= nv
%        nv = [nv v(i)]; % nv: List of neighbor vertices of target with 1-node
%     end
% end
% 
% % vertex normal calculation (unit vector by sum of the around face normal)
% 
% normals = getNormals(V, F);
% normals_F = normals(row,:);
% normals_F = sum(normals_F);
% normals_F = normals_F/norm(normals_F);
% vni = normals_F;
% 
% % angle(theta) between dv and vertex normal
% % dv: projection vector, vni: vertex normal of vi
% dv = dv./dv2;
% 
% cosTH = dot(dv,vni)/(norm(dv)*norm(vni)); 


%%  vertex seperation
tic
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

    Sj = zeros(21,5); Sj(1:21,1) = [1:21]';

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
        %dv1 = sqrt(norm(av)^2 - norm(cv)^2);
        dv = av - dot(av,bv)/norm(bv)^2 * bv; % projection vector
        dv2 = norm(dv);
        Sj(segment,3) = dv2;
        dv = dv./dv2;
        cosTH = dot(dv,vni)/(norm(dv)*norm(vni));
        Sj(segment,4) = cosTH;

        % min-distance
        if delta >= 0 && delta <= 1
           min_d = dv2;
        elseif delta < 0
           min_d = norm(vt-jna);
        else
           min_d = norm(vt-jnb);
        end
        Sj(segment,5) = min_d;

    end

Sj_c = Sj;

    if size(Sj_c,1) ~= 0
        LIX = Sj_c(:,4)>=0; %angle condition1
        Sj_c = Sj(LIX,:);

        if size(Sj_c,1) ~= 0
            LIX = Sj_c(:,4)<1; % angle condition2
            Sj_c = Sj_c(LIX,:);
            
            if size(Sj_c,1) ~= 0
                LIX = Sj_c(:,2)>=0; % cut with delta 
                Sj_c = Sj_c(LIX,:);
                
                if size(Sj_c,1) ~= 0
                    LIX = Sj_c(:,2)<1; % cut with delta
                    Sj_c = Sj_c(LIX,:);
                                        
                    if size(Sj_c,1) ~= 0
                        LIX = Sj_c(:,5)<17; % cut with distance
                        Sj_c = Sj_c(LIX,:);
                        
                        if size(Sj_c,1) == 0;
                            v_segment(vertexIdx,1) = 22;
                        else
                            v_mindist = min(Sj_c(:,3));
                            [row,col] = find(Sj_c(:,3) == v_mindist); % searching min distance
                            v_delta = Sj_c(row,2); % final delta of vi
                            v_segment(vertexIdx,1) = Sj_c(row,1); % final segment of vi
                        end
                        
                    else
                        v_segment(vertexIdx,1) = 22;
                    end
                    
                else
                    v_segment(vertexIdx,1) = 22;
                end
                
            else
                v_segment(vertexIdx,1) = 22;
            end
            
        else
            v_segment(vertexIdx,1) = 22;
        end

    else
        v_segment(vertexIdx,1) = 22;
    end  
    
 v_mindist_s = min(Sj(:,5)); % cut with distance
% [row,col] = find(Sj(:,5) == v_mindist_s);
% LIX = Sj_c(:,5) == v_mindist_s;
% Sj_1 = Sj_c(:,1); Sj_2 = Sj_c(:,2); Sj_3 = Sj_c(:,3); Sj_4 = Sj_c(:,4); Sj_5 = Sj_c(:,5);
% Sj_c = [Sj_1(LIX) Sj_2(LIX) Sj_3(LIX) Sj_4(LIX) Sj_5(LIX)];

end


% segment 22 treatment 
% Sg22 = [idx, x, y, z]

Sg22 = []; 
temLI = v_segment == 22;  Sg22(:,2:4) =  V2(temLI,:);
Sg22(:,1) = V_idx(temLI);


for i = 1:size(Sg22,1)

    vertexIdx = Sg22(i);
%     vertexIdx = 17

        Sz = zeros(21,3); Sz(1:21,1) = [1:21]';

    for segment=1:21

        % projection-distance (Sz(:,2))
        vt = V(vertexIdx,:);
        jna = A(S(segment,1),:); jnb = A(S(segment,2),:); 
        vt_jna = vt - jna;
        jnb_jna = jnb - jna;
        delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2;
        av = vt-jna; 
        bv = jnb-jna; 
        cv = dot(av,bv)/norm(bv);
        dv = av - dot(av,bv)/norm(bv)^2 * bv; % projection vector
        dv2 = norm(dv);
        Sz(segment,2) = dv2;
        
        % min-distance (Sz(:,3))
        if delta >= 0 && delta <= 1
           min_d = dv2;
        elseif delta < 0
           min_d = norm(vt-jna);
        else
           min_d = norm(vt-jnb);
        end
        Sz(segment,3) = min_d;

    end
    
    v22_mindist1 = min(Sz(:,3));
    [row3,col3] = find(Sz(:,3) == v22_mindist1);
        
    if size(row3,1) == 1
        v_segment(vertexIdx,1) = Sz(row3,1);
    else
        v22_mindist2 = [Sz(row3(1),2); Sz(row3(end),2)];
        v22_mindist2 = min(v22_mindist2);
        [row4,col4] = find(Sz(:,2) == v22_mindist2);
        v_segment(vertexIdx,1) = Sz(row4,1);
    end
    
end     
toc

%% Assign tip near points to the edge segments

S_edge = [8; 11; 14; 17; 20; 21]; % S_edge(1) = thumb, ~ S_edge(5) = little, S_edge(6) = wrist
edge_points = zeros(size(S_edge,1),3);
for i = 1:size(S_edge,1);
edge_points(i,:) = centers(S(S_edge(i),2),:);
end 

V_idx = [1:size(V,1)]';
V_seg = [V_idx, V, v_segment];
LIX = v_segment == 22; % 22 segment
Vseg_1 = V_seg(:,1); Vseg_2 = V_seg(:,2); Vseg_3 = V_seg(:,3); Vseg_4 = V_seg(:,4); Vseg_5 = V_seg(:,5);
V_seg = [Vseg_1(LIX) Vseg_2(LIX) Vseg_3(LIX) Vseg_4(LIX) Vseg_5(LIX)];
dist_v_eg = zeros(6,1);

for i = 1:size(V_seg,1)

dist_v_eg(1) = norm(V_seg(i,2:4)-edge_points(1,:));
dist_v_eg(2) = norm(V_seg(i,2:4)-edge_points(2,:));
dist_v_eg(3) = norm(V_seg(i,2:4)-edge_points(3,:));
dist_v_eg(4) = norm(V_seg(i,2:4)-edge_points(4,:));
dist_v_eg(5) = norm(V_seg(i,2:4)-edge_points(5,:));
dist_v_eg(6) = norm(V_seg(i,2:4)-edge_points(6,:));
dist_v_eg_min = min(dist_v_eg(:,1));
[row,col] = find(dist_v_eg(:,1) == dist_v_eg_min);

if dist_v_eg_min <= 50
   if row == 1
      V_seg(i,5) = 8;
   elseif row == 2
      V_seg(i,5) = 11;
   elseif row == 3
      V_seg(i,5) = 14; 
   elseif row == 4
      V_seg(i,5) = 17;
   elseif row == 5
       V_seg(i,5) = 20;
   else
       V_seg(i,5) = 21;
   end
else   
   V_seg(i,5) = 22;  
end 

end 

%% finger tip assignment modification 

S_edge = [8; 11; 14; 17; 20]; % S_edge(1) = thumb, ~ S_edge(5) = little, S_edge(6) = wrist
edge_points = zeros(size(S_edge,1),3);
for i = 1:size(S_edge,1);
edge_points(i,:) = centers(S(S_edge(i),2),:);
end 

V_idx = [1:size(V,1)]';
V_seg = [V_idx, V, v_segment];
dist_v_eg = zeros(5,1);

tic
for i = 1:size(V_seg,1)

dist_v_eg(1) = norm(V_seg(i,2:4)-edge_points(1,:));
dist_v_eg(2) = norm(V_seg(i,2:4)-edge_points(2,:));
dist_v_eg(3) = norm(V_seg(i,2:4)-edge_points(3,:));
dist_v_eg(4) = norm(V_seg(i,2:4)-edge_points(4,:));
dist_v_eg(5) = norm(V_seg(i,2:4)-edge_points(5,:));
dist_v_eg_min = min(dist_v_eg(:,1));
[row,col] = find(dist_v_eg(:,1) == dist_v_eg_min);

if dist_v_eg_min <= 10
   if row == 1
      V_seg(i,5) = 8;
   elseif row == 2
      V_seg(i,5) = 11;
   elseif row == 3
      V_seg(i,5) = 14;
   elseif row == 4
      V_seg(i,5) = 17;
   elseif row == 5
       V_seg(i,5) = 20; 
   else
       V_seg(i,5) = 21;
   end
else   
   V_seg(i,5) = V_seg(i,5);
end 

end 
toc

%% v_segment update 

for i = 1:size(V_seg,1)
    v_segment(V_seg(i,1),1) = V_seg(i,5);
end 

%% current finger check

% vertexIdx = 1
% 
% mindist_seg = zeros(size(S,1),3);
% 
% for segment=1:21
%     
% vt = V(vertexIdx,:);
% jna = A(S(segment,1),:); jnb = A(S(segment,2),:); 
% vt_jna = vt - jna;
% jnb_jna = jnb - jna;
% delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2;
% Sj(segment,2) = delta;
% 
% av = vt-jna; 
% bv = jnb-jna; 
% cv = dot(av,bv)/norm(bv);
% dv1 = sqrt(norm(av)^2 - norm(cv)^2);
% dv = av - dot(av,bv)/norm(bv)^2 * bv; % projection vector
% dv2 = norm(dv);
% Sj(segment,3) = dv2;
% dv = dv./dv2;
% cosTH = dot(dv,vni)/(norm(dv)*norm(vni));
% Sj(segment,4) = cosTH;
% end
%     
%     
%     
% sd_ms_vi = zeros(size(S,1),1);
% % sd2mps : shortest distance between vi and mid point seg.
% 
% temLI = v_segment == 2; Sg2 = V_seg(temLI,:);
% 
% for j=1:size(Sg2,1);
% vertexIdx = Sg2(j);
% 
% for i = 1:21
%     sd_ms_vi(i,1) = norm(Mid_point_seg(i,:) - V(vertexIdx,:));
% end
% min_sdmd = min(sd_ms_vi);
% [row_1,col_1] = find(sd_ms_vi(:,1) == min_sdmd); % searching min distance
% 
% % search current fingers
% if row_1 > 0 && row_1 <= 5
%    current_finger = 0;
% elseif row_1 > 5 && row_1 <= 8
%    current_finger = 1;
% elseif row_1 > 8 && row_1 <= 11
%    current_finger = 2;
% elseif row_1 > 11 && row_1 <= 14
%    current_finger = 3;
% elseif row_1 > 14 && row_1 <= 17
%    current_finger = 4;
% elseif row_1 > 17 && row_1 <= 20
%    current_finger = 5;
% else
%     current_finger = 0;
% end
% 
% Sg2(j,6) = current_finger;
% end
% 
% 
% 
% LIF = Sf(:,2) == current_finger;
% Sf_1 = Sf(:,1); Sf_2 = Sf(:,2);
% Sf_2 = [Sf_1(LIF) Sf_2(LIF)];



%% Segment visualization

V2 = V;
% segment = 1;
VLI = [];
for segment=1:22
temLI = v_segment == segment;
VLI = [VLI temLI];
end 

temLI = v_segment == 1; Sg1 = V2(temLI,:);
temLI = v_segment == 2; Sg2 = V2(temLI,:);
temLI = v_segment == 3; Sg3 = V2(temLI,:);
temLI = v_segment == 4; Sg4 = V2(temLI,:);
temLI = v_segment == 5; Sg5 = V2(temLI,:);
temLI = v_segment == 6; Sg6 = V2(temLI,:);
temLI = v_segment == 7; Sg7 = V2(temLI,:);
temLI = v_segment == 8; Sg8 = V2(temLI,:);
temLI = v_segment == 9; Sg9 = V2(temLI,:);
temLI = v_segment == 10; Sg10 = V2(temLI,:);
temLI = v_segment == 11; Sg11 = V2(temLI,:);
temLI = v_segment == 12; Sg12 = V2(temLI,:);
temLI = v_segment == 13; Sg13 = V2(temLI,:);
temLI = v_segment == 14; Sg14 = V2(temLI,:);
temLI = v_segment == 15; Sg15 = V2(temLI,:);
temLI = v_segment == 16; Sg16 = V2(temLI,:);
temLI = v_segment == 17; Sg17 = V2(temLI,:);
temLI = v_segment == 18; Sg18 = V2(temLI,:);
temLI = v_segment == 19; Sg19 = V2(temLI,:);
temLI = v_segment == 20; Sg20 = V2(temLI,:);
temLI = v_segment == 21; Sg21 = V2(temLI,:);
temLI = v_segment == 22; Sg22 = V2(temLI,:);
%%
figure()
hold on
axis equal
scatter3(Sg1(:,1),Sg1(:,2),Sg1(:,3),'.', 'MarkerEdgeColor',[16/255, 241/255, 255/255])
scatter3(Sg2(:,1),Sg2(:,2),Sg2(:,3),'.', 'MarkerEdgeColor',[213/255, 42/255, 219/255])
scatter3(Sg3(:,1),Sg3(:,2),Sg3(:,3),'.', 'MarkerEdgeColor',[233/255, 30/255, 68/255])
scatter3(Sg4(:,1),Sg4(:,2),Sg4(:,3),'.', 'MarkerEdgeColor',[179/255, 59/255, 235/255])
scatter3(Sg5(:,1),Sg5(:,2),Sg5(:,3),'.', 'MarkerEdgeColor',[69/255, 204/255, 104/255])
scatter3(Sg6(:,1),Sg6(:,2),Sg6(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])
scatter3(Sg7(:,1),Sg7(:,2),Sg7(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg8(:,1),Sg8(:,2),Sg8(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg9(:,1),Sg9(:,2),Sg9(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 255/255])
scatter3(Sg10(:,1),Sg10(:,2),Sg10(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg11(:,1),Sg11(:,2),Sg11(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg12(:,1),Sg12(:,2),Sg12(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])
scatter3(Sg13(:,1),Sg13(:,2),Sg13(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg14(:,1),Sg14(:,2),Sg14(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg15(:,1),Sg15(:,2),Sg15(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 255/255])
scatter3(Sg16(:,1),Sg16(:,2),Sg16(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg17(:,1),Sg17(:,2),Sg17(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg18(:,1),Sg18(:,2),Sg18(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])
scatter3(Sg19(:,1),Sg19(:,2),Sg19(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg20(:,1),Sg20(:,2),Sg20(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg21(:,1),Sg21(:,2),Sg21(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
%scatter3(Sg22(:,1),Sg22(:,2),Sg22(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
hold off
%%
figure()
hold on
axis equal
scatter3(Sg22(:,1),Sg22(:,2),Sg22(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
hold off











