clc 
clear all
addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath(genpath('external'));

% [V1, F1, FB1, Header1] = function_loading_ply_file('newTemplate_body_WS_200209.ply'); % Wonsup's template original
%V2 = V1+[0 145.758 867.945]; % vertex translation for modification

[V, F, FB1, Header1] = function_loading_ply_file('newTemplate_body_HY_200209.ply'); % modified template (vertex location)
%load('COR.mat'); % original template COR
load('COR_new.mat');
load('Segment_joint.mat');  % segment link information
V_idx = [1:size(V,1)]'; V2 = V; VLI = []; C = COR_new; A = V; % initial setting

% set right arm link structure
S_RA = zeros(3,2); S_RA = [18 19; 19 20; 20 21];
S_LA = zeros(3,2); S_LA = [14 15; 15 16; 16 17];
S_TL = zeros(11,2); S_TL = [1 2; 1 6; 1 10; 10 12; 12 13; 2 3; 3 4; 4 5; 6 7; 7 8; 8 9];


%% Initial visualization
figure()
hold on
axis equal
scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
plot3(C(:,1),C(:,2),C(:,3),'b*')
plot3(C([1 10 11 12 13],1),C([1 10 11 12 13],2),C([1 10 11 12 13],3), 'k-');
plot3(C(1:5,1),C(1:5,2),C(1:5,3), 'k-');
plot3(C([1 6:9],1),C([1 6:9],2),C([1 6:9],3), 'k-');
plot3(C(14:17,1),C(14:17,2),C(14:17,3), 'k-');
plot3(C(18:21,1),C(18:21,2),C(18:21,3), 'k-');
plot3(C([14 11 18],1),C([14 11 18],2),C([14 11 18],3), 'k-');
scatter3(V(1,1),V(1,2),V(1,3),'x','MarkerEdgeColor',[255/255, 0/255, 0/255])
hold off


%% Right arm separation 
% separation of upper/lower arms
% 3D points of plane for right arm separation
p1 = [-174.254 178.5431 1234.9798];
p2 = [-187.7131 176.2325 1410.1475];
p3 = [-174.0201 231.5602 1267.1921];
p4 = [-250.0 200.0 800.0];
p5 = [-250.0 -200.0 800.0];
[a1, b1, c1, d1] = generate_plane_3point(p1, p2, p3);
[a2, b2, c2, d2] = generate_plane_3point(p1, p4, p5);
PL = zeros(4,4);
PL(1,:) = [a1, b1, c1, d1];
PL(2,:) = [a2, b2, c2, d2];

stone = [-264.1,152.9,1141.9];

TV = PL(1,1)*stone(1) + PL(1,2)*stone(2) + PL(1,3)*stone(3) + PL(1,4);
TV2 = PL(2,1)*stone(1) + PL(2,2)*stone(2) + PL(2,3)*stone(3) + PL(2,4);

N1 = zeros(size(A,1),1);
N2 = zeros(size(A,1),1);

A = V;
Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = a1*A(i,1)+b1*A(i,2)+c1*A(i,3)+d1;
    Compare(i) = T;
end 
Compare(Compare<0) = 0; % convert negative value as zero
N1 = Compare;

Compare = [];
Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = a2*A(i,1)+b2*A(i,2)+c2*A(i,3)+d2;
    Compare(i) = T;
end 
Compare(Compare<0) = 0;
N2 = Compare;

NA = N1.*N2;
TT = find(NA); % right arm separation

v_rightarm = zeros(size(TT,2),3);
for i = 1:size(TT,1)
    K = V(TT(i),:);
    v_rightarm(i,:) = K;
end 

v_rightarm = [TT v_rightarm];

clear a1 a2 b1 b2 c1 c2 d1 d2 Compare K p1 p2 p3 p4 p5 PL stone T TV TV2

% figure()
% hold on
% axis equal
% scatter3(v_rightarm(:,2),v_rightarm(:,3),v_rightarm(:,4),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
% hold off

%% Left arm separation
% 3D points of plane for right arm separation
p1 = [176.6636 177.6067 1238.6726];
p2 = [175.9006 222.2333 1267.6049];
p3 = [185.0385 178.8094 1414.983];
p4 = [250.0 200.0 800.0];
p5 = [250.0 -200.0 800.0];
[a1, b1, c1, d1] = generate_plane_3point(p1, p2, p3);
[a2, b2, c2, d2] = generate_plane_3point(p1, p4, p5);
PL = zeros(4,4);
PL(1,:) = [a1, b1, c1, d1];
PL(2,:) = [a2, b2, c2, d2];

stone = [267.941,151.115,1153.05];

TV = PL(1,1)*stone(1) + PL(1,2)*stone(2) + PL(1,3)*stone(3) + PL(1,4);
TV2 = PL(2,1)*stone(1) + PL(2,2)*stone(2) + PL(2,3)*stone(3) + PL(2,4);

N3 = zeros(size(A,1),1);
N4 = zeros(size(A,1),1);

Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = a1*A(i,1)+b1*A(i,2)+c1*A(i,3)+d1;
    Compare(i) = T;
end 
Compare(Compare<0) = 0; % convert negative value as zero
N3 = Compare;

Compare = zeros(size(A,1),1);
for i = 1:size(A,1)
    T = a2*A(i,1)+b2*A(i,2)+c2*A(i,3)+d2;
    Compare(i) = T;
end 
Compare(Compare>0) = 0;
N4 = Compare;

NA2 = N3.*N4;
TT2 = find(NA2); % left arm separation

v_leftarm = zeros(size(TT2,2),3);
for i = 1:size(TT2,1)
    K = V(TT2(i),:);
    v_leftarm(i,:) = K;
end 

v_leftarm = [TT2 v_leftarm];

clear a1 a2 b1 b2 c1 c2 d1 d2 Compare K p1 p2 p3 p4 p5 PL stone T TV TV2

% figure()
% hold on
% axis equal
% scatter3(v_leftarm(:,2),v_leftarm(:,3),v_leftarm(:,4),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
% hold off
%% segmentation - Right arm

for xx = 1:size(v_rightarm,1) 
  vertexIdx = v_rightarm(xx,1);

F2 = F;
LI = F2 == vertexIdx;
[row2, ~] = find(LI);
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

    Sj = zeros(3,5); Sj(1:3,1) = [1:3]';
    vt = V(vertexIdx,:);

    for segment=1:3
        
        jna = C(S_RA(segment,1),:); jnb = C(S_RA(segment,2),:); 
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
                        LIX = Sj_c(:,5)<100; % cut with distance
                        Sj_c = Sj_c(LIX,:);
                        
                        if size(Sj_c,1) == 0
                            v_segment_rightarm(vertexIdx,1) = 4;
                        else
                            v_mindist = min(Sj_c(:,3));
                            [row,~] = find(Sj_c(:,3) == v_mindist); % searching min distance
                            v_delta = Sj_c(row,2); % final delta of vi
                            v_segment_rightarm(vertexIdx,1) = Sj_c(row,1); % final segment of vi
                        end
                        
                    else
                        v_segment_rightarm(vertexIdx,1) = 4;
                    end
                    
                else
                    v_segment_rightarm(vertexIdx,1) = 4;
                end
                
            else
                v_segment_rightarm(vertexIdx,1) = 4;
            end
            
        else
            v_segment_rightarm(vertexIdx,1) = 4;
        end

    else
        v_segment_rightarm(vertexIdx,1) = 4;
    end  
    
 v_mindist_s = min(Sj(:,5)); % cut with distance
end

% segment 4 treatment % Sg4 = [idx, x, y, z]
Sg4 = [];
clear temLI
temLI = v_segment_rightarm == 4;  Sg4(:,2:4) =  V(temLI,:);
Sg4(:,1) = V_idx(temLI);

for i = 1:size(Sg4,1)

    vertexIdx = Sg4(i);
        Sz = zeros(3,3); Sz(1:3,1) = [1:3]';

    for segment=1:3
        % projection-distance (Sz(:,2))
        vt = V(vertexIdx,:);
        jna = C(S_RA(segment,1),:); jnb = C(S_RA(segment,2),:); 
        vt_jna = vt - jna;
        jnb_jna = jnb - jna;
        delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2;
        av = vt-jna; 
        bv = jnb-jna; 
        cv = dot(av,bv)/norm(bv);
        dv = av - dot(av,bv)/norm(bv)^2 * bv; % projection vector
        dv2 = norm(dv);
        Sz(segment,2) = dv2;
        Sz(segment,4) = delta;
        
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
    [row3,~] = find(Sz(:,3) == v22_mindist1);
        
    if size(row3,1) == 1
        v_segment_rightarm(vertexIdx,1) = Sz(row3,1);
    else
        v22_mindist2 = [Sz(row3(1),2); Sz(row3(end),2)];
        v22_mindist2 = max(v22_mindist2);
        [row4,~] = find(Sz(:,2) == v22_mindist2);
        v_segment_rightarm(vertexIdx,1) = Sz(row4,1);
    end
    
end     

clear temLI
% v_segment_rightarm --> v_segment
% 1 --> 15, 2 --> 16, 3 --> 17
v_segment = zeros(size(V,1),1);
temLI = v_segment_rightarm == 1;
v_segment(temLI) = 15;
temLI = v_segment_rightarm == 2;
v_segment(temLI) = 16;
temLI = v_segment_rightarm == 3;
v_segment(temLI) = 17;

clear Sg4 Sj Sj_c Sz temLI v22_mindist1 v22_mindist2 v_delta v_mindist v_mindist_s vni vt vt_jna
clear jna jnb jnb_jna LI LIX min_d normals normals_F nv row row2 row3 row4 v VLI
clear cosTH cv delta dv dv2 av bv

%% segmentation - Left arm

for xx = 1:size(v_leftarm,1) 
  vertexIdx = v_leftarm(xx,1);

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

    Sj = zeros(3,5); Sj(1:3,1) = [1:3]';
    vt = V(vertexIdx,:);

    for segment=1:3
        
        jna = C(S_LA(segment,1),:); jnb = C(S_LA(segment,2),:); 
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
                        LIX = Sj_c(:,5)<100; % cut with distance
                        Sj_c = Sj_c(LIX,:);
                        
                        if size(Sj_c,1) == 0
                            v_segment_leftarm(vertexIdx,1) = 4;
                        else
                            v_mindist = min(Sj_c(:,3));
                            [row,col] = find(Sj_c(:,3) == v_mindist); % searching min distance
                            v_delta = Sj_c(row,2); % final delta of vi
                            v_segment_leftarm(vertexIdx,1) = Sj_c(row,1); % final segment of vi
                        end
                        
                    else
                        v_segment_leftarm(vertexIdx,1) = 4;
                    end
                    
                else
                    v_segment_leftarm(vertexIdx,1) = 4;
                end
                
            else
                v_segment_leftarm(vertexIdx,1) = 4;
            end
            
        else
            v_segment_leftarm(vertexIdx,1) = 4;
        end

    else
        v_segment_leftarm(vertexIdx,1) = 4;
    end  
    
 v_mindist_s = min(Sj(:,5)); % cut with distance
end

% segment 4 treatment % Sg4 = [idx, x, y, z]
Sg4 = [];
clear temLI
temLI = v_segment_leftarm == 4;  Sg4(:,2:4) =  V(temLI,:);
Sg4(:,1) = V_idx(temLI);

for i = 1:size(Sg4,1)

    vertexIdx = Sg4(i);
        Sz = zeros(3,3); Sz(1:3,1) = [1:3]';

    for segment=1:3
        % projection-distance (Sz(:,2))
        vt = V(vertexIdx,:);
        jna = C(S_LA(segment,1),:); jnb = C(S_LA(segment,2),:); 
        vt_jna = vt - jna;
        jnb_jna = jnb - jna;
        delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2;
        av = vt-jna; 
        bv = jnb-jna; 
        cv = dot(av,bv)/norm(bv);
        dv = av - dot(av,bv)/norm(bv)^2 * bv; % projection vector
        dv2 = norm(dv);
        Sz(segment,2) = dv2;
        Sz(segment,4) = delta;
        
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
        v_segment_leftarm(vertexIdx,1) = Sz(row3,1);
    else
        v22_mindist2 = [Sz(row3(1),2); Sz(row3(end),2)];
        v22_mindist2 = max(v22_mindist2);
        [row4,col4] = find(Sz(:,2) == v22_mindist2);
        v_segment_leftarm(vertexIdx,1) = Sz(row4,1);
    end
    
end     

clear temLI
% v_segment_rightarm --> v_segment
% 1 --> 15, 2 --> 16, 3 --> 17
temLI = v_segment_leftarm == 1;
v_segment(temLI) = 12;
temLI = v_segment_leftarm == 2;
v_segment(temLI) = 13;
temLI = v_segment_leftarm == 3;
v_segment(temLI) = 14;

clear Sg4 Sj Sj_c Sz temLI v22_mindist1 v22_mindist2 v_delta v_mindist v_mindist_s vni vt vt_jna
clear jna jnb jnb_jna LI LIX min_d normals normals_F nv row row2 row3 row4 v VLI
clear cosTH cv delta dv dv2 av bv


%% segmentation - torso & legs

% separation torso and leg vertices from the left & right arm
temLI = v_segment == 0;
v_torso_leg(:,2:4) =  V(temLI,:); 
v_torso_leg(:,1) = V_idx(temLI);


for xx = 1:size(v_torso_leg,1) 
  vertexIdx = v_torso_leg(xx,1);

F2 = F;
LI = F2 == vertexIdx;
[row2, ~] = find(LI);
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

    Sj = zeros(11,5); Sj(1:11,1) = [1:11]';

    for segment=1:11

        vt = V(vertexIdx,:);
        jna = C(S_TL(segment,1),:); jnb = C(S_TL(segment,2),:);
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
                        LIX = Sj_c(:,5)<100; % cut with distance
                        Sj_c = Sj_c(LIX,:);
                        
                        if size(Sj_c,1) == 0
                            v_segment(vertexIdx,1) = 18;
                        else
                            v_mindist = min(Sj_c(:,3));
                            [row,~] = find(Sj_c(:,3) == v_mindist); % searching min distance
                            v_delta = Sj_c(row,2); % final delta of vi
                            v_segment(vertexIdx,1) = Sj_c(row,1); % final segment of vi
                        end
                        
                    else
                        v_segment(vertexIdx,1) = 18;
                    end
                    
                else
                    v_segment(vertexIdx,1) = 18;
                end
                
            else
                v_segment(vertexIdx,1) = 18;
            end
            
        else
            v_segment(vertexIdx,1) = 18;
        end

    else
        v_segment(vertexIdx,1) = 18;
    end  
    
 v_mindist_s = min(Sj(:,5)); % cut with distance

end


% segment 21 treatment 
% Sg21 = [idx, x, y, z]

Sg18 = []; 
clear temLI
temLI = v_segment == 18;  Sg18(:,2:4) =  V2(temLI,:);
Sg18(:,1) = V_idx(temLI);


for i = 1:size(Sg18,1)

    vertexIdx = Sg18(i);

        Sz = zeros(11,3); Sz(1:11,1) = [1:11]';

    for segment=1:11

        % projection-distance (Sz(:,2))
        vt = V(vertexIdx,:);
        jna = C(S_TL(segment,1),:); jnb = C(S_TL(segment,2),:); 
        vt_jna = vt - jna;
        jnb_jna = jnb - jna;
        delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2;
        av = vt-jna; 
        bv = jnb-jna; 
        cv = dot(av,bv)/norm(bv);
        dv = av - dot(av,bv)/norm(bv)^2 * bv; % projection vector
        dv2 = norm(dv);
        Sz(segment,2) = dv2;
        Sz(segment,4) = delta;
        
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
    [row3,~] = find(Sz(:,3) == v22_mindist1);
        
    if size(row3,1) == 1
        v_segment(vertexIdx,1) = Sz(row3,1);
    else
        v22_mindist2 = [Sz(row3(1),2); Sz(row3(end),2)];
        v22_mindist2 = max(v22_mindist2);
        [row4,~] = find(Sz(:,2) == v22_mindist2);
        v_segment(vertexIdx,1) = Sz(row4,1);
    end
    
end     

clear Sg18 Sj Sj_c Sz temLI v22_mindist1 v22_mindist2 v_delta v_mindist v_mindist_s vni vt vt_jna
clear jna jnb jnb_jna LI LIX min_d normals normals_F nv row row2 row3 row4 v VLI
clear cosTH cv delta dv dv2 av bv

%% Assign edge near points to the edge segments

S_edge = [5; 8; 11; 14; 17]; % S_edge(1) = head, S_edge(2) = left foot, S_edge(3) = right foot, S_edge(4) = left hand, S_edge(5) = right hand
edge_points = zeros(size(S_edge,1),3);
for i = 1:size(S_edge,1)
edge_points(i,:) = C(S_edge(i),:);
end 

V_idx = [1:size(V,1)]';
V_seg = [V_idx, V, v_segment];
LIX = v_segment == 18; % 18 segment

V_seg = V_seg(LIX,:);
% Vseg_1 = V_seg(:,1); Vseg_2 = V_seg(:,2); Vseg_3 = V_seg(:,3); Vseg_4 = V_seg(:,4); Vseg_5 = V_seg(:,5);
% V_seg = [Vseg_1(LIX) Vseg_2(LIX) Vseg_3(LIX) Vseg_4(LIX) Vseg_5(LIX)];
dist_v_eg = zeros(6,1);

for i = 1:size(V_seg,1)

dist_v_eg(1) = norm(V_seg(i,2:4)-edge_points(1,:));
dist_v_eg(2) = norm(V_seg(i,2:4)-edge_points(2,:));
dist_v_eg(3) = norm(V_seg(i,2:4)-edge_points(3,:));
dist_v_eg(4) = norm(V_seg(i,2:4)-edge_points(4,:));
dist_v_eg(5) = norm(V_seg(i,2:4)-edge_points(5,:));
dist_v_eg_min = min(dist_v_eg(:,1));
[row,col] = find(dist_v_eg(:,1) == dist_v_eg_min);

if dist_v_eg_min <= 100
   if row == 1
      V_seg(i,5) = 5;
   elseif row == 2
      V_seg(i,5) = 8;
   elseif row == 3
      V_seg(i,5) = 11; 
   elseif row == 4
      V_seg(i,5) = 14;
   else
       V_seg(i,5) = 17;
   end
else   
   V_seg(i,5) = 18;  
end 

end 

clear dist_v_eg 

%% V_segment update
 
for i = 1:size(V_seg,1)
    v_segment(V_seg(i,1),1) = V_seg(i,5);
end 


%%
V2 = V;
VLI = [];
for segment=1:18
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

%%
figure()
hold on
axis equal
scatter3(Sg1(:,1),Sg1(:,2),Sg1(:,3),'.', 'MarkerEdgeColor',[16/255, 241/255, 255/255])
scatter3(Sg2(:,1),Sg2(:,2),Sg2(:,3),'.', 'MarkerEdgeColor',[213/255, 42/255, 219/255])
scatter3(Sg3(:,1),Sg3(:,2),Sg3(:,3),'.', 'MarkerEdgeColor',[69/255, 204/255, 104/255])     % lower torso
scatter3(Sg4(:,1),Sg4(:,2),Sg4(:,3),'.', 'MarkerEdgeColor',[233/255, 30/255, 68/255])      % upper torso
scatter3(Sg5(:,1),Sg5(:,2),Sg5(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 255/255])
scatter3(Sg6(:,1),Sg6(:,2),Sg6(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])      % left thigh
scatter3(Sg7(:,1),Sg7(:,2),Sg7(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg8(:,1),Sg8(:,2),Sg8(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg9(:,1),Sg9(:,2),Sg9(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])      % right thigh
scatter3(Sg10(:,1),Sg10(:,2),Sg10(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])
scatter3(Sg11(:,1),Sg11(:,2),Sg11(:,3),'.', 'MarkerEdgeColor',[242/255, 62/255, 27/255])
scatter3(Sg12(:,1),Sg12(:,2),Sg12(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])   % upper arm
scatter3(Sg13(:,1),Sg13(:,2),Sg13(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])   % lower arm
scatter3(Sg14(:,1),Sg14(:,2),Sg14(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255])      % left hand
scatter3(Sg15(:,1),Sg15(:,2),Sg15(:,3),'.', 'MarkerEdgeColor',[191/255, 247/255, 20/255])   % upper arm
scatter3(Sg16(:,1),Sg16(:,2),Sg16(:,3),'.', 'MarkerEdgeColor',[247/255, 170/255, 20/255])   % lower arm
scatter3(Sg17(:,1),Sg17(:,2),Sg17(:,3),'.', 'MarkerEdgeColor',[255/255, 0/255, 0/255])      % right hand
scatter3(Sg18(:,1),Sg18(:,2),Sg18(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
hold off

% [191/255, 247/255, 255/255] Skyblue 하늘색
% [16/255, 241/255, 255/255] Cyan
% [69/255, 204/255, 104/255] Green
% [233/255, 30/255, 68/255] Red
% [179/255, 59/255, 235/255] Purple
% [213/255, 42/255, 219/255] Magenta

%%
figure()
hold on
axis equal
scatter3(Sg21(:,1),Sg21(:,2),Sg21(:,3),'.', 'MarkerEdgeColor',[0, 0, 0])
hold off

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

plot3(C(:,1),C(:,2),C(:,3),'b*')
plot3(C([1 10 11 12 13],1),C([1 10 11 12 13],2),C([1 10 11 12 13],3), 'k-');
plot3(C(1:5,1),C(1:5,2),C(1:5,3), 'k-');
plot3(C([1 6:9],1),C([1 6:9],2),C([1 6:9],3), 'k-');
plot3(C(14:17,1),C(14:17,2),C(14:17,3), 'k-');
plot3(C(18:21,1),C(18:21,2),C(18:21,3), 'k-');
plot3(C([14 11 18],1),C([14 11 18],2),C([14 11 18],3), 'k-');
hold off

%% 3D Rendering

h1 = trisurf(F, V(:,1), V(:,2), V(:,3));
    set(h1, 'FaceColor',[1 0.88 0.77]) %face color matrix 생성하여 개별값 삽입 가능
    set(h1, 'EdgeColor', 'none');
    set(h1, 'facealpha', 1);
    
    view(2);
    axis equal;
    light('Position', [3 5 7], 'Style', 'infinite');
    lighting gouraud;
    material dull;
    
    hold on;



