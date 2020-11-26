function Pjt_pt = vector_projection(o, pt2, pt3, target_pt)
% Pjt_vector: projected vector to plane
% pt1 = center(o), pt2 = axis1 pt, pt3 = axis2 pt

% o = A(16,:);
% pt2 = A(16,:) + axes{6}(1 : 3, 1)';
% pt3 = A(16,:) + axes{6}(1 : 3, 2)';
% target_pt = B(15,:);

% o = A(12,:);
% pt2 = A(12,:) + axes_d{9}(1 : 3, 1)';
% pt3 = A(12,:) + axes_d{9}(1 : 3, 2)';
% target_pt = B(11,:);

% o = A(4,:);
% pt2 = A(4,:) + axes_d{15}(1 : 3, 1)';
% pt3 = A(4,:) - axes_d{15}(1 : 3, 2)';
% target_pt = B(3,:);

[a, b, c, d] = generate_plane_3point(o, pt2, pt3);
Pn = [a,b,c]; % plane normal
Pn = Pn/norm(Pn);
target_vector = target_pt - o;

dist = dot(target_vector,Pn); % dot product of target vector and plane normal
Pjt_pt = Pn * -dist + target_pt;
p_to_plane_dist = a*Pjt_pt(1) + b*Pjt_pt(2) + c*Pjt_pt(3) + d;

    if p_to_plane_dist > 0.1
       Pjt_pt = Pn * dist + target_pt;
    else 
       Pjt_pt = Pn * -dist + target_pt;
    end 

end 

function [a, b, c, d] = generate_plane_3point(p1, p2, p3)

% plane is defined as  ax+by+cz+d = 0

v1 = p2 - p1;
v2 = p3 - p1;
n = cross(v1,v2);
a = n(1); b = n(2); c = n(3);
d = -a*p2(1)-b*p2(2)-c*p2(3);

end 

% 
% ax1 = axes{6}(1 : 3, 1)';
% ax2 = axes{6}(1 : 3, 2)';
% ax3 = axes{6}(1 : 3, 3)';
% vt = B(15,:)-A(16,:)/norm(B(15,:)-A(16,:));
% 
% vt_o = B(15,:)-A(16,:);
% 
% ax1_dot = dot(ax1,vt);
% ax2_dot = dot(ax2,vt);
% ax3_dot = dot(ax3,vt);
% 
% -14.5, 36.9725, 43.7972
% 
% 
% pt1 = A(16,:);
% pt2 = A(16,:) + axes{6}(1 : 3, 1)';
% pt3 = A(16,:) + axes{6}(1 : 3, 2)';

% pt1 = A(12,:);
% pt2 = A(12,:) + 10 * axes_d{9}(1 : 3, 1)';
% pt3 = A(12,:) + 10 * axes_d{9}(1 : 3, 2)';
% PTS = [pt1; pt2; pt3];

% pt1 = A(4,:);
% pt2 = A(4,:)  + 10 * axes_d{15}(1 : 3, 1)';
% pt3 = A(4,:)  + 10 * axes_d{15}(1 : 3, 2)';
% PTS = [pt1; pt2; pt3];
% 
% % 
% [a, b, c, d] = generate_plane_3point(pt1, pt2, pt3);
% plane_normal = [a,b,c];
% plane_normal = plane_normal/norm(plane_normal);
% 
% P_P0 = B(15,:) - A(16,:);
% d = dot(P_P0,plane_normal)
% 
% ss = plane_normal*-d + B(15,:);

% figure()
% hold on
% axis equal
% axis off
% plot3(B(3,1),B(3,2),B(3,3),'ko')
% plot3(PTS(:,1),PTS(:,2),PTS(:,3),'co')
% plot3(A(:,1),A(:,2),A(:,3),'k*')
% plot3(A(1:4,1),A(1:4,2),A(1:4,3),'-b')
% plot3(A(5:8,1),A(5:8,2),A(5:8,3),'-b')
% plot3(A(9:12,1),A(9:12,2),A(9:12,3),'-b')
% plot3(A(13:16,1),A(13:16,2),A(13:16,3),'-b')
% plot3(A([17:20 22],1),A([17:20 22],2),A([17:20 22],3),'-b')
% plot3(A([4 22 8],1),A([4 22 8],2),A([4 22 8],3),'-b')
% plot3(A([12 22 16],1),A([12 22 16],2),A([12 22 16],3),'-b')
% plot3(B(:,1),B(:,2),B(:,3),'r*');
% plot3(B(1:4,1),B(1:4,2),B(1:4,3),'-r');
% plot3(B(5:8,1),B(5:8,2),B(5:8,3),'-r');
% plot3(B(9:12,1),B(9:12,2),B(9:12,3),'-r');
% plot3(B(13:16,1),B(13:16,2),B(13:16,3),'-r');
% plot3(B([4 22 8],1),B([4 22 8],2),B([4 22 8],3),'-r')
% plot3(B([12 22 16],1),B([12 22 16],2),B([12 22 16],3),'-r')
% hold off