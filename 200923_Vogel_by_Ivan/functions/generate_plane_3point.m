function [a, b, c, d] = generate_plane_3point(p1, p2, p3)

% plane is defined as  ax+by+cz+d = 0

v1 = p2 - p1;
v2 = p3 - p1;
n = cross(v1,v2);
a = n(1); b = n(2); c = n(3);
d = -a*p2(1)-b*p2(2)-c*p2(3);

end 
