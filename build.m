function axe = build(o, x, y)
% o: cross point (1 by 3) of x, y vector
% x, y is vector(1 by 3) to build basie 2 axis 

    x = normalizerow(x);
    y = normalizerow(y);
    z = normalizerow(cross(x, y));
    y = cross(z, x);
    axe = [x', y', z', o'; 0, 0, 0, 1];
end