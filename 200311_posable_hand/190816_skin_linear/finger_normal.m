function normal = finger_normal(u, v, w, x)
    uv = normalizerow(v - u);
    uw = normalizerow(w - u);
    ux = normalizerow(x - u);
    n = cross(uw, uv);
    l = norm(n);
    if l < 0.001
        n = ux;
    else
        n = n / l;
    end
    if dot(ux, n) < 0
        n = -n;
    end
    normal = l * n + (1 - l) * ux;
end