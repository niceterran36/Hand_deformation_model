function transform = interpolate(transforms, weights)
    transform = zeros(4);
    for i = 1 : length(transforms)
        transform = transform + transforms{i} * weights(i);
    end
end
