function [sum_results] =  dist_sum(verticesA, verticesB, pairs)

dist_array = [];
    for i = 1:size(pairs,1)
    dist = norm(verticesA(pairs(i,1),:)-verticesB(pairs(i,2),:));
    dist_array = [dist_array; dist];
    end 
sum_results = sum(dist_array);

end 