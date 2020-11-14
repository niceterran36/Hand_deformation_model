function [scale_factor] = s_scale_factor(A, B)

m = size(A,1);
n = size(B,1);

scale_factor = zeros(m-1,1);

    for i = 1:(m-1)
        LA(i,1) = norm(A(i,:)-A(i+1,:));
        LA(i,2) = norm(B(i,:)-B(i+1,:));
        scale_factor(i,1) = LA(i,1)/LA(i,2);
        scale_factor(i,2) = i;
        scale_factor(i,3) = i+1;
    end
    
    scale_factor([3 6 9 12:end],:) = [];
        
end 
% srf_LMt = LMt(1:12,:); % palmar
% srf_LMs = LMs(1:12,:);
% 
% srf2_LMt = LMt(1:12,:); % dorsal