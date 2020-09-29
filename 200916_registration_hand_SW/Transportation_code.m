function [Sol] = Transportation_code(A, supply, demand)

vogelResult = full(vogels(A,supply,demand)) %returns back allocation matrix, a basic feasible solution, from Vogel's Method
%xlswrite('vogelResult.xlsx',vogelResult);  % editIvan_20200929
vogelCost = sum(sum(A.*vogelResult)) %calculates the total cost of the initial basic feasible solution
% optimalSolution = modi(A, vogelResult, supply, demand) %optimization % function editIvan_20200929
Sol = vogelResult;  % editIvan_20200929
% fprintf("Optimized cost = %d\n",sum(sum(A.*optimalSolution))) % editIvan_20200929
fprintf("Optimized cost = %d\n",sum(sum(A.*Sol)))
end