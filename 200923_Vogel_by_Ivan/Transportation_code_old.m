function [Sol] = Transportation_code(A, supply, demand)

vogelResult = full(vogels(A,supply,demand)) %returns back allocation matrix, a basic feasible solution, from Vogel's Method
vogelCost = sum(sum(A.*vogelResult)) %calculates the total cost of the initial basic feasible solution
optimalSolution = modi(A, vogelResult, supply, demand) %optimization function
A
fprintf("Optimized cost = %d\n",sum(sum(A.*optimalSolution)))
end
