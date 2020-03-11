
SM = [-76.23,-24.25;0,0;39.80,-8.40;59.50,-24.20;71.80,-41.7]
IK = [-76.23,-24.250;0,0;39.2,-10.80;60.4,-25.5;71.60,-43.0]


figure(1)
hold on
axis equal
axis([-100 100 -80 60])

scatter(SM(:,1), SM(:,2), '.', 'MarkerEdgeColor',[0, 0, 0])
scatter(IK(:,1), IK(:,2), '*', 'MarkerEdgeColor',[1, 0, 0])


hold off