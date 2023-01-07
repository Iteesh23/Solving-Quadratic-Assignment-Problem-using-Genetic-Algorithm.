function PlotSolution(s,model)

    n=model.n;
    x=model.x;
    y=model.y;
    
    plot(x(s(1:n)),y(s(1:n)),'rs',...
        'MarkerSize',12,...
        'MarkerFaceColor','y');                 % Assigned Locations
    
    hold on;

    plot(x(s(n+1:end)),y(s(n+1:end)),'bo');     % Not-Assigned Locations
    
    for i=1:n
        text(x(s(i))+1,y(s(i))-2,num2str(i),'FontSize',12);
    end
    
    hold off;
    grid on;
    axis equal;
    
end