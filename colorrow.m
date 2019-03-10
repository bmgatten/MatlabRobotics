figure(2)
hold on
xvals = xy(2:N+1,1);

for it = 1:length(xvals)
    ffff = truth(1,it,:)
    scatter(xvals(it)*ones(1,RL),1:20,'s','MarkerFaceColor',ffff,'MarkerEdgeColor',ffff)
end