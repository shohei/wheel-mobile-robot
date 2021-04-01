disp('Initial belief p(X0)')
p_X0 = [1 0 0 0 0]

P_xxu_null = 0.8; % P(X=i|X'=j,U'=u), i=j+u
P_xxu_less = 0.1; % P(X=i|X'=j,U'=u), i=j+u-1
P_xxu_more = 0.1; % P(X=i|X'=j,U'=u), i=j+u+1

disp('Belief p(X1|U0=2)');
p_xXu = [0 0 P_xxu_more P_xxu_null P_xxu_less]; % for U=2
p_Xu = zeros(1,5);
for i=1:5
    p_Xu(i) = p_xXu*p_X0.';
    p_xXu = p_xXu([end 1:end-1]);
end
p_X1 = p_Xu

hFig = OFig(3, 1, 2);
l = [0.5, length(p_X0)+0.5];
hFig.axes(1);
bar(p_X0);
xlim(l);
title('Initial belief $p(X_0)$');
hFig.axes(2);
bar(p_X1);
xlim(l);
title('Belief $p(X_1|U_0=2)$');

set(hFig.getAxes(), 'XTick', 1:length(p_X0), 'XTickLabel', {'$x_1$', '$x_2$', '$x_3$', '$x_4$', '$x_5$'});
linkaxes(hFig.getAxes(), 'xy');