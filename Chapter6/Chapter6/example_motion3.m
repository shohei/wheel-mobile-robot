disp('Initial belief p(X0)')
p_X0 = [1 0 0 0 0]

P_xxu_null = 0.8; % P(X=i|X'=j,U'=u), i=j+u
P_xxu_less = 0.1; % P(X=i|X'=j,U'=u), i=j+u-1
P_xxu_more = 0.1; % P(X=i|X'=j,U'=u), i=j+u+1

p_X = p_X0;
for k=1:1000
    p_xXu = [P_xxu_less 0 0 P_xxu_more P_xxu_null]; % for U=1
    p_Xu = zeros(1,5);
    for i=1:5
        p_Xu(i) = p_xXu*p_X.';
        p_xXu = p_xXu([end 1:end-1]);
    end
    p_X = p_Xu;
    if k==10
        disp('Belief p(X10|U9=1)');
        p_X10 = p_X
    elseif k==1000
        disp('Belief p(X1000|U999=1)');
        p_X1000 = p_X
    end
end

hFig = OFig(3, 1);
l = [0.5, length(p_X0)+0.5];
hFig.axes(1);
bar(p_X0);
xlim(l);
title('Initial belief $p(X_0)$');
hFig.axes(2);
bar(p_X10);
xlim(l);
title('Belief $p(X_{10}|U_9=1)$');
hFig.axes(3);
bar(p_X1000);
xlim(l);
title('Belief $p(X_{1000}|U_{999}=1)$');

set(hFig.getAxes(), 'XTick', 1:length(p_X0), 'XTickLabel', {'$x_1$', '$x_2$', '$x_3$', '$x_4$', '$x_5$'});
linkaxes(hFig.getAxes(), 'xy');