disp('Sensor measurement distribution p(Z=dark|X)')
p_ZdX = [0.2 0.2 0.6 0.6 0.2]
disp('Sensor measurement distribution p(Z=bright|X)')
p_ZbX = 1-p_ZdX
disp('Initial distribution p(X)')
p_X = ones(1,5)/5

disp('Probability of detecting a dark cell P(Z=dark)')
P_Zd = p_ZdX*p_X.'

disp('Posterior distribution p(X|Z=dark)')
p_XZd = p_ZdX.*p_X/P_Zd

hFig = OFig(3, 1);
l = [0.5, length(p_X)+0.5];
hFig.axes(1);
bar(p_ZdX);
xlim(l);
title('$p(Z=dark)$');
hFig.axes(2);
bar(p_X);
xlim(l);
title('$p(X)$');
hFig.axes(3);
bar(p_XZd);
xlim(l);
title('$p(X|Z=dark)$');

set(hFig.getAxes(), 'XTick', 1:length(p_X), 'XTickLabel', {'$x_1$', '$x_2$', '$x_3$', '$x_4$', '$x_5$'});
linkaxes(hFig.getAxes(), 'xy');