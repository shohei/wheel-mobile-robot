p_ZdX = [0.2 0.2 0.6 0.6 0.2];
p_ZbX = 1-p_ZdX;
p_X = ones(1,5)/5;

disp('Probability of detecting a dark tile P(Z1=dark)')
P_z1 = p_ZdX*p_X.'
disp('Posterior distribution p(X1|Z1=dark)')
p_Xz1 = p_ZdX.*p_X/P_z1

disp('Probability of detecting a bright tile P(Z2=bright|Z1=dark)')
P_z2 = p_ZbX*p_Xz1.'
disp('Posterior distribution p(X2|Z1=dark,Z2=bright)')
p_Xz2 = p_ZbX.*p_Xz1/P_z2

disp('Probability of detecting a dark tile P(Z3=dark|Z1=dark,Z2=bright)')
P_z3 = p_ZdX*p_Xz2.'
disp('Posterior distribution p(X3|Z1=dark,Z2=bright,Z3=dark)')
p_Xz3 = p_ZdX.*p_Xz2/P_z3

hFig = OFig(3, 1);
l = [0.5, length(p_Xz1)+0.5];
hFig.axes(1);
bar(p_Xz1);
xlim(l);
title('$p(X_1|Z_1=dark)$');
hFig.axes(2);
bar(p_Xz2);
xlim(l);
title('$p(X_2|Z_1=dark,Z_2=bright)$');
hFig.axes(3);
bar(p_Xz3);
xlim(l);
title('$p(X_3|Z_1=dark,Z_2=bright,Z_3=dark)$');

set(hFig.getAxes(), 'XTick', 1:length(p_Xz1), 'XTickLabel', {'$x_1$', '$x_2$', '$x_3$', '$x_4$', '$x_5$'});
linkaxes(hFig.getAxes(), 'xy');