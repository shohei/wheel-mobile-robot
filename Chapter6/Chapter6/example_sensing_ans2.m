p_ZdX = [0.2 0.2 0.6 0.6 0.2];
p_ZbX = 1-p_ZdX;
p_X = ones(1,5)/5;

disp('Probability of detecting a dark tile P(Z1=dark)')
P_z1 = p_ZdX*p_X.'
disp('Posterior distribution p(X1|Z1=dark)')
p_Xz1 = p_ZdX.*p_X/P_z1

disp('Probability of detecting a dark tile again P(Z2=dark|Z1=dark)')
P_z2 = p_ZdX*p_Xz1.'
disp('Posterior distribution p(X2|Z1=dark,Z2=dark)')
p_Xz2 = p_ZdX.*p_Xz1/P_z2