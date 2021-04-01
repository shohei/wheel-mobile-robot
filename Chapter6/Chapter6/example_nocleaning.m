% Probabilities of clean and dirty floor
P_Xc = 0.4    % P(X=clean)
P_Xd = 1-P_Xc % P(X=dirty)
% Conditional probabilities of the dirt sensor measurements
P_ZcXc = 0.8      % P(Z=clean|X=clean)
P_ZdXc = 1-P_ZcXc % P(Z=dirty|X=clean)
P_ZdXd = 0.9      % P(Z=dirty|X=dirty)
P_ZcXd = 1-P_ZdXd % P(Z=clean|X=dirty)

disp('Time step k = 1: Z=clean')
% Measurement probability in the case clean floor is detected
P_Zc_k1 = P_ZcXc*P_Xc + P_ZcXd*P_Xd
% Probability of clean floor after the measurement is made (Bayes' rule)
P_XcZc_k1 = P_ZcXc*P_Xc/P_Zc_k1
P_XdZc_k1 = 1-P_XcZc_k1;

disp('Time step k = 2: Z=clean')
% Measurement probability in the case clean floor is detected
P_Zc_k2 = P_ZcXc*P_XcZc_k1 + P_ZcXd*P_XdZc_k1
% Probability of clean floor after the measurement is made (Bayes' rule)
P_XcZc_k2 = P_ZcXc*P_XcZc_k1/P_Zc_k2
P_XdZc_k2 = 1-P_XcZc_k2;

disp('Time step k = 3: Z=dirty')
% Measurement probability in the case dirty floor is detected
P_Zd_k3 = P_ZdXc*P_XcZc_k2 + P_ZdXd*P_XdZc_k2
% Probability of clean floor after the measurement is made (Bayes' rule)
P_XcZd_k3 = P_ZdXc*P_XcZc_k2/P_Zd_k3
P_XdZd_k3 = 1-P_XcZd_k3;

hFig = OFig(2, 1, 1);
bar([P_XcZc_k1; ...
     P_XcZc_k2; ...
     P_XcZd_k3]);
xlabel('k'); ylabel('P(X=clean|Z)'); xlim([0.5, 3+0.5]);
labels = {'$Z=clean$', '$Z=clean$', '$Z=dirty$'};
for i=1:length(labels)
    text(i, 1.05, labels{i}, 'HorizontalAlignment', 'center', ...
                              'VerticalAlignment', 'bottom');
end
set(hFig.getAxes(), 'XTick', 1:3);