% Notation: X === X(k), X' === X(k-1)
disp('Initial belief of clean and dirty floor')
bel_Xc = 0.5; % bel(X=clean)
bel_X = [bel_Xc 1-bel_Xc] % bel(X=clean), bel(X=dirty)

disp('Conditional probabilities of the dirt sensor measurements')
P_ZcXc = 0.8;      % P(Z=clean|X=clean)
P_ZdXc = 1-P_ZcXc; % P(Z=dirty|X=clean)
P_ZdXd = 0.9;      % P(Z=dirty|X=dirty)
P_ZcXd = 1-P_ZdXd; % P(Z=clean|X=dirty)
p_ZX = [P_ZcXc, P_ZcXd; ...
        P_ZdXc, P_ZdXd]

disp('Outcome probabilities in the case of cleaning')
P_XcXcUc = 1;          % P(X=clean|X'=clean,U'=clean)
P_XdXcUc = 1-P_XcXcUc; % P(X=dirty|X'=clean,U'=clean)
P_XcXdUc = 0.8;        % P(X=clean|X'=dirty,U'=clean)
P_XdXdUc = 1-P_XcXdUc; % P(X=dirty|X'=dirty,U'=clean)
p_ZXUc = [P_XcXcUc, P_XdXcUc; ...
          P_XcXdUc, P_XdXdUc]

disp('Outcome probabilities in the case of no cleaning action is taken')
P_XcXcUn = 1;          % P(X=clean|X'=clean,U'=null)
P_XdXcUn = 1-P_XcXcUn; % P(X=dirty|X'=clean,U'=null)
P_XcXdUn = 0;          % P(X=clean|X'=dirty,U'=null)
P_XdXdUn = 1-P_XcXdUn; % P(X=dirty|X'=dirty,U'=null)
p_ZXUn = [P_XcXcUn, P_XdXcUn; ...
          P_XcXdUn, P_XdXdUn]

U = {'null',  'clean', 'clean'};
Z = {'dirty', 'clean', 'clean'};
for k=1:length(U)
    fprintf('Prediction step: U(%d)=%s\n', k-1, U{k})
    if strcmp(U(k), 'clean')
        belp_X = bel_X*p_ZXUc
    else
        belp_X = bel_X*p_ZXUn
    end
    
    fprintf('Correction step: Z(%d)=%s\n', k, Z{k})
    if strcmp(Z(k), 'clean')
        bel_X = p_ZX(1,:).*belp_X;
    else
        bel_X = p_ZX(2,:).*belp_X;
    end
    bel_X = bel_X/sum(bel_X)
end