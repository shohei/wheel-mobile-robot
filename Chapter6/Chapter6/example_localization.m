disp('Initial belief p(X0)')
bel_X0 = ones(1,5)/5

P_xxu_null = 0.8; % P(X=i|X'=j,U'=u), i=j+u
P_xxu_less = 0.1; % P(X=i|X'=j,U'=u), i=j+u-1
P_xxu_more = 0.1; % P(X=i|X'=j,U'=u), i=j+u+1

p_ZdX = [0.2 0.2 0.6 0.6 0.2]; % p(Z=dark|X)
p_ZbX = 1-p_ZdX;               % p(Z=bright|X)

bel_X = bel_X0;
for k=1:3
    % Prediction step
    p_xXu = [P_xxu_less 0 0 P_xxu_more P_xxu_null]; % for U=1
    belp_X = zeros(1,5);
    for i=1:5
        belp_X(i) = p_xXu*bel_X.';
        p_xXu = p_xXu([end 1:end-1]);
    end
    
    % Correction step
    if k==1
        bel_X = p_ZbX.*belp_X;
    else
        bel_X = p_ZdX.*belp_X;
    end
    bel_X = bel_X/sum(bel_X);
    
    if k==1
        disp('Beliefs belp_X1 and bel_X1')
        belp_X1 = belp_X
        bel_X1 = bel_X
    elseif k==2
        disp('Beliefs belp_X2 and bel_X2')
        belp_X2 = belp_X
        bel_X2 = bel_X
    elseif k==3
        disp('Beliefs belp_X3 and bel_X3')
        belp_X3 = belp_X
        bel_X3 = bel_X
        disp('Less likely to most likely position')
        [m,mi] = sort(bel_X)
    end
end

hFig = OFig(3, 2);
l = [0.5, length(belp_X1)+0.5];
for i=1:3
    hFig.axes(2*i-1);
    bar(eval(sprintf('belp_X%d', i)));
    xlim(l);
    title(sprintf('$bel_p(X_%d)$', i));
    hFig.axes(2*i);
    bar(eval(sprintf('bel_X%d', i)));
    xlim(l);
    title(sprintf('$bel(X_%d)$', i));
end

set(hFig.getAxes(), 'XTick', 1:length(belp_X1), 'XTickLabel', {'$x_1$', '$x_2$', '$x_3$', '$x_4$', '$x_5$'});
linkaxes(hFig.getAxes(), 'xy');