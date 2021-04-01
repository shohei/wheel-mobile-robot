% Probabilities of selecting the first, second or third path:
% p(A) = [P(A1), P(A2), P(A3)]
p_A = [0.7 0.1 0.2]
% Probabilities of encountering on an obstacle on the first, second and
% third path: p(B|A) = [P(B|A1), P(B|A2), P(B|A3)]
p_BA = [0.05 0.1 0.08]

% Probability of an obstacle: P(B)
P_B = p_BA*p_A.'

% Probability of mobile robot getting stuck on the first, second and third
% path: p(A|B) = [P(A1|B), P(A2|B), P(A3|B)]
p_AB = (p_BA.*p_A)./P_B