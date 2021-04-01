o  = 1000*[0.0149, 0.0693; ...
           1.6228, 0.0679; ...
           1.6241, 1.0867; ...
           0.0112, 1.0854];
o1 = 1000*[0.4263, 0.4569; ...
           0.6144, 0.6857; ...
           0.3097, 0.9414; ...
           0.1190, 0.7126];
o2 = 1000*[0.8151, 0.2079; ...
           1.0885, 0.3008; ...
           0.9644, 0.6574; ...
           0.8753, 0.6278; ...
           0.9706, 0.3573; ...
           0.7838, 0.2927];
o3 = 1000*[1.3319, 0.4865; ...
           1.4723, 0.5659; ...
           1.3845, 0.7112];
% Free space is on the right-hand side of the obstacle
obstacles = {flipud(o), o1, o2, o3};