% This is a script to analyze the statistics of MuJoCo simulation results
% of three experimental objects. The visualization of the results
% correspond to Figure 9 in the manuscript.
% 
% Analysis approach: ANOVA + posthoc t-test. For each object, consider two
% labels:
%   - palm: 'P': with palm adaptation, 'NP': no palm adaptation
%   - finger: 'F': with finger regulation; 'NF': no finger kinematic regulation
% 
% Explanation of the variable name saved in mat file:
%   Format: OBJECTNAME_EXPTYPE_METRIC, where:
%       OBJECTNAME: {'finger_tip','apc_2x2','bunny'}
%       EXPTYPE: {00,01,10,11}
%       METRIC: {0,1,2}, 0: palm penetration (rate, %), 1: fingertips
%       penetration (rate, %), 2: fingertips losing contact (rate, %)

data = load('metric_data.mat');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A: the convex object ('finger_tip' -> 'F') %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% E.g., 'F0' is the metric type 0 of the object 'F'
% Notice that:
% Column: finger (case: _0 vs _1)
% Row: palm (0_ vs 1_)
% 
% y = [X00(:), X01(:);...
%      X10(:), X11(:)];

%% Compare metric 0: palm penetration (lower the better)
F0_00 = data.finger_tip_00_0;
F0_01 = data.finger_tip_01_0;
F0_10 = data.finger_tip_10_0;
F0_11 = data.finger_tip_11_0;

% [p,tbl,stats] = perform_statistical_analysis(F0_00,F0_01,F0_10,F0_11);
%{
Source          SS      df     MS        F      Prob>F
------------------------------------------------------
Columns       0.00006    1   0.00006     0.06   0.808 
Rows          0.21225    1   0.21225   214.39   0     
Interaction   0.001      1   0.001       1.01   0.3214
Error         0.03564   36   0.00099                  
Total         0.24895   39                            

p =

    0.8080    0.0000    0.3214
%}

%% Compare metric 1: fingertips penetration (lower the better)
F1_00 = data.finger_tip_00_1;
F1_01 = data.finger_tip_01_1;
F1_10 = data.finger_tip_10_1;
F1_11 = data.finger_tip_11_1;

% [p,tbl,stats] = perform_statistical_analysis(F1_00,F1_01,F1_10,F1_11);
%{
Source            SS        df       MS         F     Prob>F
------------------------------------------------------------
Columns       6.96574e-07    1   6.96574e-07   1.12   0.2977
Rows          5.39764e-08    1   5.39764e-08   0.09   0.7704
Interaction   2.95838e-07    1   2.95838e-07   0.47   0.4955
Error         2.24631e-05   36   6.23975e-07                
Total         2.35095e-05   39                              
%}

%% Compare metric 2: fingertips losing contact (lower the better)
F2_00 = data.finger_tip_00_2;
F2_01 = data.finger_tip_01_2;
F2_10 = data.finger_tip_10_2;
F2_11 = data.finger_tip_11_2;

[p,tbl,stats] = perform_statistical_analysis(F2_00,F2_01,F2_10,F2_11);
%{
Source          SS      df     MS        F     Prob>F
-----------------------------------------------------
Columns       0.0005     1   0.0005     7.32   0.0104
Rows          0.0066     1   0.0066    96.78   0     
Interaction   0.00219    1   0.00219   32.18   0     
Error         0.00245   36   0.00007                 
Total         0.01175   39                           

p =

    0.0104    0.0000    0.0000

multcompare(stats)
Note: Your model includes an interaction term that is significant at the level 
you specified. Testing main effects under these conditions is questionable.

ans =

    1.0000    2.0000    0.0018    0.0071    0.0124    0.0104
% First two digits: 1 vs 2: Column 1 vs Column 2 (Finger)
%}

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% B: the bottle ('apc_2x2' -> 'A') %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Compare metric 0: palm penetration (lower the better)
A0_00 = data.apc_2x2_00_0;
A0_01 = data.apc_2x2_01_0;
A0_10 = data.apc_2x2_10_0;
A0_11 = data.apc_2x2_11_0;

% [p,tbl,stats] = perform_statistical_analysis(A0_00,A0_01,A0_10,A0_11);
%{
Source          SS      df     MS         F      Prob>F
-------------------------------------------------------
Columns       0.0035     1   0.0035       6.81   0.0132
Rows          0.80767    1   0.80767   1568.25   0     
Interaction   0.0012     1   0.0012       2.32   0.1363
Error         0.01854   36   0.00052                   
Total         0.83091   39                             

p =

    0.0132    0.0000    0.1363
%}

%%% Compare metric 1: fingertips penetration (lower the better)
A1_00 = data.apc_2x2_00_1;
A1_01 = data.apc_2x2_01_1;
A1_10 = data.apc_2x2_10_1;
A1_11 = data.apc_2x2_11_1;

[p,tbl,stats] = perform_statistical_analysis(A1_00,A1_01,A1_10,A1_11);
%{
Source          SS      df     MS       F     Prob>F
----------------------------------------------------
Columns       0.00024    1   0.00024   2.29   0.1393
Rows          0.00064    1   0.00064   6.01   0.0192
Interaction   0.00075    1   0.00075   7.03   0.0119
Error         0.00383   36   0.00011                
Total         0.00546   39                          

p =

    0.1393    0.0192    0.0119
%}

%%% Compare metric 2: fingertips losing contact (lower the better)
A2_00 = data.apc_2x2_00_2;
A2_01 = data.apc_2x2_01_2;
A2_10 = data.apc_2x2_10_2;
A2_11 = data.apc_2x2_11_2;

% [p,tbl,stats] = perform_statistical_analysis(A2_00,A2_01,A2_10,A2_11);
%{
Source          SS      df     MS        F     Prob>F
-----------------------------------------------------
Columns       0.01135    1   0.01135    5.77   0.0216
Rows          0.07865    1   0.07865   40.01   0     
Interaction   0.01101    1   0.01101    5.6    0.0234
Error         0.07077   36   0.00197                 
Total         0.17179   39                           

p =

    0.0216    0.0000    0.0234
%}

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% C: the bunny ('bunny' -> 'B') %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Compare metric 0: palm penetration (lower the better)
B0_00 = data.bunny_00_0;
B0_01 = data.bunny_01_0;
B0_10 = data.bunny_10_0;
B0_11 = data.bunny_11_0;

% [p,tbl,stats] = perform_statistical_analysis(B0_00,B0_01,B0_10,B0_11);
%{
Source          SS      df     MS       F     Prob>F
----------------------------------------------------
Columns       0.00105    1   0.00105   0.34   0.5642
Rows          0.00115    1   0.00115   0.37   0.5458
Interaction   0.01852    1   0.01852   5.99   0.0194
Error         0.11132   36   0.00309                
Total         0.13204   39                          

p =

    0.5642    0.5458    0.0194
%}

%%% Compare metric 1: fingertips penetration (lower the better)
B1_00 = data.bunny_00_1;
B1_01 = data.bunny_01_1;
B1_10 = data.bunny_10_1;
B1_11 = data.bunny_11_1;

% [p,tbl,stats] = perform_statistical_analysis(B1_00,B1_01,B1_10,B1_11);
%{
Source          SS      df     MS       F     Prob>F
----------------------------------------------------
Columns       0.00002    1   0.00002   0.12   0.73  
Rows          0.00076    1   0.00076   3.84   0.0578
Interaction   0.00003    1   0.00003   0.16   0.6932
Error         0.00716   36   0.0002                 
Total         0.00798   39                          

p =

    0.7300    0.0578    0.6932
%}

%%% Compare metric 2: fingertips losing contact (lower the better)
B2_00 = data.bunny_00_2;
B2_01 = data.bunny_01_2;
B2_10 = data.bunny_10_2;
B2_11 = data.bunny_11_2;

% [p,tbl,stats] = perform_statistical_analysis(B2_00,B2_01,B2_10,B2_11);
%{
Source          SS      df     MS        F     Prob>F
-----------------------------------------------------
Columns       0.00056    1   0.00056    0.22   0.6439
Rows          0.05023    1   0.05023   19.41   0.0001
Interaction   0.00093    1   0.00093    0.36   0.5534
Error         0.09317   36   0.00259                 
Total         0.1449    39                           

p =

    0.6439    0.0001    0.5534
%}



function [p,tbl,stats] = perform_statistical_analysis(X00,X01,X10,X11)
    % balanced design, 2-way ANOVA
    % Factor A: palm; factor B: finger; replication: length of each vector
    N = length(X00);
    assert(length(X01)==N);
    assert(length(X10)==N);
    assert(length(X11)==N);

    y = [X00(:), X01(:);...
        X10(:), X11(:)];
    reps = N;
    [p,tbl,stats] = anova2(y,reps);
end