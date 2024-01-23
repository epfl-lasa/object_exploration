% Bar Chart with Error Bars

%% 1\% Motion range

t_fmincon = [0.0608 0.0468 0.0555 0.0414;
0.0613 0.0476 0.0415 0.0467;
0.0574 0.0600 0.0496 0.0524;
0.0734 0.0440 0.0511 0.0500;
0.0458 0.0431 0.0460 0.0495];

t_qp = [0.0085 0.0027 0.0032 0.0027;
0.0035 0.0026 0.0031 0.0022;
0.0026 0.0225 0.0020 0.0022;
0.0023 0.0023 0.0037 0.0036;
0.0024 0.0025 0.0027 0.0023];

% figure, hold on;
% subplot(1,2,1);
% hold on;
% plot_bar_chart_with_error_bars(mean(t_fmincon,1), std(t_fmincon,[],1));
% ylim([0 0.15]);
% title('fmincon');
% hold off;
% 
% subplot(1,2,2);
% hold on;
% plot_bar_chart_with_error_bars(mean(t_qp,1), std(t_qp,[],1));
% ylim([0 0.15]);
% title('QP');
% hold off;

fval_fmincon = [0.2116 0.1599 0.1753 0.1922;
0.1832 0.1613 0.1395 0.1662;
0.1987 0.1653 0.1401 0.1896;
0.2025 0.1937 0.1922 0.1641;
0.2181 0.1592 0.1862 0.1634];

fval_qp = [0.4503 0.3859 0.4102 0.3133;
0.3976 0.3594 0.3081 0.3738;
0.4381 0.3708 0.3086 0.4278;
0.4462 0.4374 0.4337 0.3661;
0.4778 0.3518 0.4201 0.3646];

% subplot(1,2,1);
% hold on;
% plot_bar_chart_with_error_bars(mean(fval_fmincon,1), std(fval_fmincon,[],1));
% ylim([0 1]);
% title('fmincon');
% hold off;
% 
% subplot(1,2,2);
% hold on;
% plot_bar_chart_with_error_bars(mean(fval_qp,1), std(fval_qp,[],1));
% ylim([0 1]);
% title('QP');
% hold off;

cartesian_err_fmincon = [0.0098 0.0107 0.0116 0.0121;
0.0093 0.0112 0.0102 0.0113;
0.0096 0.0110 0.0102 0.0120;
0.0097 0.0122 0.0121 0.0113;
0.0100 0.0107 0.0120 0.0109];

cartesian_err_qp = [9.0792 6.7590 6.6767 6.5069;
8.4004 6.1709 6.5419 6.9608;
9.1680 7.7467 6.4055 7.2155;
9.1221 7.1996 7.1812 6.1562;
8.9342 7.5776 6.7764 7.6432];

subplot(1,2,1);
hold on;
plot_bar_chart_with_error_bars(mean(cartesian_err_fmincon,1), std(cartesian_err_fmincon,[],1));
ylim([0 1]);
title('fmincon');
hold off;

subplot(1,2,2);
hold on;
plot_bar_chart_with_error_bars(mean(cartesian_err_qp,1), std(cartesian_err_qp,[],1));
ylim([0 10]);
title('QP');
hold off;
%% 

function plot_bar_chart_with_error_bars(data, std)
    x = 1:numel(data);
    data = data(:).';
    std = std(:).';
    errhigh = data + std;
    errlow  = data - std;

    bar(x,data)                
    hold on;

    er = errorbar(x,data,errlow,errhigh);    
    er.Color = [0 0 0];                            
    er.LineStyle = 'none';
    xlabel('Finger Index');
    ylabel('Error in Cartesian Space');
%     hold off;
%     axis equal
    grid on

end