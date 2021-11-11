clc
clf
close all

T = readtable('handspose_final (1).csv');


% plot(Ta.Var1,Ta.Var5)
% hold on
%
% %plot(Ta.Var1,Ta.Var6)
% plot(Ta.Var1,Ta.Var7)
% %plot(Ta.Var1,Ta.Var8)
% hold off

% ctr R ctr T

%   yyaxis left
% plot(T.TimeMs(1:49),T.left_0_x(1:49))

% hold on
% % yyaxis right
% plot(T.TimeMs(1:49),T.left_0_y(1:49))
%
% yyaxis right
% plot(T.TimeMs(1:49),T.min_dist_left(1:49))

b = 131:220;




% plot(T.TimeMs(b), T.left_0_x(b), 'b','LineWidth' ,1, 'LineStyle','-');
% hold on
% plot(T.TimeMs(b), T.left_0_y(b), 'r','LineWidth' ,1, 'LineStyle','-');
% 
% plot(T.TimeMs(b), T.right_0_x(b), 'c','LineWidth' ,1, 'LineStyle','-');
% plot(T.TimeMs(b), T.right_0_y(b), 'm','LineWidth' ,1, 'LineStyle','-');
% ylabel('POINT LOCATION');
% 
% yyaxis right

plot(T.TimeMs(b), T.min_dist_left(b), 'g','LineWidth' ,1, 'LineStyle','-');
hold on
plot(T.TimeMs(b), T.min_dist_right(b),'k','LineWidth' ,1, 'LineStyle','-');


% legend('Left x','Left y','Right x','Right y', 'min distance Left','min distance right','Location','Northeastoutside')
legend( 'min distance Left hand','min distance right hand','Location','Northeastoutside')
xlabel('time(ms)');
ylabel('Minimum distance');



