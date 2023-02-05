% define figure properties
opts.Colors     = get(groot,'defaultAxesColorOrder');
opts.saveFolder = 'plotting/img/';
opts.width      = 8;
opts.height     = 6;
opts.fontType   = 'Times';
opts.fontSize   = 9;

% create new figure
% fig = figure; clf

%% plot
% load('q.mat')
eef_position = zeros(3, size(q, 2));
base_frame = zeros(4, 1);
base_frame(4, 1) = 1;
time_steps = size(q, 2);
for i=1:time_steps
    T_ = forwardKinematicsAllJoints(q(:, i)) ;
    eef(:, i) = T_(:,:,6) * base_frame;
end

box_6_dof_plot(q(:, 1))
hold on
p = plot3(eef(1, :), eef(2, :), eef(3, :));
hold off
box_6_dof_plot(q(:, time_steps))
grid on
p.LineWidth = 2;
axis equal

% %% plot
% 
% % add axis labes and legend
% axis tight
% xlabel('Time in seconds')
% ylabel('Motor Torque in Nm')
% lgd=legend("joint 1", "joint 2", "joint 3", "joint 4", "joint 5", "joint 6")
% set(lgd,'FontSize',6)
% % scaling
% fig.Units               = 'centimeters'
% fig.Position(3)         = opts.width;
% fig.Position(4)         = opts.height;
% 
% % set text properties
% set(fig.Children, ...
%     'FontName',     'Times', ...
%     'FontSize',     9);
% 
% % remove unnecessary white space
% set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
% set(lgd,'FontSize',6)
% 
% % export to png
% fig.PaperPositionMode   = 'auto';
% print([opts.saveFolder 'q_torque_plot'], '-dpng', '-r600')