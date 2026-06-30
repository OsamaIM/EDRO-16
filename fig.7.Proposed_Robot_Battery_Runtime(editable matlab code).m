% Robot Battery Runtime - 7.4V 1500mAh BEAT LiPo 2S 35C (Experimental)

modes = {'Idle', 'Smooth Handshake', 'Slow Walk', 'Normal/Fast Walk', 'Dancing' };
runtimes = [303, 167, 115, 83, 75];

figure('Color', 'white', 'Position', [100, 100, 900, 750]);

b = bar(runtimes, 'FaceColor', [0.102 0.337 0.859], 'EdgeColor', 'none', 'BarWidth', 0.6);
% Labels above bars
for i = 1:length(runtimes)
    text(i, runtimes(i) + 4, sprintf('%d min', runtimes(i)), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'FontSize', 13, ...
        'FontName', 'Times New Roman', ...
        'FontWeight', 'bold', ...
        'Color', 'k');
end

% Axes formatting
ax = gca;
% Increase bottom margin
ax.Position = [0.10 0.20 0.80 0.72];
xlim(ax,[0.5 numel(modes)+0.9])
ax.XTickLabel = modes;
ax.XTickLabelRotation = 0;

ax.FontName = 'Times New Roman';
ax.FontSize = 14;

ax.YLim = [0 340];

ax.YGrid = 'on';
% Border thickness
ax.LineWidth = 1.8; 
ax.GridColor = [0.85 0.85 0.85];
ax.GridLineStyle = '--';
ax.GridAlpha = 0.8;

ax.Box = 'on';
ax.XColor = 'k';
ax.YColor = 'k';

xlabel('Robot Operating Mode', ...
    'FontSize', 15, ...
    'FontWeight', 'bold', ...
    'FontName', 'Times New Roman');
ax.TickLabelInterpreter = 'none';

ylabel('Runtime (minutes)', ...
    'FontSize', 15, ...
    'FontWeight', 'bold', ...
    'FontName', 'Times New Roman');

lgd = legend('7.4V 1500mAh BEAT LiPo 2S 35C (Experimental)', ...
    'Location', 'northeast', ...
    'FontSize', 13);

lgd.FontName = 'Times New Roman';

set(gcf, 'PaperPositionMode', 'auto');

% Export high-resolution image (400 DPI)
set(gcf, 'PaperPositionMode', 'auto');


exportgraphics(gcf, ...
    'robot_battery_runtime_400dpi.png', ...
    'Resolution', 400, ...
    'BackgroundColor', 'white');