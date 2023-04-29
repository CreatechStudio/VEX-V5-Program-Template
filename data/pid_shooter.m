clear;
i = "tbh-shoot9-0.040";
filename = 'shooter-data-0.040.txt';
temp = readmatrix(filename);
num = size(temp, 1);
timestep = 0.005;
time = 0: timestep: timestep * (num-1);

input = temp(:, 1);
output = temp(:, 2);
tbh = temp(:, 3);
tiledlayout(3,1);

ax1 = nexttile;
plot(ax1, time, input);
title(ax1,filename);
ylabel(ax1, 'input');
ax2 = nexttile;
plot(ax2, time, output);
title(ax2,filename);
ylabel(ax2, 'output');

% normalized
% output = (output .* 39.8) - 217;

ax3 = nexttile;
plot(ax3, time, tbh, time, output);
title(ax3,filename);
ylabel(ax3, 'tbh');

saveas(gcf,sprintf('graph/shooter-graph-%s.jpg', i));


    