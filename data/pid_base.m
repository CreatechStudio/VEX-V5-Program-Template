clear;

for i = 3.10: 0.1: 4.4
location = 'robot';
filename = sprintf('%s/rotate-d-%1.2f-%1.2f.txt', location, 1.00, i);
% filename = sprintf('%s/rotate-p-%1.2f.txt', location, i);
% filename = sprintf('%s/forward-p-%1.2f.txt', location, i);
% filename = sprintf('%s/forward-d-%1.2f-%1.2f.txt', location, 0.34, i);
temp = readmatrix(filename);
num = size(temp, 1);
timestep = 0.02;
time = 0: timestep: timestep * (num-1);

input = temp(:, 1);
output = temp(:, 2);

tiledlayout(2,1);
ax1 = nexttile;
plot(ax1, time, input);
title(ax1,filename);
ylabel(ax1, 'input');
% yline(600);
yline(180);
ax2 = nexttile;
plot(ax2, time, output);
title(ax2,filename);
ylabel(ax2, 'output');
yline(0);
saveas(gcf,sprintf('%s/rotate-d-%1.2f-%1.2f.jpg', location, 1.00, i));
% saveas(gcf,sprintf('%s/rotate-p-%1.2f.jpg', location, i));
% saveas(gcf,sprintf('%s/forward-p-%1.2f.jpg', location, i));
% saveas(gcf,sprintf('%s/forward-d-%1.2f-%1.2f.jpg', location, 0.34, i));

end

    