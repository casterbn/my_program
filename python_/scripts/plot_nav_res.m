wx01 = [-2754340.55489618 4694399.89492641 3314096.35328581]';
wx02 = [-2754345.09182931, 4694375.35932328, 3314127.32088641]';

ref = wx02;

dirpath = '..\NEWRTK\results\';

% pos = csvread('.\log_data\109\static_st_spp_ins.csv', 1, 0);
pos = load([dirpath, 'gga_2019-06-25 22-32-39.pos']);

col_t = 1; % 3
col_s_pos = 1;
col_e_pos = 3;
col_q = 5;

idx = find(pos(:,col_q) == 5);
% t = pos(idx, col_t);
xyz = pos(idx, col_s_pos:col_e_pos); % 18:20

% [tow, ia, ic] = unique(t);
ia = idx;

% xyz = xyz(ia,:);

n = length(ia);
t = 1:1:n;

enu = zeros(n, 3);

for i = 1:n
    plh = xyz(i, :)';
    plh(1:2) = plh(1:2).*(pi/180.0);
    ecef = plh2xyz(plh);
%     ecef = plh;
    enu(i, :) = ConvertVector_from_ECEF_to_ENU(ecef, ref)';
    
end

stat = rms(enu(:, 1:2));
%%
figure;
subplot(3,1,1)
plot(t, enu(:, 1),'b*')
subplot(3,1,2)
plot(t, enu(:, 2),'b*')
subplot(3,1,3)
plot(t, enu(:, 3),'b*')

%%
figure;
plot(enu(:, 1), enu(:, 2), 'b*')
xlabel(gca, 'East (m)')
ylabel(gca, 'North (m)')
title(gca, ['RMS (m): East: ', num2str(stat(1),'%.2f'), ', North: ', num2str(stat(2), '%.2f')]);
grid on

