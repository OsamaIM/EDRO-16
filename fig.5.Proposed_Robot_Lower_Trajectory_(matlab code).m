%% ================================================================
%  gait_trajectory_plot.m
%  Updated Bipedal Gait Trajectory — derived from main.cpp
%
%  FIXES vs previous version:
%    - Phase D background shading and label now visible
%    - Legend shows proper names (no "data1..data7")
%    - Left hip pitch no longer spikes at end of cycle
%    - All 4 phase labels positioned correctly
% ================================================================

clear; clc; close all;

%% Parameters (match your tuning constants)
HIP   = 12;
KNEE  = 14;
ANKLE =  1;
ROLL  =  2;

%% Gait cycle axis
t = linspace(0, 100, 2001);

%% Smoothstep helper
ss = @(x, t0, t1) max(0, min(1, (x-t0)./(t1-t0))).^2 .* ...
                  (3 - 2*max(0, min(1, (x-t0)./(t1-t0))));

%% ── LEFT HIP PITCH ──────────────────────────────────────────────
% Phase B (25-50): swing forward +HIP
% Phase D (75-100): swing backward -HIP (contralateral)
lhip = zeros(size(t));

mask = (t >= 25 & t < 50);
lhip(mask) = HIP .* sin(ss(t(mask), 25, 50) .* pi);

mask = (t >= 75 & t <= 100);
lhip(mask) = -HIP .* sin(ss(t(mask), 75, 100) .* pi);

%% RIGHT HIP PITCH (antiphase)
rhip = -lhip;

%% ── LEFT KNEE ───────────────────────────────────────────────────
lknee = zeros(size(t));
mask = (t >= 25 & t < 38);
lknee(mask) = KNEE .* sin(ss(t(mask), 25, 38) .* (pi/2));
mask = (t >= 38 & t < 50);
lknee(mask) = KNEE .* cos(ss(t(mask), 38, 50) .* (pi/2));

%% RIGHT KNEE (offset by half cycle)
rknee = zeros(size(t));
mask = (t >= 75 & t < 88);
rknee(mask) = KNEE .* sin(ss(t(mask), 75, 88) .* (pi/2));
mask = (t >= 88 & t <= 100);
rknee(mask) = KNEE .* cos(ss(t(mask), 88, 100) .* (pi/2));

%% ── ANKLE COMPENSATION ──────────────────────────────────────────
lankle = zeros(size(t));
mask = (t < 25);
lankle(mask) = -ANKLE .* ss(t(mask), 0, 25);
mask = (t >= 25 & t < 50);
lankle(mask) = -ANKLE + ANKLE .* ss(t(mask), 25, 50);
mask = (t >= 50 & t < 75);
lankle(mask) = ANKLE .* ss(t(mask), 50, 75);
mask = (t >= 75 & t <= 100);
lankle(mask) = ANKLE - ANKLE .* ss(t(mask), 75, 100);

rankle = -lankle;

%% ── HIP ROLL ────────────────────────────────────────────────────
lroll = zeros(size(t));

segs = [0 12.5; 12.5 25; 25 37.5; 37.5 50; ...
        50 62.5; 62.5 75; 75 87.5; 87.5 100];
dirs  = [1; -1; -1; 1; 1; -1; -1; 1];   % sign of the ramp within each seg

for k = 1:size(segs,1)
    t0 = segs(k,1); t1 = segs(k,2);
    u  = ss(t, t0, t1);
    mask = (t >= t0 & t < t1);
    if k == size(segs,1)
        mask = (t >= t0 & t <= t1);
    end
    % absolute peak at segment mid: rises from 0->ROLL or ROLL->0
    if dirs(k) == 1
        lroll(mask) = ROLL .* u(mask);
    else
        lroll(mask) = ROLL .* (1 - u(mask));
    end
    % sign of phase: A/D positive, B/C negative
    if k <= 2,      lroll(mask) =  lroll(mask);
    elseif k <= 4,  lroll(mask) = -lroll(mask);
    elseif k <= 6,  lroll(mask) =  lroll(mask);
    else,           lroll(mask) = -lroll(mask);
    end
end

rroll = -lroll;

%% ── FIGURE ───────────────────────────────────────────────────────
fig = figure('Color','white','Position',[80 80 1100 560]);
ax  = axes('Parent', fig);
hold(ax, 'on');

%% Phase shading — all 4 phases, equal 25% width
phaseColors = [
    0.97 0.97 0.97;
    0.97 0.97 0.97;
    0.97 0.97 0.97;
    0.97 0.97 0.97];
phaseEdges = [0 25 50 75 100];
ylims = [-30 30];

for p = 1:4
    fill(ax, ...
        [phaseEdges(p) phaseEdges(p+1) phaseEdges(p+1) phaseEdges(p)], ...
        [ylims(1) ylims(1) ylims(2) ylims(2)], ...
        phaseColors(p,:), 'EdgeColor','none', 'FaceAlpha', 0.25);
end

%% Phase dividers
for xv = [25 50 75]
    xline(ax, xv, '--', 'Color', [0.60 0.60 0.60], 'LineWidth', 0.9);
end

%% Phase labels (inside each band, top)
phaseTxt  = {'Phase A','Phase B','Phase C','Phase D'};
phaseCx   = [12.5 37.5 62.5 87.5];
phaseTCol = {
    [0.20 0.20 0.20], ...
    [0.20 0.20 0.20], ...
    [0.20 0.20 0.20], ...
    [0.20 0.20 0.20]};
for p = 1:4
    text(ax, phaseCx(p), 18.5, phaseTxt{p}, ...
        'HorizontalAlignment','center', ...
        'FontSize', 16, 'FontWeight','bold', ...
        'Color', phaseTCol{p});
end

%% ── PLOT CURVES ──────────────────────────────────────────────────
blue   = [0.10 0.35 0.85];
red    = [0.80 0.10 0.10];
green  = [0.05 0.60 0.20];
purple = [0.50 0.15 0.78];

h(1) = plot(ax, t, lhip,   '-',  'Color',blue,   'LineWidth',2.2, 'DisplayName','Left hip pitch');
h(2) = plot(ax, t, rhip,   '--', 'Color',blue,   'LineWidth',2.2, 'DisplayName','Right hip pitch');
h(3) = plot(ax, t, lknee,  '-',  'Color',red,    'LineWidth',2.2, 'DisplayName','Left knee');
h(4) = plot(ax, t, rknee,  '--', 'Color',red,    'LineWidth',2.2, 'DisplayName','Right knee');
h(5) = plot(ax, t, lankle, '-',  'Color',green,  'LineWidth',2.0, 'DisplayName','Left ankle');
h(6) = plot(ax, t, rankle, '--', 'Color',green,  'LineWidth',2.0, 'DisplayName','Right ankle');
h(7) = plot(ax, t, lroll,  '-',  'Color',purple, 'LineWidth',1.6, 'DisplayName','Left hip roll');
h(8) = plot(ax, t, rroll,  '--', 'Color',purple, 'LineWidth',1.6, 'DisplayName','Right hip roll');

%% ── ANNOTATIONS ──────────────────────────────────────────────────
%text(ax, 32, 15.2, sprintf('Knee: %d°', KNEE),    'Color',red,    'FontSize',11,'FontWeight','bold');
%text(ax, 56, 13.5, sprintf('Hip: \\pm%d°', HIP),  'Color',blue,   'FontSize',11,'FontWeight','bold');
%text(ax, 56, -2.8, sprintf('Ankle: \\pm%d°', ANKLE),'Color',green,'FontSize',11,'FontWeight','bold');
%text(ax,  1,  4.0, sprintf('Roll: \\pm%d°', ROLL),'Color',purple, 'FontSize',11,'FontWeight','bold');

%% ── AXES ─────────────────────────────────────────────────────────
hold(ax,'off');
xlim(ax,[0 100]);
ylim(ax,ylims);
xlabel(ax,'Gait cycle (%)','FontSize',18,'FontName','Times New Roman');

ylabel(ax,'Joint angle (degrees)','FontSize',18,'FontName','Times New Roman');


grid(ax,'on');
ax.GridColor     = [0.88 0.88 0.88];
ax.GridLineStyle = ':';
ax.GridAlpha     = 0.30;
ax.Box           = 'on';
ax.LineWidth     = 1.2;
ax.XTick         = 0:10:100;
ax.YTick         = -30:5:30;
ax.FontSize      = 16;
ax.FontName      = 'Times New Roman';
ax.XColor        = 'k';
ax.YColor        = 'k';
ax.Layer         = 'top';

%% ── LEGEND — 2 columns, all 8 entries named ──────────────────────
legend(ax, h, ...
    'Left hip pitch','Right hip pitch', ...
    'Left knee',     'Right knee', ...
    'Left ankle',    'Right ankle', ...
    'Left hip roll', 'Right hip roll', ...
    'Location','northeast','NumColumns',4, ...
    'FontSize',14,'FontName','Times New Roman', ...
    'Box','on', 'EdgeColor','k');
    set(findall(fig,'Type','text'),'FontName','Times New Roman');

%% ── EXPORT  ───────────────────────────────────
set(fig,'PaperPositionMode','auto');
print(fig,'gait_trajectory','-dpng','-r600');

fprintf('\nParameters used:\n');
fprintf('  HIP_SWING_DEG  = %d\n', HIP);
fprintf('  KNEE_LIFT_DEG  = %d\n', KNEE);
fprintf('  ANKLE_COMP_DEG = %d\n', ANKLE);
fprintf('  HIP_ROLL_DEG   = %d\n', ROLL);