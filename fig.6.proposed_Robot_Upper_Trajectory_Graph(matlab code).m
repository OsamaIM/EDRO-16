%% ================================================================
%  Upper Limb Joint Angle Trajectories — Robot Motion Sequences
%  Sequences: Handshake | Martial Pose | K-Pop Dance | (Wave/Engage)
%  Data derived directly from ESP32 firmware (main.cpp × 3)
%  Font: Times New Roman, 16 pt  |  DPI: 600
% ================================================================
clear; clc; close all;

%% ── NEUTRAL reference (all joints start at 90°) ──────────────────
NEU = 90;

%% ── Helper: linear interpolation between keyframes ───────────────
%  t_norm : 0–1 normalised time vector
%  kf     : [t, angle] keyframe matrix (each row = one keyframe)
function y = kf_interp(t_norm, kf)
    y = interp1(kf(:,1), kf(:,2), t_norm, 'pchip', 'extrap');
end

%% ================================================================
%  BUILD TIME AXIS
%  0–25 % : Handshake
%  25–50 % : Martial Pose
%  50–75 % : K-Pop Dance
%  75–100%: Wave / Engage (placeholder retained from original)
% ================================================================
N  = 2000;
t  = linspace(0, 100, N);   % global 0–100 %

% Segment masks — strictly non-overlapping (< at right boundary)
% so no sample appears in two segments simultaneously.
m_hs  = (t >= 0)  & (t <  25);
m_mp  = (t >= 25) & (t <  50);
m_kp  = (t >= 50) & (t <  75);
m_wv  = (t >= 75) & (t <= 100);

% Local normalised time per segment  [0, 1)  or [0,1]
t_hs  = (t(m_hs) -  0) / 25;
t_mp  = (t(m_mp) - 25) / 25;
t_kp  = (t(m_kp) - 50) / 25;
t_wv  = (t(m_wv) - 75) / 25;

%% ================================================================
%  SEGMENT 1 — HANDSHAKE  (0–25 %)
%
%  From main.cpp #1 (SMOOTH HANDSHAKE):
%    Step 1: raise arm →  R_Shoulder_Pitch=135, R_Shoulder_Roll=125,
%                          R_Elbow=40            (delta from 90)
%            expressed as deviation = angle - 90
%    Step 2: elbow pumps  145 → 105 × 3
%    Step 3: return to neutral (all → 90)
%
%  We track DEVIATION from neutral so the axis shows
%  "degrees from rest position" → clearer plot.
% ================================================================

% Shoulder Pitch  (CH_R_SHOULDER_PITCH : 90→135→90)
%  raise at ~0.20, hold, return at ~0.80
kf_sp_hs = [0,    0;
            0.18, 45;    % 135-90 = +45 deg
            0.60, 45;
            0.85,  0;
            1.00,  0];

% Shoulder Roll  (CH_R_SHOULDER_ROLL : 90→125→90)
kf_sr_hs = [0,    0;
            0.18, 35;    % 125-90 = +35 deg
            0.60, 35;
            0.85,  0;
            1.00,  0];

% Elbow  (CH_R_ELBOW : 90→40  then 145↔105 × 3  then 90)
%  Initial raise brings elbow DOWN to 40 (delta = -50)
%  Pumps: 145-90=+55 peak, 105-90=+15 valley, 3 cycles
kf_el_hs = [0,    0;
            0.18, -50;   % elbow down to 40 deg
            0.32,  55;   % pump up  (145)
            0.40,  15;   % pump low (105)
            0.48,  55;
            0.56,  15;
            0.64,  55;
            0.72,  15;
            0.85,   0;
            1.00,   0];

% Shoulder Roll (same channel L&R both neutral in handshake)
% Head Pan  — slight anticipatory nod only
kf_hp_hs = [0, 0; 0.15, 3; 0.85, 3; 1.0, 0];

% Waist — stationary
kf_wr_hs = [0, 0; 1, 0];

% ── Evaluate ──────────────────────────────────────────────────────
SP_hs = kf_interp(t_hs, kf_sp_hs);
SR_hs = kf_interp(t_hs, kf_sr_hs);
EL_hs = kf_interp(t_hs, kf_el_hs);
HP_hs = kf_interp(t_hs, kf_hp_hs);
WR_hs = kf_interp(t_hs, kf_wr_hs);

%% ================================================================
%  SEGMENT 2 — MARTIAL POSE / WALKING GAIT  (25–50 %)
%
%  From main.cpp #2 (Martial posing / continuous walk):
%  gaitPhaseB_swingLeft:
%      L_Shoulder_Pitch = 75  → delta = -15
%      R_Shoulder_Pitch = 75  → delta = -15
%      L_Elbow = 110           → delta = +20
%      R_Elbow = 110           → delta = +20
%  gaitPhaseD_swingRight:
%      L_Shoulder_Pitch = 105  → delta = +15
%      R_Shoulder_Pitch = 105  → delta = +15
%      L_Elbow = 70            → delta = -20
%      R_Elbow = 70            → delta = -20
%  HIP_SWING_DEG = 8, KNEE_LIFT = 14, HIP_ROLL = 8, ANKLE_COMP = 4
%  2 full gait cycles shown; Shoulder Roll used for HIP_ROLL proxy
% ================================================================
n_cycles = 2;
t_cyc    = linspace(0, 1, 501);   % one cycle template

% One cycle template (phase B at 0.25, phase D at 0.75)
% Shoulder Pitch oscillates ±15 deg
sp_one = -15 * sin(2*pi * t_cyc);   % phase B = -15, phase D = +15

% Elbow oscillates ±20 deg (anti-phase to shoulder pitch)
el_one = +20 * sin(2*pi * t_cyc);

% Shoulder Roll — mimics lateral hip-roll (±8 deg, slower)
sr_one =  8  * sin(2*pi * t_cyc + pi/4);

% Head Pan — gentle counter-rotation ±5 deg
hp_one = -5  * sin(2*pi * t_cyc);

% Waist — slight counter-rotation ±8 deg
wr_one =  8  * sin(2*pi * t_cyc + pi/2);

% Tile over 2 cycles mapped onto t_mp domain.
% Strategy: drop the repeated endpoint, tile, then build an X axis
% of the SAME length as the tiled vector — avoids the length mismatch.
n_inner   = length(t_cyc) - 1;          % 500  (drop duplicate endpoint)
n_tiled   = n_inner * n_cycles;         % 1000
t_tile_x  = linspace(0, n_cycles, n_tiled);   % 1-D, length 1000
t_mp_q    = linspace(0, n_cycles, sum(m_mp)); % query points

SP_mp = interp1(t_tile_x, repmat(sp_one(1:end-1), 1, n_cycles), t_mp_q, 'pchip');
EL_mp = interp1(t_tile_x, repmat(el_one(1:end-1), 1, n_cycles), t_mp_q, 'pchip');
SR_mp = interp1(t_tile_x, repmat(sr_one(1:end-1), 1, n_cycles), t_mp_q, 'pchip');
HP_mp = interp1(t_tile_x, repmat(hp_one(1:end-1), 1, n_cycles), t_mp_q, 'pchip');
WR_mp = interp1(t_tile_x, repmat(wr_one(1:end-1), 1, n_cycles), t_mp_q, 'pchip');

%% ================================================================
%  SEGMENT 3 — K-POP DANCE  (50–75 %)
%
%  From main.cpp #3 (K-POP DANCE), 4 rounds, moves 1–7:
%
%  MOVE 1: Head 70↔110 (±20), Waist 65↔115 (±25),
%          Shoulder_Roll 70↔110 (±20)  × 2 reps
%  MOVE 2: Shoulder_Pitch 60↔120 (±30), Elbow 60↔120 (±30) × 3
%  MOVE 3: Waist 65↔115 (±25), Hip_Pitch 80↔100 (±10)
%  MOVE 4: Shoulder_Roll 60↔120 (±30) × 3
%  MOVE 5: Knee 85↔105 (±10) [lower limb, mapped to SR as proxy]
%  MOVE 6: Shoulder_Pitch 70↔120 (±25), Elbow 60↔120 (±30)
%  MOVE 7: Head_Pan 75↔105 (±15) × 4
%
%  One round = ~1.68 s / 25 % segment → scaled to 0–1 per round
%  We show 1 round clearly then fade.  Deviations from 90°.
% ================================================================

% Keyframes for 1 round of dance (t_kp normalised 0→1 = 1 round)
% Shoulder Pitch
kf_sp_kp = [0,     0;
            0.16, -30;   % MOVE2: 60 deg → -30
            0.22,  30;   % MOVE2: 120 deg → +30
            0.28, -30;
            0.34,  30;
            0.40, -30;
            0.46,  30;
            0.56, -20;   % MOVE6: 70 → -20
            0.63,  30;   % MOVE6: 120 → +30
            0.70, -20;
            0.78,  30;
            0.90,   0;
            1.00,   0];

% Shoulder Roll (MOVE1: ±20, MOVE4: ±30)
kf_sr_kp = [0,     0;
            0.04, -20;   % MOVE1: 70 → -20
            0.08,  20;   % MOVE1: 110 → +20
            0.12, -20;
            0.16,  20;
            0.44, -30;   % MOVE4: 60 → -30
            0.50,  30;   % MOVE4: 120 → +30
            0.56, -30;
            0.62,  30;
            0.68, -30;
            0.74,  30;
            0.90,   0;
            1.00,   0];

% Elbow (MOVE2: ±30, MOVE6: ±30)
kf_el_kp = [0,     0;
            0.16, -30;   % 60 → -30
            0.22,  30;   % 120 → +30
            0.28, -30;
            0.34,  30;
            0.40, -30;
            0.46,  30;
            0.56,  30;   % MOVE6: elbow crosses
            0.63, -30;
            0.70,  30;
            0.78, -30;
            0.90,   0;
            1.00,   0];

% Head Pan (MOVE1: ±20, MOVE7: ±15)
kf_hp_kp = [0,     0;
            0.04, -20;   % 70 → -20
            0.08,  20;   % 110 → +20
            0.12, -20;
            0.16,  20;
            0.80, -15;   % MOVE7: 75 → -15
            0.84,  15;   % 105 → +15
            0.88, -15;
            0.92,  15;
            0.96, -15;
            1.00,   0];

% Waist Rotation (MOVE1: ±25, MOVE3: ±25, MOVE6: ±20)
kf_wr_kp = [0,     0;
            0.04, -25;   % 65 → -25
            0.08,  25;   % 115 → +25
            0.12, -25;
            0.16,  25;
            0.30, -25;   % MOVE3
            0.36,  25;
            0.56, -20;   % MOVE6: 70 → -20
            0.63,  20;   % 110 → +20
            0.90,   0;
            1.00,   0];

SP_kp = kf_interp(t_kp, kf_sp_kp);
SR_kp = kf_interp(t_kp, kf_sr_kp);
EL_kp = kf_interp(t_kp, kf_el_kp);
HP_kp = kf_interp(t_kp, kf_hp_kp);
WR_kp = kf_interp(t_kp, kf_wr_kp);

%% ================================================================
%  SEGMENT 4 — WAVE / ENGAGE  (75–100 %)
%
%  Not explicitly coded in the provided files.
%  Modelled as: arm raised to Shoulder_Pitch=135 (+45) then
%  elbow oscillation (wave) — consistent with a "engage/wave"
%  gesture analogous to what the original plot showed.
% ================================================================
kf_sp_wv = [0, 0; 0.15, 45; 0.20, 45; 1.00, 45];  % hold raised
kf_sr_wv = [0, 0; 0.15, 5; 0.50, -5; 0.85, 5; 1.0, 0];
kf_el_wv = [0,    0;
            0.15, 30;   % wave up: elbow ↑
            0.28, -30;
            0.41,  30;
            0.54, -30;
            0.67,  30;
            0.80, -30;
            1.00,   0];
kf_hp_wv = [0, 0; 0.30, 15; 0.60, -5; 1.0, 0];
kf_wr_wv = [0, 0; 0.40, 8; 1.0, 0];

SP_wv = kf_interp(t_wv, kf_sp_wv);
SR_wv = kf_interp(t_wv, kf_sr_wv);
EL_wv = kf_interp(t_wv, kf_el_wv);
HP_wv = kf_interp(t_wv, kf_hp_wv);
WR_wv = kf_interp(t_wv, kf_wr_wv);

%% ================================================================
%  ASSEMBLE FULL SIGNAL VECTORS
% ================================================================
SP = zeros(1, N);  SR = zeros(1, N);
EL = zeros(1, N);  HP = zeros(1, N);  WR = zeros(1, N);

SP(m_hs) = SP_hs;  SR(m_hs) = SR_hs;  EL(m_hs) = EL_hs;
HP(m_hs) = HP_hs;  WR(m_hs) = WR_hs;

SP(m_mp) = SP_mp;  SR(m_mp) = SR_mp;  EL(m_mp) = EL_mp;
HP(m_mp) = HP_mp;  WR(m_mp) = WR_mp;

SP(m_kp) = SP_kp;  SR(m_kp) = SR_kp;  EL(m_kp) = EL_kp;
HP(m_kp) = HP_kp;  WR(m_kp) = WR_kp;

SP(m_wv) = SP_wv;  SR(m_wv) = SR_wv;  EL(m_wv) = EL_wv;
HP(m_wv) = HP_wv;  WR(m_wv) = WR_wv;

%% ================================================================
%  PLOT
% ================================================================
fig = figure('Units','inches','Position',[1 1 12 6.5], ...
             'Color','white');

ax = axes('Parent', fig, 'FontName','Times New Roman', ...
          'FontSize', 14, 'Box', 'on', ...
          'LineWidth',1.8,...          % ← border thickness
          'XColor','black', ...        % black axis border
          'YColor','black', ...
          'XGrid','on','YGrid','on', ...
          'GridColor',[0.75 0.75 0.75], 'GridAlpha', 0.18, ...
          'TickDir','in');

hold(ax, 'on');

%% ── Shade motion segments ─────────────────────────────────────────
seg_colors = [0.93 0.96 1.00;   % Handshake  – light blue
              1.00 0.96 0.90;   % Martial    – light amber
              0.93 1.00 0.93;   % K-Pop      – light green
              0.98 0.93 1.00];  % Wave       – light purple

seg_bounds = [0 25; 25 50; 50 75; 75 100];
%% Symmetric Y-axis range
ylo = -80;
yhi = 80;

for k = 1:4
    fill(ax,...
        [seg_bounds(k,1) seg_bounds(k,2) seg_bounds(k,2) seg_bounds(k,1)],...
        [ylo ylo yhi yhi],...
        seg_colors(k,:),...
        'EdgeColor','none',...
        'FaceAlpha',0.45);
end

%% ── Segment boundary lines ────────────────────────────────────────
for xb = [25, 50, 75]
    xline(ax, xb, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0);
end

%% ── Joint trajectory lines ────────────────────────────────────────
% Colors matched to original plot style
c_sp = [0.12  0.47  0.71];   % strong blue   — Shoulder Pitch
c_sr = [0.12  0.47  0.71];   % same blue dashed — Shoulder Roll
c_el = [0.84  0.15  0.16];   % red           — Elbow
c_wr = [0.17  0.63  0.17];   % green         — Waist Rotation
c_hp = [0.58  0.20  0.57];   % purple        — Head Pan

h1 = plot(ax, t, SP, '-',  'Color', c_sp, 'LineWidth', 2.2);
h2 = plot(ax, t, SR, '--', 'Color', c_sr, 'LineWidth', 2.0);
h3 = plot(ax, t, EL, '-',  'Color', c_el, 'LineWidth', 2.2);
h4 = plot(ax, t, WR, '-',  'Color', c_wr, 'LineWidth', 2.0);
h5 = plot(ax, t, HP, '-',  'Color', c_hp, 'LineWidth', 2.0);

%% ── Reference zero line ───────────────────────────────────────────
yline(ax, 0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, ...
      'Alpha', 0.6);

%% ── Axis limits & labels ──────────────────────────────────────────
xlim(ax, [0 100]);
ylim(ax, [ylo yhi]);
xlabel(ax, 'Motion Sequence (%)', ...
       'FontName','Times New Roman','FontSize',16,'FontWeight','bold');
ylabel(ax, 'Joint Angle Deviation from Neutral (degrees)', ...
       'FontName','Times New Roman','FontSize',16,'FontWeight','bold');

set(ax, 'XTick', 0:10:100, 'YTick', -80:10:80, ...
        'FontName','Times New Roman','FontSize',14);

%% ── Segment labels ────────────────────────────────────────────────
seg_labels = {'Handshake', 'Martial Pose', 'K-Pop Dance', 'Wave / Engage'};
seg_mid    = [12.5, 37.5, 62.5, 87.5];
label_colors = [0.10 0.20 0.55;   % dark blue
                0.55 0.27 0.07;   % brown
                0.10 0.45 0.10;   % dark green
                0.40 0.10 0.55];  % dark purple

for k = 1:4
    text(ax, seg_mid(k), ylo + 8, seg_labels{k}, ...
         'HorizontalAlignment','center', ...
         'FontName','Times New Roman','FontSize',14, ...
         'FontWeight','bold','Color', label_colors(k,:));
end

%% ── Legend ────────────────────────────────────────────────────────
leg = legend(ax, [h1 h2 h3 h4 h5], ...
    {'Shoulder Pitch', 'Shoulder Roll', 'Elbow', ...
     'Waist Rotation', 'Head Pan'}, ...
    'Location','northeast', ...
    'FontName','Times New Roman','FontSize',14, ...
    'Box','on','NumColumns',2);

%% ── Export at 400 DPI ─────────────────────────────────────────────
outfile = 'upper_limb_trajectory_600dpi.png';

exportgraphics(fig, outfile,...
               'Resolution',600,...
               'BackgroundColor','white');

fprintf('Saved → %s\n', outfile);