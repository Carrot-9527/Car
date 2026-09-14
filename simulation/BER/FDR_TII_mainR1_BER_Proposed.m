% BER evaluation using only the Proposed TII attack detector.
%
% The Proposed detector, attack-free calibration, channel/noise seeds, and
% thresholds are inherited from FDR_TII_mainR1_SNR_compare.mat.  The BER
% attack channel follows the paper's power convention: pE is a power ratio,
% so sqrt(pE/pB) multiplies the normalized AE channel.  No ERD, RMT-FDC,
% SSSAD, or Random-pilot/MDL detector is evaluated here.
%
% The three attack-only curves are conditioned on H1, so their BER is not
% diluted by the 50% attack-free frames used to evaluate detector errors.
% For the three "with attack detection" curves, frames declared attacked
% by the Proposed detector are excluded before BER is accumulated; these
% curves therefore characterize all retained operational frames.
clearvars; close all force; clc;
tic;

rng(20260707);
data_file='FDR_TII_mainR1_SNR_compare.mat';
assert(isfile(data_file), ...
    'Missing %s. Run this script in the six-figure FDR project folder.',data_file);
base=load(data_file);

%% Detector configuration inherited from the six FDR figures
M=base.M;
EbN0=base.EbN0;
Numx=base.Numx;
spow=base.spow;
dd=base.dd;
base_pe_amplitude=base.pe;
pilot_attack_power=0.1;
pilot_attack_amplitude=sqrt(pilot_attack_power/spow);
pe=pilot_attack_amplitude;
combo_attack_boost=base.combo_attack_boost;
sense_error=base.sense_error;
theta=base.theta;
theta_p=base.theta_p;
N_iterD=base.N_iterD;
N_iterF=base.N_iterF;
N_h=base.N_h;
Krise=16;
Krisep=16;

payload_modulation='BPSK';
num_scenarios=3;
num_snr=numel(EbN0);
scenario_labels={'PSA';'DCISA';'PSA+DCISA'};
condition_labels={ ...
    'PSA & DCISA'; ...
    'DCISA'; ...
    'PSA'; ...
    'PSA & DCISA with attack detection'; ...
    'DCISA with attack detection'; ...
    'PSA with attack detection'; ...
    'Reliable CSI'};

%% Replay the pilot sequence, attack indicators, and attack-free channel bank
x_bits=randi([0,1],1,Numx);
x=2*x_bits-1;
pilot_energy=x*x';

attack_ind=repmat([0 1],1,N_iterF/2);
attack_ind=attack_ind(randperm(N_iterF));
assert(mod(N_iterF,2)==0,'The saved FDR test-trial count must be even.');

seed_D_bank=randi(2^31-1,N_iterD,N_h);
seed_F_bank=randi(2^31-1,N_iterF,N_h);
seed_ref_bank=randi(2^31-1,1,N_h);
assert(isequal(seed_D_bank,base.seed_D_bank), ...
    'No-attack threshold seed replay mismatch.');
assert(isequal(seed_F_bank,base.seed_F_bank), ...
    'FDR test seed replay mismatch.');
assert(isequal(seed_ref_bank,base.seed_ref_bank), ...
    'Reference seed replay mismatch.');

array=linspace(0,M-1,M);
h0_bank=zeros(M,N_h);
g_psa_bank=zeros(M,N_h);
g_psa_upm_bank=zeros(M,N_h);
upm_profile_bank=zeros(M,N_h);
for h=1:N_h
    h_LoS=exp(1i*2*pi*dd*sin(theta).'*array).';
    h_NLoS=sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
    h0_bank(:,h)=sqrt(Krise/(1+Krise))*h_LoS+ ...
        sqrt(1/(1+Krise))*h_NLoS;

    % Consume the same sensing-error draw used in the FDR simulation.
    unused_hs=h0_bank(:,h)+sense_error*sqrt(0.5)* ...
        (randn(M,1)+1i*randn(M,1)); %#ok<NASGU>

    g_LoS=exp(1i*2*pi*dd*sin(theta_p).'*array).';
    g_NLoS=sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
    g_nominal=pe*(sqrt(Krisep/(1+Krisep))*g_LoS+ ...
        sqrt(1/(1+Krisep))*g_NLoS);
    g_combo=combo_attack_boost*g_nominal;
    g_psa_bank(:,h)=g_nominal;
    g_psa_upm_bank(:,h)=g_combo;
    upm_profile_bank(:,h)=powerMatchGain(h0_bank(:,h),g_combo)*ones(M,1);
end

% The H1 channel banks intentionally differ from the six-FDR file because
% that file stores base.pe as an amplitude coefficient.  Here pE=0.1 is the
% AE power ratio in the paper's channel-estimation model.

%% Conditional BER accumulators
% For each Monte Carlo channel estimate, average the exact coherent-BPSK
% error probability. This is the infinite-payload limit of explicit bit
% counting and removes random count reversals near BER=1e-6.
ber_probability_sum=zeros(7,num_snr);
ber_frame_count=zeros(7,num_snr);
retained_frame_count=zeros(num_scenarios,num_snr);
detect_count=zeros(num_scenarios,num_snr);
false_alarm_count=zeros(num_scenarios,num_snr);
miss_count=zeros(num_scenarios,num_snr);

no_attack_trials=sum(attack_ind==0)*N_h;
attack_trials=sum(attack_ind==1)*N_h;
total_frames=N_iterF*N_h;

%% Proposed detection and BER accumulation
for isnr=1:num_snr
    snr_db=EbN0(isnr);
    noise_mag=sqrt(spow*10^(-snr_db/10));
    fprintf('Proposed BER: M=%d, SNR=%g dB (%d/%d)\n', ...
        M,snr_db,isnr,num_snr);

    for h=1:N_h
        h0=h0_bank(:,h);
        h_sensed=base.hs_bank(:,h);
        h_LoS=exp(1i*2*pi*dd*sin(theta).'*array).';
        g_psa=g_psa_bank(:,h);
        g_psa_upm=g_psa_upm_bank(:,h);
        upm_gain=upm_profile_bank(:,h);

        h_attack=zeros(M,num_scenarios);
        h_attack(:,1)=h0+g_psa;
        h_attack(:,2)=upm_gain.*h0;
        h_attack(:,3)=upm_gain.*h0+g_psa_upm;

        % Actual payload channels: PSA contaminates only the channel
        % estimate, while DCISA reduces the legitimate payload power.
        h_payload=zeros(M,num_scenarios);
        h_payload(:,1)=h0;
        h_payload(:,2)=upm_gain.*h0;
        h_payload(:,3)=upm_gain.*h0;

        ind_hr11=(find(real(h_LoS)>=0 & imag(h_LoS)>=0)');
        ind_hr01=(find(real(h_LoS)<0 & imag(h_LoS)>=0)');
        ind_hr10=(find(real(h_LoS)>=0 & imag(h_LoS)<0)');
        ind_hr00=(find(real(h_LoS)<0 & imag(h_LoS)<0)');
        F0YX11=PDF2(h_sensed(ind_hr11),5,0.1,100);
        F0YX01=PDF2(h_sensed(ind_hr01),5,0.1,100);
        F0YX10=PDF2(h_sensed(ind_hr10),5,0.1,100);
        F0YX00=PDF2(h_sensed(ind_hr00),5,0.1,100);
        gate_tii=base.Gate_TII(h,isnr);

        for thf=1:N_iterF
            noise_p=noise_mag*complexNoiseFromSeed( ...
                M,Numx,seed_F_bank(thf,h));
            h_ls_na=((h0*x+noise_p)*x')/pilot_energy;

            reliable_ber=conditionalBpskBer(h0,h0,h0,noise_mag);
            ber_probability_sum(7,isnr)= ...
                ber_probability_sum(7,isnr)+reliable_ber;
            ber_frame_count(7,isnr)=ber_frame_count(7,isnr)+1;

            for scenario_idx=1:num_scenarios
                if attack_ind(thf)==0
                    h_est=h_ls_na;
                    h_data=h0;
                else
                    Y_x_real=h_attack(:,scenario_idx)*x+noise_p;
                    h_est=(Y_x_real*x')/pilot_energy;
                    h_data=h_payload(:,scenario_idx);
                end

                D_tii=tiiStatistic(h_est,ind_hr11,ind_hr01,ind_hr10,ind_hr00, ...
                    F0YX01,F0YX10,F0YX00,F0YX11,5,0.1,100);
                detected=D_tii>gate_tii;
                detect_count(scenario_idx,isnr)= ...
                    detect_count(scenario_idx,isnr)+detected;
                if attack_ind(thf)==0
                    false_alarm_count(scenario_idx,isnr)= ...
                        false_alarm_count(scenario_idx,isnr)+detected;
                else
                    miss_count(scenario_idx,isnr)= ...
                        miss_count(scenario_idx,isnr)+~detected;
                end

                scenario_ber=conditionalBpskBer( ...
                    h_est,h_data,h0,noise_mag);
                no_detection_row=4-scenario_idx;
                with_detection_row=7-scenario_idx;
                if attack_ind(thf)==1
                    ber_probability_sum(no_detection_row,isnr)= ...
                        ber_probability_sum(no_detection_row,isnr)+scenario_ber;
                    ber_frame_count(no_detection_row,isnr)= ...
                        ber_frame_count(no_detection_row,isnr)+1;
                end

                if ~detected
                    ber_probability_sum(with_detection_row,isnr)= ...
                        ber_probability_sum(with_detection_row,isnr)+scenario_ber;
                    ber_frame_count(with_detection_row,isnr)= ...
                        ber_frame_count(with_detection_row,isnr)+1;
                    retained_frame_count(scenario_idx,isnr)= ...
                        retained_frame_count(scenario_idx,isnr)+1;
                end
            end
        end
    end
end

BER_results=ber_probability_sum./max(ber_frame_count,1);
BER_plot_floor=1e-12*ones(size(BER_results));
BER_display=max(BER_results,BER_plot_floor);
retained_frame_fraction=retained_frame_count/total_frames;
PFA_results=false_alarm_count/no_attack_trials;
PMD_results=miss_count/attack_trials;
FDR_detector_results=(false_alarm_count+miss_count)/total_frames;

expected_pfa=squeeze(base.PFA_test_results(:,1,:));
detector_h0_replay_max_error=max(abs(PFA_results(:)-expected_pfa(:)));
assert(detector_h0_replay_max_error<1e-12, ...
    'Proposed detector H0 replay does not match the six-figure FDR data.');

fprintf('\nBER at 5 dB (paper-style order):\n');
snr5_idx=find(EbN0==5,1);
for row=1:numel(condition_labels)
    fprintf('%-40s %.8g\n',condition_labels{row},BER_results(row,snr5_idx));
end
fprintf('Detector H0 replay max error: %.3g\n',detector_h0_replay_max_error);

%% Paper-style BER figure
colors=[0.0000 0.4470 0.7410;
        0.8500 0.3250 0.0980;
        0.9290 0.6940 0.1250;
        0.4940 0.1840 0.5560;
        0.4660 0.6740 0.1880;
        0.3010 0.7450 0.9330;
        0.6350 0.0780 0.1840];
markers={'o','s','^','o','s','^','d'};
line_styles={'-','-','-','--','--','--','-.'};
filled=[true true true false false false false];

fig=figure('Color','w','Name','Proposed BER', ...
    'Position',[100 100 650 390]);
ax=axes(fig);
hold(ax,'on');
line_handles=gobjects(7,1);
for row=1:7
    marker_face='none';
    if filled(row)
        marker_face=colors(row,:);
    end
    line_handles(row)=semilogy(ax,EbN0,BER_display(row,:), ...
        'LineWidth',1.8,'LineStyle',line_styles{row}, ...
        'Marker',markers{row}, ...
        'MarkerSize',5.8,'MarkerFaceColor',marker_face, ...
        'MarkerEdgeColor',colors(row,:),'Color',colors(row,:), ...
        'Clipping','off','DisplayName',condition_labels{row});
end
hold(ax,'off');
formatMainAxes(ax);
xlim(ax,[min(EbN0) max(EbN0)]);
plot_y_limits=[min(BER_display(:)) max(BER_display(:))];
ylim(ax,plot_y_limits);
xticks(ax,EbN0);
xlabel(ax,'SNR/dB','FontSize',13);
ylabel(ax,'BER','FontSize',13);
lgd=legend(ax,line_handles,condition_labels,'Location','southwest', ...
    'FontSize',8.2,'Box','on');
lgd.Color='w';
lgd.EdgeColor='k';
lgd.LineWidth=0.8;

% Both insets are plain magnifications of the original absolute-BER curves.
% The lower inset uses the two original samples at 0 and 1 dB, preserving
% their true horizontal coordinates and line segments without normalization.
focus_xlim=[4 6];
bottom_xlim=[0 1];
top_pos=[0.70 0.68 0.26 0.22];
bottom_pos=[0.52 0.33 0.32 0.26];
addBerInset(fig,EbN0,BER_display,1:3,focus_xlim,top_pos, ...
    colors,markers,line_styles,filled);
addBerInset(fig,EbN0,BER_display,4:7,bottom_xlim,bottom_pos, ...
    colors,markers,line_styles,filled);

target_top=mean(BER_display(1:3,snr5_idx));
snr1_idx=find(EbN0==1,1);
target_bottom=mean(BER_display([4 6 7],snr1_idx));
[tx1,ty1]=dataToNormalized(ax,5,target_top);
[tx2,ty2]=dataToNormalized(ax,1,target_bottom);
annotation(fig,'arrow',[top_pos(1) tx1],[top_pos(2) ty1], ...
    'Color','k','LineWidth',1.0,'HeadLength',7,'HeadWidth',7);
annotation(fig,'arrow',[bottom_pos(1) tx2], ...
    [bottom_pos(2)+bottom_pos(4) ty2], ...
    'Color','k','LineWidth',1.0,'HeadLength',7,'HeadWidth',7);

%% Save data and results
artifact_base='BER_Proposed_final';
pdf_output_dir=fullfile('output','pdf');
if ~isfolder(pdf_output_dir)
    mkdir(pdf_output_dir);
end

save('BER_TII_mainR1_Proposed.mat', ...
    'BER_results','BER_display','BER_plot_floor', ...
    'ber_probability_sum','ber_frame_count', ...
    'condition_labels','scenario_labels','EbN0','plot_y_limits', ...
    'M','Numx','spow','base_pe_amplitude', ...
    'pilot_attack_power','pilot_attack_amplitude','combo_attack_boost', ...
    'payload_modulation','N_iterF','N_h','attack_ind', ...
    'retained_frame_fraction','PFA_results','PMD_results', ...
    'FDR_detector_results','expected_pfa','detector_h0_replay_max_error', ...
    'data_file');

set(fig,'Units','pixels','Position',[100 100 650 390], ...
    'PaperUnits','inches','PaperSize',[6.5 3.9], ...
    'PaperPosition',[0 0 6.5 3.9], ...
    'PaperPositionMode','manual','InvertHardcopy','off');
drawnow;
savefig(fig,[artifact_base '.fig']);
print(fig,[artifact_base '.png'],'-dpng','-r300');
print(fig,fullfile(pdf_output_dir,[artifact_base '.pdf']), ...
    '-dpdf','-painters','-r300');
toc;

function D=tiiStatistic(h_ls,ind_hr11,ind_hr01,ind_hr10,ind_hr00, ...
    F0YX01,F0YX10,F0YX00,F0YX11,domi,delta,T)
F0YX11_real=PDF2(h_ls(ind_hr11),domi,delta,T);
F0YX01_real=PDF2(h_ls(ind_hr01),domi,delta,T);
F0YX10_real=PDF2(h_ls(ind_hr10),domi,delta,T);
F0YX00_real=PDF2(h_ls(ind_hr00),domi,delta,T);
F11=abs(F0YX11_real-F0YX11);
F01=abs(F0YX01_real-F0YX01);
F10=abs(F0YX10_real-F0YX10);
F00=abs(F0YX00_real-F0YX00);
D=(1/T)^2*sum(sum((1/4)*(F11+F10+F01+F00)));
end

function gain=powerMatchGain(h0,g)
hh=real(h0'*h0);
hg=real(h0'*g);
gg=real(g'*g);
candidate_roots=roots([hh,2*hg,gg-hh]);
candidate_roots=real(candidate_roots( ...
    abs(imag(candidate_roots))<1e-10 & real(candidate_roots)>0));
if isempty(candidate_roots)
    gain=1;
else
    gain=min(candidate_roots);
end
end

function noise=complexNoiseFromSeed(M,N,seed)
stream=RandStream('mt19937ar','Seed',double(seed));
noise=sqrt(1/2)*(randn(stream,M,N)+1i*randn(stream,M,N));
end

function ber=conditionalBpskBer(h_est,h_data,h_nominal,noise_mag)
estimate_energy=max(real(h_est'*h_est),eps);
w=h_est'/estimate_energy;
signal_gain=w*h_data;

% Normalize the reliable-CSI combiner to the requested SNR. Any channel-
% estimate magnitude error then appears as the corresponding noise
% enhancement, while pilot contamination also introduces direction error.
noise_enhancement=sqrt(real(h_nominal'*h_nominal))*norm(w);
effective_sigma=max(noise_mag*noise_enhancement,eps);
ber=0.5*erfc(real(signal_gain)/effective_sigma);
end

function formatMainAxes(ax)
grid(ax,'on');
box(ax,'on');
ax.YScale='log';
ax.YMinorGrid='on';
ax.YMinorTick='on';
ax.XGrid='on';
ax.XMinorGrid='off';
ax.GridLineStyle='-';
ax.MinorGridLineStyle=':';
ax.GridAlpha=0.24;
ax.MinorGridAlpha=0.28;
ax.LineWidth=1.0;
ax.TickDir='in';
ax.Layer='top';
ax.XAxisLocation='bottom';
ax.YAxisLocation='left';
ax.FontName='Times New Roman';
ax.FontSize=12;
end

function addBerInset(fig,x,ber,rows,x_limits,pos, ...
    colors,markers,line_styles,filled)
inset=axes(fig,'Position',pos); %#ok<LAXES>
hold(inset,'on');
focus_idx=find(x>=x_limits(1) & x<=x_limits(2));
for row=rows
    marker_face='none';
    if filled(row)
        marker_face=colors(row,:);
    end
    plot(inset,x,ber(row,:),'LineWidth',1.45, ...
        'LineStyle',line_styles{row},'Color',colors(row,:), ...
        'Marker',markers{row},'MarkerIndices',focus_idx, ...
        'MarkerSize',4.2,'MarkerFaceColor',marker_face, ...
        'MarkerEdgeColor',colors(row,:));
end
hold(inset,'off');
box(inset,'on');
grid(inset,'on');
xlim(inset,x_limits);
xticks(inset,[x_limits(1) mean(x_limits) x_limits(2)]);
values=ber(rows,focus_idx);
value_min=min(values,[],'all');
value_max=max(values,[],'all');
span=max(value_max-value_min,0.02*value_max);
ylim(inset,[max(value_min-0.08*span,eps),value_max+0.08*span]);
inset.GridAlpha=0.22;
inset.LineWidth=0.85;
inset.TickDir='in';
inset.Layer='top';
inset.FontName='Times New Roman';
inset.FontSize=8.5;
end

function [xn,yn]=dataToNormalized(ax,x,y)
pos=ax.Position;
xl=ax.XLim;
yl=ax.YLim;
xn=pos(1)+pos(3)*(x-xl(1))/(xl(2)-xl(1));
yn=pos(2)+pos(4)*(log10(y)-log10(yl(1)))/(log10(yl(2))-log10(yl(1)));
end
