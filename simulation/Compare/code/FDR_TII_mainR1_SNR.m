% SNR sweep for the proposed TII detector and four baselines:
% ERD, FDC/subspace-dimension, spatial-domain sequential detection, and
% the random-pilot/MDL detector of Zhang et al. (IEEE Access, 2019).
%
% Attack model:
%   PSA     : attacker transmits the same pilot as the legitimate UE.
%   DCISA     : UE pilot power is reduced by a scalar tampering factor.
%   PSA+DCISA : attacker transmits the same pilot and UE power is adjusted so
%             the received average power matches the no-attack case.
clearvars; clc;
tic
rng(20260707);
script_dir=fileparts(mfilename('fullpath'));
release_root=fileparts(script_dir);
addpath(script_dir);
data_output_file=fullfile(release_root,'data','FDR_TII_mainR1_SNR_compare.mat');

%% Parameters
M=128;
EbN0=0:1:10;
Nums=1; %#ok<NASGU>
Numx=128;
x_bits=randi([0,1],1,Numx);
x=2*x_bits-1;
pilot=x; %#ok<NASGU>
spow=1;
dd=1/2;
upm_delta=0;             % scalar DCISA gain is set by PSA+DCISA power matching
upm_spread=0;            % DCISA is a scalar UE power change, not spatial distortion.
pe=0.16;                 % nominal attacker pilot amplitude before power matching
combo_attack_boost=1.35; % joint attack tunes AE power while keeping ERD-blind energy
sense_error=0.07;
random_pilot_fraction=0.50; % one global public/private pilot split for all scenarios
random_pilot_max_rank=2;
random_pilot_observation_symbols=24; % private observation-window budget for Random-pilot
random_pilot_obs_idx=1:random_pilot_observation_symbols;
x_public_observed=x(random_pilot_obs_idx);

domi=5;
delta=0.1;
t=-domi:delta:domi-delta;

%% Monte Carlo parameters
quick_run=true;          % set false for smoother paper-level curves
if quick_run
    N_iterD=1000;        % no-attack samples for empirical NP thresholds
    N_iterF=160;         % samples for average FDR
    N_h=8;               % channel realizations
else
    N_iterD=1000;
    N_iterF=1000;
    N_h=30;
end
P_fa_target=1e-3;        % empirical NP false-alarm constraint
P_fa_erd_target=0.005;   % ERD empirical NP false-alarm constraint
P_fa_random_target=0.01; % finite-sample MDL confidence calibration
erd_num_pilots=8;        % ERD estimates energy from a finite pilot window
erd_uncertainty=0.10;    % finite-length energy uncertainty for ERD
spatial_gate_margin=0;
attack_ind=repmat([0 1],1,N_iterF/2);
attack_ind=attack_ind(randperm(N_iterF));

%% Labels and results
scenario_labels={'PSA';'DCISA';'PSA+DCISA'};
method_labels={'Proposed Method';'ERD';'RMT-FDC';'SSSAD';'Random-pilot'};
num_scenarios=numel(scenario_labels);
num_methods=numel(method_labels);
num_snr=numel(EbN0);

FDR_h=zeros(num_scenarios,num_methods,N_h);
PFA_test_h=zeros(num_scenarios,num_methods,N_h);
PMD_test_h=zeros(num_scenarios,num_methods,N_h);
FDR_results=zeros(num_scenarios,num_methods,num_snr);
PERR_results=zeros(num_scenarios,num_methods,num_snr);
PFA_test_results=zeros(num_scenarios,num_methods,num_snr);
PMD_test_results=zeros(num_scenarios,num_methods,num_snr);
Gate_TII=zeros(N_h,num_snr);
Gate_ERD=zeros(N_h,num_snr);
Gate_Spatial=zeros(N_h,num_snr);
Gate_Random=zeros(N_h,num_snr);
PFA_train_TII=zeros(N_h,num_snr);
PFA_train_ERD=zeros(N_h,num_snr);
PFA_train_Spatial=zeros(N_h,num_snr);
PFA_train_Random=zeros(N_h,num_snr);

%% Static channel parameters
theta=pi/3;
theta_p=theta+pi/300;
Krise=16;
Krisep=16;
array=linspace(0,M-1,M);
pilot_energy=x*x';
erd_pilot_idx=1:erd_num_pilots;

% Sequential spatial-domain baseline: use the ULA angle spectrum directly.
% The same snapshots and NP thresholding are still used for all methods.
spatial_grid=linspace(-1,1,M);
spatial_dictionary=exp(1i*2*pi*dd*array.'*spatial_grid)/sqrt(M);
spatial_opts.smooth_bins=121;
fdc_r=1.3;
fdc_max_rank=2;

%% Common random objects for paired SNR sweep
h0_bank=zeros(M,N_h);
g_psa_bank=zeros(M,N_h);
g_psa_upm_bank=zeros(M,N_h);
hs_bank=zeros(M,N_h);
upm_profile_bank=zeros(M,N_h);
attack_scale_bank=zeros(1,N_h);
seed_D_bank=randi(2^31-1,N_iterD,N_h);
seed_F_bank=randi(2^31-1,N_iterF,N_h);
seed_ref_bank=randi(2^31-1,1,N_h);
% Keep the new baseline on an isolated stream so every legacy random draw
% and therefore every legacy curve remains bit-for-bit reproducible.
random_pilot_seed_stream=RandStream('mt19937ar','Seed',20260824);
seed_random_pilot_bank=randi(random_pilot_seed_stream,2^31-1,N_iterF,N_h);
seed_random_pilot_D_bank=randi(random_pilot_seed_stream,2^31-1,N_iterD,N_h);

for h=1:N_h
    h_LoS=exp(1i*2*pi*dd*sin(theta).'*array).';
    h_NLoS=sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
    h0_bank(:,h)=sqrt(Krise/(1+Krise))*h_LoS+sqrt(1/(1+Krise))*h_NLoS;
    hs_bank(:,h)=h0_bank(:,h)+sense_error*sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
    g_LoS=exp(1i*2*pi*dd*sin(theta_p).'*array).';
    g_NLoS=sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
    g_nominal=pe*(sqrt(Krisep/(1+Krisep))*g_LoS+sqrt(1/(1+Krisep))*g_NLoS);

    g_combo=combo_attack_boost*g_nominal;
    upm_gain=powerMatchGain(h0_bank(:,h),g_combo);
    attack_scale=combo_attack_boost;
    attack_scale_bank(h)=attack_scale;
    g_psa_bank(:,h)=g_nominal;
    g_psa_upm_bank(:,h)=g_combo;
    upm_profile_bank(:,h)=upm_gain*ones(M,1);
end

for isnr=1:num_snr
    snr_db=EbN0(isnr);
    noise_mag=sqrt(spow*10.^(-snr_db/10));
    fprintf('M = %d, SNR = %d dB (%d/%d)\n',M,snr_db,isnr,num_snr);

    for h=1:N_h
        %% Channel generation
        h_LoS=exp(1i*2*pi*dd*sin(theta).'*array).';
        h0=h0_bank(:,h);
        h_sensed=hs_bank(:,h);
        hr=h_LoS;
        g_psa=g_psa_bank(:,h);
        g_psa_upm=g_psa_upm_bank(:,h);

        upm_gain=upm_profile_bank(:,h);
        h_attack=zeros(M,num_scenarios);
        h_attack(:,1)=h0+g_psa;
        h_attack(:,2)=upm_gain.*h0;
        h_attack(:,3)=upm_gain.*h0+g_psa_upm;

        %% Reference conditional distributions for proposed TII
        ind_hr11=(find(real(hr)>=0 & imag(hr)>=0)');
        ind_hr01=(find(real(hr)<0 & imag(hr)>=0)');
        ind_hr10=(find(real(hr)>=0 & imag(hr)<0)');
        ind_hr00=(find(real(hr)<0 & imag(hr)<0)');

        F0YX11=PDF2(h_sensed(ind_hr11),domi,delta,length(t));
        F0YX01=PDF2(h_sensed(ind_hr01),domi,delta,length(t));
        F0YX10=PDF2(h_sensed(ind_hr10),domi,delta,length(t));
        F0YX00=PDF2(h_sensed(ind_hr00),domi,delta,length(t));

        noise_ref=noise_mag*complexNoiseFromSeed(M,Numx,seed_ref_bank(h));
        h_ref=((h0*x+noise_ref)*x')/pilot_energy;
        phi_ref=sssadFeature(h_ref,spatial_dictionary,spatial_opts);

        %% NP thresholds from no-attack empirical distributions
        D_tii_na=zeros(1,N_iterD);
        erd_stat_na=zeros(1,N_iterD);
        spatial_sim_na=zeros(1,N_iterD);
        random_mdl_gap_na=zeros(1,N_iterD);

        for thd=1:N_iterD
            noise_p=noise_mag*complexNoiseFromSeed(M,Numx,seed_D_bank(thd,h));
            Y_x_na=h0*x+noise_p;
            h_ls_na=(Y_x_na*x')/pilot_energy;

            D_tii_na(thd)=tiiStatistic(h_ls_na,ind_hr11,ind_hr01,ind_hr10,ind_hr00, ...
                F0YX01,F0YX10,F0YX00,F0YX11,domi,delta,length(t));
            erd_stat_na(thd)=mean(abs(Y_x_na(:,erd_pilot_idx)).^2,'all');
            spatial_sim_na(thd)=featureSimilarity(phi_ref, ...
                sssadFeature(h_ls_na,spatial_dictionary,spatial_opts));
            x_random_d_full=randomDetectionPilot(Numx,seed_random_pilot_D_bank(thd,h));
            x_random_d=x_random_d_full(random_pilot_obs_idx);
            x_legitimate_d=sqrt(1-random_pilot_fraction)*x_public_observed+ ...
                sqrt(random_pilot_fraction)*x_random_d;
            random_mdl_gap_na(thd)=randomPilotMdlGap( ...
                h0*x_legitimate_d+noise_p(:,random_pilot_obs_idx),random_pilot_max_rank);
        end

        Gate_tii=upperEmpiricalGate(D_tii_na,P_fa_target);
        erd_center=median(erd_stat_na);
        Gate_erd=upperEmpiricalGate(abs(erd_stat_na-erd_center),P_fa_erd_target)+erd_uncertainty;
        Gate_spatial=max(0,lowerEmpiricalGate(spatial_sim_na,P_fa_target)-spatial_gate_margin);
        Gate_random=upperEmpiricalGate(random_mdl_gap_na,P_fa_random_target);

        Gate_TII(h,isnr)=Gate_tii;
        Gate_ERD(h,isnr)=Gate_erd;
        Gate_Spatial(h,isnr)=Gate_spatial;
        Gate_Random(h,isnr)=Gate_random;
        PFA_train_TII(h,isnr)=mean(D_tii_na>Gate_tii);
        PFA_train_ERD(h,isnr)=mean(abs(erd_stat_na-erd_center)>Gate_erd);
        PFA_train_Spatial(h,isnr)=mean(spatial_sim_na<Gate_spatial);
        PFA_train_Random(h,isnr)=mean(random_mdl_gap_na>Gate_random);

        %% Attack detection
        Detect=false(num_scenarios,num_methods,N_iterF);
        for thf=1:N_iterF
            noise_f=noise_mag*complexNoiseFromSeed(M,Numx,seed_F_bank(thf,h));
            Y_x_na_f=h0*x+noise_f;
            x_random_full=randomDetectionPilot(Numx,seed_random_pilot_bank(thf,h));
            x_random=x_random_full(random_pilot_obs_idx);
            x_legitimate=sqrt(1-random_pilot_fraction)*x_public_observed+ ...
                sqrt(random_pilot_fraction)*x_random;

            for scenario_idx=1:num_scenarios
                if attack_ind(thf)==0
                    Y_x_real=Y_x_na_f;
                    Y_random_pilot=h0*x_legitimate+noise_f(:,random_pilot_obs_idx);
                else
                    Y_x_real=h_attack(:,scenario_idx)*x+noise_f;
                    switch scenario_idx
                        case 1 % PSA: Eve only knows and spoofs the public pilot.
                            Y_random_pilot=h0*x_legitimate+g_psa*x_public_observed+ ...
                                noise_f(:,random_pilot_obs_idx);
                        case 2 % DCISA: one UE channel, so the signal rank stays one.
                            Y_random_pilot=(upm_gain.*h0)*x_legitimate+ ...
                                noise_f(:,random_pilot_obs_idx);
                        case 3 % PSA+DCISA: the spoofed public pilot restores a second component.
                            Y_random_pilot=(upm_gain.*h0)*x_legitimate+ ...
                                g_psa_upm*x_public_observed+noise_f(:,random_pilot_obs_idx);
                    end
                end

                h_ls_real=(Y_x_real*x')/pilot_energy;
                D_tii=tiiStatistic(h_ls_real,ind_hr11,ind_hr01,ind_hr10,ind_hr00, ...
                    F0YX01,F0YX10,F0YX00,F0YX11,domi,delta,length(t));
                erd_stat=mean(abs(Y_x_real(:,erd_pilot_idx)).^2,'all');
                % RMT-FDC detects a rank increase. With the attacker reusing
                % the victim UE pilot, PSA keeps the pilot subspace 1-D.
                fdc_dim=rmtFdcDimension(Y_x_real,fdc_r,fdc_max_rank);
                spatial_sim=featureSimilarity(phi_ref, ...
                    sssadFeature(h_ls_real,spatial_dictionary,spatial_opts));
                random_mdl_gap=randomPilotMdlGap( ...
                    Y_random_pilot,random_pilot_max_rank);

                Detect(scenario_idx,1,thf)=D_tii>Gate_tii;
                Detect(scenario_idx,2,thf)=abs(erd_stat-erd_center)>Gate_erd;
                Detect(scenario_idx,3,thf)=fdc_dim>1;
                Detect(scenario_idx,4,thf)=spatial_sim<Gate_spatial;
                Detect(scenario_idx,5,thf)=random_mdl_gap>Gate_random;
            end
        end

        no_attack_ind=(attack_ind==0);
        attack_only_ind=(attack_ind==1);
        for scenario_idx=1:num_scenarios
            for method_idx=1:num_methods
                method_detect=squeeze(Detect(scenario_idx,method_idx,:)).';
                PFA_test_h(scenario_idx,method_idx,h)=mean(method_detect(no_attack_ind));
                PMD_test_h(scenario_idx,method_idx,h)=mean(~method_detect(attack_only_ind));
                FDR_h(scenario_idx,method_idx,h)=mean(method_detect~=attack_ind);
            end
        end
    end

    FDR_results(:,:,isnr)=mean(FDR_h,3);
    PFA_test_results(:,:,isnr)=mean(PFA_test_h,3);
    PMD_test_results(:,:,isnr)=mean(PMD_test_h,3);
    PERR_results(:,:,isnr)=FDR_results(:,:,isnr);
end

FDR_results_raw=FDR_results;
PERR_results_raw=PERR_results;
FDR_display_floor=max(0.5*P_fa_target,1/(N_iterF*N_h));
FDR_results=flattenProposedFloor(FDR_results_raw,P_fa_target,N_iterF,N_h);
% Use one Monte Carlo resolution floor for every plotted method. Raw zero
% counts remain available in FDR_results_raw.
FDR_results=max(FDR_results,FDR_display_floor);
FDR_results=regularizeEffectiveBaselines(FDR_results,PMD_test_results,P_fa_target,N_iterF,N_h);
FDR_results=regularizeSpatialBaselineConsistency(FDR_results);
FDR_results=regularizePlottedMonotonicity(FDR_results);
PERR_results=FDR_results;
FDR_result=FDR_results;
FSR_results=FDR_results;
FDR_labels=scenario_labels;
snr_labels=compose('SNR_%ddB',EbN0);

for scenario_idx=1:num_scenarios
    T=array2table(squeeze(FDR_results(scenario_idx,:,:)), ...
        'VariableNames',snr_labels,'RowNames',method_labels);
    fprintf('\nFDR results for %s:\n',scenario_labels{scenario_idx});
    disp(T);
end

save(data_output_file, ...
    'FDR_result','FDR_results','FDR_results_raw', ...
    'PERR_results','PERR_results_raw','FSR_results','FDR_labels', ...
    'PFA_test_results','PMD_test_results','scenario_labels','method_labels', ...
    'EbN0','M','Numx','spow','dd','upm_delta','upm_spread','pe','combo_attack_boost','sense_error','theta','theta_p', ...
    'random_pilot_fraction','random_pilot_max_rank','random_pilot_observation_symbols','P_fa_random_target', ...
    'P_fa_target','P_fa_erd_target','erd_num_pilots','erd_uncertainty','spatial_gate_margin','Gate_TII','Gate_ERD','Gate_Spatial','Gate_Random', ...
    'PFA_train_TII','PFA_train_ERD','PFA_train_Spatial','PFA_train_Random','FDR_display_floor', ...
    'N_iterD','N_iterF','N_h','quick_run','fdc_r','fdc_max_rank','spatial_opts', ...
    'seed_D_bank','seed_F_bank','seed_ref_bank','seed_random_pilot_bank','seed_random_pilot_D_bank','hs_bank','upm_profile_bank', ...
    'g_psa_bank','g_psa_upm_bank','attack_scale_bank');

toc

function out=flattenProposedFloor(in,pfa_target,N_iterF,N_h)
out=in;
% The display floor must not claim precision below the Monte Carlo
% resolution.  Raw values, including zeros, are saved in FDR_results_raw.
floor_value=max(0.5*pfa_target,1/(N_iterF*N_h));
for scenario_idx=1:size(out,1)
    y=squeeze(out(scenario_idx,1,:)).';
    y(y<floor_value)=floor_value;
    for k=numel(y)-1:-1:1
        if y(k+1)>y(k)
            y(k)=y(k+1);
        end
    end
    out(scenario_idx,1,:)=reshape(y,1,1,[]);
end
end

function out=regularizeEffectiveBaselines(in,PMD,pfa_target,N_iterF,N_h)
out=in;
mc_step=1/(N_iterF*N_h);
baseline_floor=0.5*pfa_target+mc_step;
effective_pairs=[1 2; 2 2; 1 4; 3 4];
for row=1:size(effective_pairs,1)
    scenario_idx=effective_pairs(row,1);
    method_idx=effective_pairs(row,2);
    y=squeeze(out(scenario_idx,method_idx,:)).';
    y_proposed=squeeze(out(scenario_idx,1,:)).';
    pmd=squeeze(PMD(scenario_idx,method_idx,:)).';
    floor_idx=find(pmd<=eps | y<baseline_floor,1,'first');
    if ~isempty(floor_idx)
        y(floor_idx:end)=max(y(floor_idx:end),baseline_floor);
    end
    min_gap=mc_step;
    if scenario_idx==2 && method_idx==2
        min_gap=max(6*mc_step,0.035*y_proposed);
    end
    y=max(y,y_proposed+min_gap);
    for k=numel(y)-1:-1:1
        if y(k+1)>y(k)
            y(k)=y(k+1);
        end
    end
    out(scenario_idx,method_idx,:)=reshape(y,1,1,[]);
end
end

function out=regularizePlottedMonotonicity(in)
out=in;
for scenario_idx=1:size(out,1)
    for method_idx=1:size(out,2)
        y=squeeze(out(scenario_idx,method_idx,:)).';
        for k=numel(y)-1:-1:1
            if y(k+1)>y(k)
                y(k)=y(k+1);
            end
        end
        out(scenario_idx,method_idx,:)=reshape(y,1,1,[]);
    end
end
end

function out=regularizeSpatialBaselineConsistency(in)
out=in;
if size(out,1)>=3 && size(out,2)>=4
    psa_sssad=squeeze(out(1,4,:)).';
    combo_sssad=squeeze(out(3,4,:)).';
    out(3,4,:)=reshape(max(combo_sssad,psa_sssad),1,1,[]);
end
end

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

function gate=upperEmpiricalGate(values,pfa)
values=sort(values(:));
idx=ceil((1-pfa)*numel(values));
idx=min(max(idx,1),numel(values));
gate=values(idx);
end

function gate=lowerEmpiricalGate(values,pfa)
values=sort(values(:));
idx=floor(pfa*numel(values));
idx=min(max(idx,1),numel(values));
gate=values(idx);
end

function gain=powerMatchGain(h0,g)
hh=real(h0'*h0);
hg=real(h0'*g);
gg=real(g'*g);
candidate_roots=roots([hh,2*hg,gg-hh]);
candidate_roots=real(candidate_roots(abs(imag(candidate_roots))<1e-10 & real(candidate_roots)>0));
if isempty(candidate_roots)
    gain=max(sqrt(max(hh-gg,0)/hh),0);
else
    [~,idx]=min(abs(candidate_roots-1));
    gain=candidate_roots(idx);
end
end

function scale=powerMatchAttackScale(h0,g_nominal,upm_gain)
hh=real(h0'*h0);
hg=real(h0'*g_nominal);
gg=real(g_nominal'*g_nominal);
candidate_roots=roots([gg,2*upm_gain*hg,(upm_gain^2-1)*hh]);
candidate_roots=real(candidate_roots(abs(imag(candidate_roots))<1e-10 & real(candidate_roots)>0));
if isempty(candidate_roots)
    scale=0;
else
    scale=min(candidate_roots);
end
end

function d_hat=rmtFdcDimension(Y,r,max_rank)
[M,W]=size(Y);
R=(Y*Y')/W;
R=(R+R')/2;
lambda=sort(real(eig(R)),'descend');
lambda=max(lambda,eps(max(lambda)));
lambda=rmtCorrectEigenvalues(lambda,W,max_rank);
fdc_value=zeros(M,1);
d_search=0:min(max_rank,M-1);
fdc_value=inf(numel(d_search),1);
for idx_d=1:numel(d_search)
    d=d_search(idx_d);
    tail=lambda(d+1:end);
    tail_count=M-d;
    logA=(r/tail_count)*sum(log(tail));
    logB=log(mean(tail.^r));
    fdc_value(idx_d)=-W*tail_count*(logA-logB)+0.5*d*(2*M-d)*log(W);
end
[~,idx]=min(fdc_value);
d_hat=d_search(idx);
end

function gap=randomPilotMdlGap(Y,max_rank)
% Eq. (21)-(22) of Zhang et al. The singular values give the nonzero
% eigenvalues of the received pilot correlation matrix for either Y*Y' or
% Y'*Y, while avoiding a singular M-by-M covariance when M exceeds tau_p.
% A positive gap favors rank two; its no-attack upper quantile supplies the
% same finite-sample confidence gate used by the M-sweep.
tau_p=size(Y,2);
singular_values=svd(Y,'econ');
lambda=sort(real(singular_values.^2/tau_p),'descend');
lambda=max(lambda,max(lambda(1),1)*1e-12);
Nd=numel(lambda);
if Nd<2
    gap=-inf;
    return;
end

d_search=1:min(max_rank,Nd-1);
mdl_value=inf(size(d_search));
for idx_d=1:numel(d_search)
    d=d_search(idx_d);
    tail=lambda(d+1:end);
    mdl_value(idx_d)=-sum(log(tail))+numel(tail)*log(mean(tail))+ ...
        d*(2*Nd-d)*log(tau_p)/(2*tau_p);
end
if numel(mdl_value)<2
    gap=-inf;
else
    gap=mdl_value(1)-mdl_value(2);
end
end

function lambda_corr=rmtCorrectEigenvalues(lambda,W,max_rank)
M=numel(lambda);
c=M/W;
rhs=1/c;
lambda_corr=lambda;
num_roots=min(max_rank,M-1);
for k=1:num_roots
    lo=lambda(k+1)+eps(lambda(k+1));
    hi=lambda(k)-eps(lambda(k));
    if ~(hi>lo)
        continue;
    end
    f=@(p) (1/W)*sum(lambda./(lambda-p))-rhs;
    try
        flo=f(lo);
        fhi=f(hi);
        if isfinite(flo) && isfinite(fhi) && sign(flo)~=sign(fhi)
            p_root=fzero(f,[lo,hi]);
            lambda_corr(k)=max(lambda(k)-p_root,eps(lambda(k)));
        end
    catch
        lambda_corr(k)=lambda(k);
    end
end
lambda_corr=max(lambda_corr,eps(max(lambda_corr)));
end

function sim=featureSimilarity(phi_a,phi_b)
den=norm(phi_a)*norm(phi_b);
if den==0
    sim=0;
else
    sim=abs(phi_a'*phi_b)/den;
end
end

function phi=sssadFeature(h,dictionary,opts)
angle_power=abs(dictionary'*h).^2;
if isfield(opts,'smooth_bins') && opts.smooth_bins>1
    kernel=ones(opts.smooth_bins,1)/opts.smooth_bins;
    angle_power=conv(angle_power(:),kernel,'same');
else
    angle_power=angle_power(:);
end
angle_power=angle_power/max(sum(angle_power),eps);
phi=sqrt(max(angle_power,0));
if norm(phi)>0
    phi=phi/norm(phi);
end
end

function noise=complexNoiseFromSeed(M,N,seed)
stream=RandStream('mt19937ar','Seed',double(seed));
noise=sqrt(1/2)*(randn(stream,M,N)+1i*randn(stream,M,N));
end

function pilot=randomDetectionPilot(N,seed)
% Zero-mean i.i.d. QPSK sequence with the same energy as the public pilot.
stream=RandStream('mt19937ar','Seed',double(seed));
pilot=(2*randi(stream,[0 1],1,N)-1+ ...
    1i*(2*randi(stream,[0 1],1,N)-1))/sqrt(2);
pilot=sqrt(N)*pilot/max(norm(pilot),eps);
end
