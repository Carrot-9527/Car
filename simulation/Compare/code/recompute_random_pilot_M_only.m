% Recompute only the Random-pilot/MDL baseline for the antenna sweep.
% Methods 1--4 are loaded from the existing MAT file and are never
% recalculated or reassigned.
clearvars; clc;
tic;

script_dir=fileparts(mfilename('fullpath'));
release_root=fileparts(script_dir);
data_file=fullfile(release_root,'data','FDR_TII_mainR1_M_compare.mat');
base=load(data_file);

%% Random-pilot configuration shared with the SNR sweep
random_pilot_fraction=0.50;
random_pilot_max_rank=2;
P_fa_random_target=0.01;
random_pilot_observation_symbols=24;
random_pilot_test_trials=1000;

M_list=base.M_list;
snr_db=base.snr_db;
Numx=base.Numx;
assert(random_pilot_observation_symbols<=Numx, ...
    'Random-pilot observation window exceeds the simulated pilot length.');
obs_idx=1:random_pilot_observation_symbols;
spow=base.spow;
dd=base.dd;
pe=base.pe;
combo_attack_boost=base.combo_attack_boost;
upm_gain_target=base.upm_gain_target;
sense_error=base.sense_error;
theta=base.theta;
theta_p=base.theta_p;
N_iterD=base.N_iterD;
N_iterF_base=base.N_iterF;
N_h=base.N_h;
num_scenarios=numel(base.scenario_labels);
num_m=numel(M_list);
Krise=16;
Krisep=16;
assert(mod(random_pilot_test_trials,2)==0, ...
    'Random-pilot test-trial count must be even for a balanced test set.');

%% Replay the original deterministic objects and channel RNG state
rng(20260707);
x_bits=randi([0,1],1,Numx);
x=2*x_bits-1;
x_obs=x(obs_idx);
attack_ind_replay=repmat([0 1],1,N_iterF_base/2);
attack_ind_replay=attack_ind_replay(randperm(N_iterF_base)); %#ok<NASGU>
seed_D_bank=randi(2^31-1,N_iterD,N_h);
seed_F_bank_replay=randi(2^31-1,N_iterF_base,N_h);
seed_ref_bank=randi(2^31-1,1,N_h);
assert(isequal(seed_D_bank,base.seed_D_bank),'No-attack seed replay mismatch.');
assert(isequal(seed_F_bank_replay,base.seed_F_bank),'Test seed replay mismatch.');
assert(isequal(seed_ref_bank,base.seed_ref_bank),'Reference seed replay mismatch.');

random_pilot_seed_stream=RandStream('mt19937ar','Seed',20260824);
seed_random_pilot_bank_replay=randi( ...
    random_pilot_seed_stream,2^31-1,N_iterF_base,N_h);
seed_random_pilot_D_bank=randi( ...
    random_pilot_seed_stream,2^31-1,N_iterD,N_h);
assert(isequal(seed_random_pilot_bank_replay,base.seed_random_pilot_bank), ...
    'Random-pilot test seed replay mismatch.');
assert(isequal(seed_random_pilot_D_bank,base.seed_random_pilot_D_bank), ...
    'Random-pilot training seed replay mismatch.');

% Higher-resolution Random-pilot evaluation uses isolated deterministic
% streams and therefore cannot perturb any legacy method.
attack_stream=RandStream('mt19937ar','Seed',20260827);
random_pilot_attack_ind=repmat([0 1],1,random_pilot_test_trials/2);
random_pilot_attack_ind=random_pilot_attack_ind( ...
    randperm(attack_stream,random_pilot_test_trials));
noise_eval_stream=RandStream('mt19937ar','Seed',20260826);
seed_random_noise_eval_bank=randi(noise_eval_stream,2^31-1, ...
    random_pilot_test_trials,N_h);
pilot_eval_stream=RandStream('mt19937ar','Seed',20260825);
seed_random_pilot_eval_bank=randi(pilot_eval_stream,2^31-1, ...
    random_pilot_test_trials,N_h);

noise_mag=sqrt(spow*10.^(-snr_db/10));
Gate_Random=zeros(N_h,num_m);
PFA_train_Random=zeros(N_h,num_m);
random_fdr_raw=zeros(num_scenarios,num_m);
random_pfa=zeros(num_scenarios,num_m);
random_pmd=zeros(num_scenarios,num_m);
no_attack_ind=(random_pilot_attack_ind==0);
attack_only_ind=(random_pilot_attack_ind==1);

for im=1:num_m
    M=M_list(im);
    array=linspace(0,M-1,M);
    h0_bank=zeros(M,N_h);
    g_psa_bank=zeros(M,N_h);
    g_psa_upm_bank=zeros(M,N_h);
    hs_bank=zeros(M,N_h);
    upm_profile_bank=zeros(M,N_h);
    attack_scale_bank=zeros(1,N_h);

    for h=1:N_h
        h_LoS=exp(1i*2*pi*dd*sin(theta).'*array).';
        h_NLoS=sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
        h0_bank(:,h)=sqrt(Krise/(1+Krise))*h_LoS+ ...
            sqrt(1/(1+Krise))*h_NLoS;
        hs_bank(:,h)=h0_bank(:,h)+sense_error*sqrt(0.5)* ...
            (randn(M,1)+1i*randn(M,1));
        g_LoS=exp(1i*2*pi*dd*sin(theta_p).'*array).';
        g_NLoS=sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
        g_nominal=pe*(sqrt(Krisep/(1+Krisep))*g_LoS+ ...
            sqrt(1/(1+Krisep))*g_NLoS);
        attack_scale=powerMatchAttackScale( ...
            h0_bank(:,h),g_nominal,upm_gain_target);
        if attack_scale<=0
            attack_scale=combo_attack_boost;
        end
        g_psa_bank(:,h)=g_nominal;
        g_psa_upm_bank(:,h)=attack_scale*g_nominal;
        upm_profile_bank(:,h)=upm_gain_target*ones(M,1);
        attack_scale_bank(h)=attack_scale;
    end

    if im==num_m
        assert(max(abs(g_psa_bank(:)-base.g_psa_bank(:)))<1e-12, ...
            'PSA channel replay mismatch.');
        assert(max(abs(g_psa_upm_bank(:)-base.g_psa_upm_bank(:)))<1e-12, ...
            'Joint channel replay mismatch.');
        assert(max(abs(hs_bank(:)-base.hs_bank(:)))<1e-12, ...
            'Sensing-channel replay mismatch.');
        assert(max(abs(upm_profile_bank(:)-base.upm_profile_bank(:)))<1e-12, ...
            'DCISA profile replay mismatch.');
        assert(max(abs(attack_scale_bank(:)-base.attack_scale_bank(:)))<1e-12, ...
            'Joint attack-scale replay mismatch.');
    end

    fprintf('Random-pilot only: M=%d, Lrp=%d (%d/%d)\n', ...
        M,random_pilot_observation_symbols,im,num_m);
    random_fdr_h=zeros(num_scenarios,N_h);
    random_pfa_h=zeros(num_scenarios,N_h);
    random_pmd_h=zeros(num_scenarios,N_h);

    for h=1:N_h
        h0=h0_bank(:,h);
        g_psa=g_psa_bank(:,h);
        g_psa_upm=g_psa_upm_bank(:,h);
        upm_gain=upm_profile_bank(:,h);

        random_mdl_gap_na=zeros(1,N_iterD);
        for thd=1:N_iterD
            noise_d_full=noise_mag*complexNoiseFromSeed( ...
                M,Numx,seed_D_bank(thd,h));
            x_random_d_full=randomDetectionPilot( ...
                Numx,seed_random_pilot_D_bank(thd,h));
            x_random_d=x_random_d_full(obs_idx);
            x_legitimate_d=sqrt(1-random_pilot_fraction)*x_obs+ ...
                sqrt(random_pilot_fraction)*x_random_d;
            random_mdl_gap_na(thd)=randomPilotMdlGap( ...
                h0*x_legitimate_d+noise_d_full(:,obs_idx), ...
                random_pilot_max_rank);
        end
        Gate_random=upperEmpiricalGate( ...
            random_mdl_gap_na,P_fa_random_target);
        Gate_Random(h,im)=Gate_random;
        PFA_train_Random(h,im)=mean(random_mdl_gap_na>Gate_random);

        detected=false(num_scenarios,random_pilot_test_trials);
        for thf=1:random_pilot_test_trials
            noise_f_full=noise_mag*complexNoiseFromSeed( ...
                M,Numx,seed_random_noise_eval_bank(thf,h));
            noise_f=noise_f_full(:,obs_idx);
            x_random_full=randomDetectionPilot( ...
                Numx,seed_random_pilot_eval_bank(thf,h));
            x_random=x_random_full(obs_idx);
            x_legitimate=sqrt(1-random_pilot_fraction)*x_obs+ ...
                sqrt(random_pilot_fraction)*x_random;

            for scenario_idx=1:num_scenarios
                if random_pilot_attack_ind(thf)==0
                    Y_random_pilot=h0*x_legitimate+noise_f;
                else
                    switch scenario_idx
                        case 1
                            Y_random_pilot=h0*x_legitimate+g_psa*x_obs+noise_f;
                        case 2
                            Y_random_pilot=(upm_gain.*h0)*x_legitimate+noise_f;
                        case 3
                            Y_random_pilot=(upm_gain.*h0)*x_legitimate+ ...
                                g_psa_upm*x_obs+noise_f;
                    end
                end
                detected(scenario_idx,thf)=randomPilotMdlGap( ...
                    Y_random_pilot,random_pilot_max_rank)>Gate_random;
            end
        end

        for scenario_idx=1:num_scenarios
            d=detected(scenario_idx,:);
            random_pfa_h(scenario_idx,h)=mean(d(no_attack_ind));
            random_pmd_h(scenario_idx,h)=mean(~d(attack_only_ind));
            random_fdr_h(scenario_idx,h)=mean( ...
                d~=random_pilot_attack_ind);
        end
    end

    random_fdr_raw(:,im)=mean(random_fdr_h,2);
    random_pfa(:,im)=mean(random_pfa_h,2);
    random_pmd(:,im)=mean(random_pmd_h,2);
end

%% Replace only method 5; preserve methods 1--4 bit-for-bit
random_display=max(random_fdr_raw,base.FDR_display_floor);
for scenario_idx=1:num_scenarios
    for k=num_m-1:-1:1
        if random_display(scenario_idx,k+1)>random_display(scenario_idx,k)
            random_display(scenario_idx,k)=random_display(scenario_idx,k+1);
        end
    end
end

slot_raw=reshape(random_fdr_raw,num_scenarios,1,num_m);
slot_display=reshape(random_display,num_scenarios,1,num_m);
slot_pfa=reshape(random_pfa,num_scenarios,1,num_m);
slot_pmd=reshape(random_pmd,num_scenarios,1,num_m);
base.FDR_results_raw(:,5,:)=slot_raw;
base.PERR_results_raw(:,5,:)=slot_raw;
base.FDR_results(:,5,:)=slot_display;
base.PERR_results(:,5,:)=slot_display;
base.FDR_result(:,5,:)=slot_display;
base.FSR_results(:,5,:)=slot_display;
base.PFA_test_results(:,5,:)=slot_pfa;
base.PMD_test_results(:,5,:)=slot_pmd;

base.random_pilot_fraction=random_pilot_fraction;
base.random_pilot_max_rank=random_pilot_max_rank;
base.P_fa_random_target=P_fa_random_target;
base.random_pilot_observation_symbols=random_pilot_observation_symbols;
base.random_pilot_test_trials=random_pilot_test_trials;
base.Gate_Random=Gate_Random;
base.PFA_train_Random=PFA_train_Random;
base.seed_random_pilot_D_bank=seed_random_pilot_D_bank;
base.seed_random_noise_eval_bank=seed_random_noise_eval_bank;
base.seed_random_pilot_eval_bank=seed_random_pilot_eval_bank;
base.random_pilot_attack_ind=random_pilot_attack_ind;
save(data_file,'-struct','base','-v7.3');

fprintf('\nUpdated Random-pilot M-sweep FDR rows:\n');
fprintf('PSA: '); fprintf('%.8g ',random_fdr_raw(1,:)); fprintf('\n');
fprintf('DCISA: '); fprintf('%.8g ',random_fdr_raw(2,:)); fprintf('\n');
fprintf('PSA+DCISA: '); fprintf('%.8g ',random_fdr_raw(3,:)); fprintf('\n');
toc;

function gate=upperEmpiricalGate(values,pfa)
values=sort(values(:));
idx=ceil((1-pfa)*numel(values));
idx=min(max(idx,1),numel(values));
gate=values(idx);
end

function scale=powerMatchAttackScale(h0,g_nominal,upm_gain)
hh=real(h0'*h0);
hg=real(h0'*g_nominal);
gg=real(g_nominal'*g_nominal);
candidate_roots=roots([gg,2*upm_gain*hg,(upm_gain^2-1)*hh]);
candidate_roots=real(candidate_roots( ...
    abs(imag(candidate_roots))<1e-10 & real(candidate_roots)>0));
if isempty(candidate_roots)
    scale=0;
else
    scale=min(candidate_roots);
end
end

function gap=randomPilotMdlGap(Y,max_rank)
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

function noise=complexNoiseFromSeed(M,N,seed)
stream=RandStream('mt19937ar','Seed',double(seed));
noise=sqrt(1/2)*(randn(stream,M,N)+1i*randn(stream,M,N));
end

function pilot=randomDetectionPilot(N,seed)
stream=RandStream('mt19937ar','Seed',double(seed));
pilot=(2*randi(stream,[0 1],1,N)-1+ ...
    1i*(2*randi(stream,[0 1],1,N)-1))/sqrt(2);
pilot=sqrt(N)*pilot/max(norm(pilot),eps);
end
