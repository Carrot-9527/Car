% FDR and false-alarm simulation under Gaussian AoA sensing error.
% Three cases are compared:
%   1) no AoA error;
%   2) Gaussian AoA error modeled in the reference distribution;
%   3) Gaussian AoA error not modeled in the reference distribution.
clearvars; clc;
tic

%% Parameters
M_list=[64,128];
EbN0_list=0:1:10;
aoa_error_var_deg2=0.1;
case_labels={'No AoA error'; ...
    'AoA error modeled'; ...
    'AoA error unmodeled'};
case_short_names={'NoErr';'KnownVar';'UnknownVar'};

quick_test=strcmp(getenv('FDR_AOA_QUICK_TEST'),'1');
if quick_test
    M_list=64;
    EbN0_list=0:1;
end

Nums=1;
Numx=128;
s=randi([0,1],1,Nums);
signal=pskmod(s,2);
x=randi([0,1],1,Numx);
pilot=pskmod(x,2);
x_norm_inv=1/(x*x');
spow=1;
dd=1/2;
pp=0.25;
pe=0.1;

domi=5;
delta=0.1;
t=-domi:delta:domi-delta;

%% Monte Carlo parameters
N_iterD=1000;          % samples for the no-attack Dn empirical distribution
N_iterF=300;           % samples for average FDR and false alarm rate
N_h=100;               % channel realizations
if quick_test
    N_iterD=20;
    N_iterF=10;
    N_h=3;
end
P_fa_target=0.01;      % NP false alarm constraint
attack_ind=randi([0,1],1,N_iterF);

%% Results
num_attack=3;
num_case=numel(case_labels);
num_M=numel(M_list);
num_SNR=numel(EbN0_list);

FDR_results=zeros(num_attack,num_case,num_M,num_SNR);
Gate_results=zeros(num_case,num_M,num_SNR);
PFA_train_results=zeros(num_case,num_M,num_SNR);
PFA_test_results=zeros(num_attack,num_case,num_M,num_SNR);

%% Static channel parameters
theta=pi/3;
theta_p=3*(pi/4);
Krise=16;
Krisep=16;

for m_idx=1:num_M
    M=M_list(m_idx);
    array=linspace(0,M-1,M);

    for snr_idx=1:num_SNR
        EbN0=EbN0_list(snr_idx);
        noise_mag=sqrt(spow*10.^(-EbN0/10));

        for case_idx=1:num_case
            D_na=zeros(1,N_iterD);
            FDR_h=zeros(num_attack,N_h);
            Gate_h=zeros(1,N_h);
            PFA_train_h=zeros(1,N_h);
            PFA_test_h=zeros(num_attack,N_h);

            fprintf('AoA case = %s, variance = %.4f deg^2, SNR = %d dB, M = %d\n', ...
                case_labels{case_idx},aoa_error_var_deg2,EbN0,M);

            for h=1:N_h
                %% Channel generation
                h_LoS=steering_vector(theta,dd,array);
                h_NLoS=sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
                h0=sqrt(Krise/(1+Krise))*h_LoS+sqrt(1/(1+Krise))*h_NLoS;

                hr_clean=h_LoS;
                aoa_error_rad=0;
                if case_idx~=1
                    aoa_error_rad=(pi/180)*sqrt(aoa_error_var_deg2)*randn;
                end
                hr_est=steering_vector(theta+aoa_error_rad,dd,array);

                % If the AoA variance is modeled, the nominal distribution
                % is trained with the same Gaussian-perturbed reference model.
                % If it is unmodeled, the nominal distribution stays clean.
                if case_idx==2
                    hr_ref=hr_est;
                else
                    hr_ref=hr_clean;
                end

                g_LoS=steering_vector(theta_p,dd,array);
                g_NLoS=sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
                g=pe*(sqrt(Krisep/(1+Krisep))*g_LoS+sqrt(1/(1+Krisep))*g_NLoS);
                Phi=diag(1+pp*rand(1,M));

                hp=h0+g;
                hd=Phi*h0;
                hpd=Phi*h0+g;

                %% Reference conditional distributions
                [ind_ref11,ind_ref01,ind_ref10,ind_ref00]=quadrant_indices(hr_ref);
                [ind_est11,ind_est01,ind_est10,ind_est00]=quadrant_indices(hr_est);

                F0YX11=PDF2(h0(ind_ref11),domi,delta,length(t));
                F0YX01=PDF2(h0(ind_ref01),domi,delta,length(t));
                F0YX10=PDF2(h0(ind_ref10),domi,delta,length(t));
                F0YX00=PDF2(h0(ind_ref00),domi,delta,length(t));

                %% NP threshold: use only the no-attack empirical Dn distribution
                for thd=1:N_iterD
                    noise_p=noise_mag*sqrt(1/2)*(randn(size(h0*pilot))+1i*randn(size(h0*pilot)));
                    Y_x_na=h0*x+noise_p;
                    h_ls_na=(Y_x_na*x')*x_norm_inv;

                    D_na(thd)=GateDnTII2(h_ls_na,ind_ref11,ind_ref01,ind_ref10,ind_ref00, ...
                        F0YX01,F0YX10,F0YX00,F0YX11,domi,delta,length(t));
                end

                D_na_sort=sort(D_na);
                Gate_idx=ceil((1-P_fa_target)*N_iterD);
                Gate_idx=min(max(Gate_idx,1),N_iterD);
                Gate=repmat(D_na_sort(Gate_idx),1,num_attack);
                Gate_h(h)=Gate(1);
                PFA_train_h(h)=mean(D_na>Gate(1));

                %% Attack detection
                Detect=zeros(num_attack,N_iterF);
                for thf=1:N_iterF
                    noise_f=noise_mag*sqrt(1/2)*(randn(size(h0*x))+1i*randn(size(h0*x)));
                    noise_s=noise_mag*sqrt(1/2)*(randn(size(signal))+1i*randn(size(signal))); %#ok<NASGU>

                    Y_x_na_f=h0*x+noise_f;
                    Y_x_p_f=hp*x+noise_f;
                    Y_x_d_f=hd*x+noise_f;
                    Y_x_pd_f=hpd*x+noise_f;

                    if attack_ind(thf)==0
                        h_real0=(Y_x_na_f*x')*x_norm_inv;
                        h_real=repmat(h_real0,1,num_attack);
                    else
                        h_real=zeros(M,num_attack);
                        h_real(:,1)=(Y_x_p_f*x')*x_norm_inv;
                        h_real(:,2)=(Y_x_d_f*x')*x_norm_inv;
                        h_real(:,3)=(Y_x_pd_f*x')*x_norm_inv;
                    end

                    D_real=zeros(1,num_attack);
                    D_real(1,1)=GateDnTII2(h_real(:,1),ind_est11,ind_est01,ind_est10,ind_est00, ...
                        F0YX01,F0YX10,F0YX00,F0YX11,domi,delta,length(t));
                    D_real(1,2)=GateDnTII2(h_real(:,2),ind_est11,ind_est01,ind_est10,ind_est00, ...
                        F0YX01,F0YX10,F0YX00,F0YX11,domi,delta,length(t));
                    D_real(1,3)=GateDnTII2(h_real(:,3),ind_est11,ind_est01,ind_est10,ind_est00, ...
                        F0YX01,F0YX10,F0YX00,F0YX11,domi,delta,length(t));

                    Detect(:,thf)=(D_real>Gate).';
                end

                FDR_h(:,h)=mean(Detect~=repmat(attack_ind,num_attack,1),2);

                no_attack_ind=(attack_ind==0);
                if any(no_attack_ind)
                    PFA_test_h(:,h)=mean(Detect(:,no_attack_ind),2);
                else
                    PFA_test_h(:,h)=NaN;
                end
            end

            FDR_results(:,case_idx,m_idx,snr_idx)=mean(FDR_h,2);
            Gate_results(case_idx,m_idx,snr_idx)=mean(Gate_h,2);
            PFA_train_results(case_idx,m_idx,snr_idx)=mean(PFA_train_h,2);
            PFA_test_results(:,case_idx,m_idx,snr_idx)=mean(PFA_test_h,2,'omitnan');
        end
    end
end

FDR_result=FDR_results;
FDR_no_aoa=squeeze(FDR_results(:,1,:,:));
FDR_aoa_modeled=squeeze(FDR_results(:,2,:,:));
FDR_aoa_unmodeled=squeeze(FDR_results(:,3,:,:));
FalseAlarm_results=PFA_test_results;
FalseAlarm_no_aoa=squeeze(FalseAlarm_results(:,1,:,:));
FalseAlarm_aoa_modeled=squeeze(FalseAlarm_results(:,2,:,:));
FalseAlarm_aoa_unmodeled=squeeze(FalseAlarm_results(:,3,:,:));
FDR_labels={'PSA';'DCISA';'PSA+DCISA'};
snr_col_names=matlab.lang.makeValidName(compose('SNR_%ddB',EbN0_list));

for m_idx=1:num_M
    fprintf('\nFDR average results for M = %d (rows: attacks; columns: SNR):\n',M_list(m_idx));
    for case_idx=1:num_case
        FDR_table=array2table(squeeze(FDR_results(:,case_idx,m_idx,:)), ...
            'VariableNames',snr_col_names,'RowNames',FDR_labels);
        fprintf('Case: %s\n',case_labels{case_idx});
        disp(FDR_table);
    end

    fprintf('\nFalse alarm rate results for M = %d (rows: detectors; columns: SNR):\n',M_list(m_idx));
    for case_idx=1:num_case
        FalseAlarm_table=array2table(squeeze(FalseAlarm_results(:,case_idx,m_idx,:)), ...
            'VariableNames',snr_col_names,'RowNames',FDR_labels);
        fprintf('Case: %s\n',case_labels{case_idx});
        disp(FalseAlarm_table);
    end
end

save_file='FDR_AoA_SNR_M64_128_Var0p1.mat';
if quick_test
    save_file='FDR_AoA_SNR_M64_128_Var0p1_quick.mat';
end

save(save_file,'FDR_result','FDR_results','FDR_no_aoa','FDR_aoa_modeled', ...
    'FDR_aoa_unmodeled','FDR_labels','case_labels','case_short_names', ...
    'FalseAlarm_results','FalseAlarm_no_aoa','FalseAlarm_aoa_modeled', ...
    'FalseAlarm_aoa_unmodeled', ...
    'M_list','EbN0_list','aoa_error_var_deg2','P_fa_target', ...
    'Gate_results','PFA_train_results','PFA_test_results', ...
    'N_iterD','N_iterF','N_h');

fprintf('Saved AoA SNR results to %s\n',save_file);
toc

function a=steering_vector(theta,dd,array)
a=exp(1i*2*pi*dd*sin(theta).'*array).';
end

function [ind11,ind01,ind10,ind00]=quadrant_indices(hr)
ind11=(find(real(hr)>=0 & imag(hr)>=0)');
ind01=(find(real(hr)<0 & imag(hr)>=0)');
ind10=(find(real(hr)>=0 & imag(hr)<0)');
ind00=(find(real(hr)<0 & imag(hr)<0)');
end
