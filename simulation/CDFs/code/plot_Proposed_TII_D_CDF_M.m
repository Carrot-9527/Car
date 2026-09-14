% Plot CDFs of the Proposed TII detector statistic D for different M.
% This script only keeps the Proposed TII statistic from the comparison
% simulations. It does not run ERD, RMT-FDC, or SSSAD.
%
% For this CDF view, PSA+DCISA is plotted as the direct superposition of the
% DCISA distortion and the boosted PSA component, so its distortion is the
% largest among the attack cases.
clearvars; clc; close all;
tic
rng(20260709);

%% Parameters
M_list=[16 128 256];
snr_db=10;
Numx=128;
x_bits=randi([0,1],1,Numx);
x=2*x_bits-1;
spow=1;
dd=1/2;
upm_delta=0.15;
upm_gain_target=sqrt(1+upm_delta);
pe=0.16;
combo_attack_boost=1.35;
sense_error=0.07;

domi=5;
delta=0.1;
t=-domi:delta:domi-delta;
T=numel(t);
fixed_gate=1.1312e-4;     % common threshold used in every CDF plot

%% Monte Carlo parameters
N_iterCDF=1000;
N_h=8;
noise_mag=sqrt(spow*10.^(-snr_db/10));

case_labels={'No attack','PSA','DCISA','PSA+DCISA'};
num_cases=numel(case_labels);
D_samples=cell(numel(M_list),1);
Gate_D=zeros(1,numel(M_list));
Gate_D_ref=fixed_gate;
nearest_attack_label=cell(1,numel(M_list));
nearest_confusion=zeros(1,numel(M_list));
nearest_pfa=zeros(1,numel(M_list));
nearest_pmd=zeros(1,numel(M_list));
PFA_fixed_DCISA=zeros(1,numel(M_list));
PMD_fixed_DCISA=zeros(1,numel(M_list));
FDR_fixed_DCISA=zeros(1,numel(M_list));

%% Static channel parameters
theta=pi/3;
theta_p=theta+pi/300;
Krise=16;
Krisep=16;
pilot_energy=x*x';

colors=[0.0000 0.4470 0.7410;
        0.8500 0.3250 0.0980;
        0.9290 0.6940 0.1250;
        0.4940 0.1840 0.5560];
line_styles={'-','-','-','-'};

for im=1:numel(M_list)
    M=M_list(im);
    array=linspace(0,M-1,M);
    total_samples=N_iterCDF*N_h;
    D_this_M=zeros(num_cases,total_samples);
    sample_idx=0;

    fprintf('Collecting Proposed TII D samples: M = %d, SNR = %d dB (%d/%d)\n', ...
        M,snr_db,im,numel(M_list));

    for h=1:N_h
        %% Channel generation
        h_LoS=exp(1i*2*pi*dd*sin(theta).'*array).';
        h_NLoS=sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
        h0=sqrt(Krise/(1+Krise))*h_LoS+sqrt(1/(1+Krise))*h_NLoS;
        h_sensed=h0+sense_error*sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
        hr=h_LoS;

        g_LoS=exp(1i*2*pi*dd*sin(theta_p).'*array).';
        g_NLoS=sqrt(0.5)*(randn(M,1)+1i*randn(M,1));
        g_psa=pe*(sqrt(Krisep/(1+Krisep))*g_LoS+sqrt(1/(1+Krisep))*g_NLoS);

        upm_gain=upm_gain_target;
        g_psa_upm=combo_attack_boost*g_psa;

        h_cases=zeros(M,num_cases);
        h_cases(:,1)=h0;
        h_cases(:,2)=h0+g_psa;
        h_cases(:,3)=upm_gain.*h0;
        h_cases(:,4)=upm_gain.*h0+g_psa_upm;

        %% Reference conditional distributions for Proposed TII
        ind_hr11=(find(real(hr)>=0 & imag(hr)>=0)');
        ind_hr01=(find(real(hr)<0 & imag(hr)>=0)');
        ind_hr10=(find(real(hr)>=0 & imag(hr)<0)');
        ind_hr00=(find(real(hr)<0 & imag(hr)<0)');

        F0YX11=PDF2(h_sensed(ind_hr11),domi,delta,T);
        F0YX01=PDF2(h_sensed(ind_hr01),domi,delta,T);
        F0YX10=PDF2(h_sensed(ind_hr10),domi,delta,T);
        F0YX00=PDF2(h_sensed(ind_hr00),domi,delta,T);

        %% Detector statistic samples
        for iter=1:N_iterCDF
            sample_idx=sample_idx+1;
            noise=noise_mag*sqrt(1/2)*(randn(M,Numx)+1i*randn(M,Numx));

            for case_idx=1:num_cases
                Y_x=h_cases(:,case_idx)*x+noise;
                h_ls=(Y_x*x')/pilot_energy;
                D_this_M(case_idx,sample_idx)=tiiStatistic(h_ls, ...
                    ind_hr11,ind_hr01,ind_hr10,ind_hr00, ...
                    F0YX01,F0YX10,F0YX00,F0YX11,domi,delta,T);
            end
        end
    end

    D_samples{im}=D_this_M;
    [Gate_D(im),nearest_idx,nearest_confusion(im),nearest_pfa(im),nearest_pmd(im)]= ...
        closestAttackGate(D_this_M);
    nearest_attack_label{im}=case_labels{nearest_idx};
    upm_idx=3;
    PFA_fixed_DCISA(im)=mean(D_this_M(1,:)>fixed_gate);
    PMD_fixed_DCISA(im)=mean(D_this_M(upm_idx,:)<=fixed_gate);
    FDR_fixed_DCISA(im)=0.5*(PFA_fixed_DCISA(im)+PMD_fixed_DCISA(im));

    %% Plot empirical CDF for this M
    figure('Color','w','Name',sprintf('Proposed TII D CDF M=%d',M), ...
        'Position',[100 100 460 330]);
    hold on;
    x_lower=quantile(D_this_M(:),0.002);
    x_upper=max(D_this_M(:));
    x_span=x_upper-x_lower;
    if x_span>0
        x_min=max(0,x_lower-0.06*x_span);
        x_max=x_upper+0.06*x_span;
    else
        x_min=0;
        x_max=max(D_this_M(:));
    end

    [x_no,y_no]=empiricalCdf(D_this_M(1,:));
    [x_upm,y_upm]=empiricalCdf(D_this_M(upm_idx,:));
    shadeFixedGateErrors(x_no,y_no,x_upm,y_upm,fixed_gate,x_min,x_max);

    for case_idx=1:num_cases
        [x_cdf,y_cdf]=empiricalCdf(D_this_M(case_idx,:));
        plot(x_cdf,y_cdf,'LineWidth',1.55,'Color',colors(case_idx,:), ...
            'LineStyle',line_styles{case_idx},'DisplayName',case_labels{case_idx});
    end
    plot([fixed_gate fixed_gate],[0 1],'k--','LineWidth',1.15,'HandleVisibility','off');
    grid on;
    ax=gca;
    ax.GridLineStyle='-';
    ax.GridAlpha=0.24;
    ax.LineWidth=1.0;
    ax.TickDir='in';
    ax.Layer='top';
    box on;
    xlabel('Detector statistic D','FontSize',11);
    ylabel('CDF','FontSize',11);
    title(sprintf('M = %d, SNR = %d dB',M,snr_db), ...
        'FontSize',11,'FontWeight','normal','Interpreter','none');
    lgd=legend('Location','northwest','FontSize',8,'Box','on');
    lgd.Color='w';
    lgd.EdgeColor='k';
    lgd.LineWidth=0.8;
    ylim([0 1]);
    yticks(0:0.25:1);

    if x_span>0
        xlim([x_min,x_max]);
    end
    gate_text=sprintf('%.2g',fixed_gate);
    text(fixed_gate+0.015*x_span,0.965,gate_text,'FontName','Times New Roman', ...
        'FontSize',8.5,'HorizontalAlignment','left','VerticalAlignment','top', ...
        'BackgroundColor','w','Margin',1.5,'Interpreter','none');
    sep_text=sprintf('DCISA\nFDR: %.2f%%',100*FDR_fixed_DCISA(im));
    text(0.98,0.08,sep_text,'Units','normalized','FontName','Times New Roman', ...
        'FontSize',8.5,'HorizontalAlignment','right','VerticalAlignment','bottom', ...
        'BackgroundColor','w','EdgeColor',[0.25 0.25 0.25],'Margin',3, ...
        'Interpreter','tex');
    hold off;
    set(gca,'FontName','Times New Roman','FontSize',10);

    savefig(gcf,sprintf('CDF_Proposed_TII_D_M%d.fig',M));
    exportgraphics(gcf,sprintf('CDF_Proposed_TII_D_M%d.png',M),'Resolution',300);
    exportgraphics(gcf,sprintf('CDF_Proposed_TII_D_M%d.pdf',M),'ContentType','vector');
end

save('Proposed_TII_D_CDF_M.mat','D_samples','M_list','snr_db','Numx','spow','dd', ...
    'upm_delta','upm_gain_target','pe','combo_attack_boost','sense_error', ...
    'theta','theta_p','N_iterCDF','N_h','case_labels','Gate_D','Gate_D_ref','fixed_gate', ...
    'nearest_attack_label','nearest_confusion','nearest_pfa','nearest_pmd', ...
    'PFA_fixed_DCISA','PMD_fixed_DCISA','FDR_fixed_DCISA');

toc

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

function [x_cdf,y_cdf]=empiricalCdf(values)
values=sort(values(:));
n=numel(values);
x_cdf=[values(1); values];
y_cdf=[0; (1:n)'/n];
end

function shadeFixedGateErrors(x_no,y_no,x_upm,y_upm,gate,x_min,x_max)
shade_color=[0.62 0.62 0.62];
shade_alpha=0.38;

% Missed detection for DCISA: DCISA samples below the fixed threshold.
pmd=interpCdfAtGate(x_upm,y_upm,gate);
if pmd>0 && gate>x_min
    idx=x_upm<=gate;
    x_curve=[x_min; x_upm(idx); gate];
    y_curve=[0; y_upm(idx); pmd];
    fill([x_curve; flipud(x_curve)], ...
        [zeros(size(x_curve)); flipud(y_curve)], ...
        shade_color,'FaceAlpha',shade_alpha,'EdgeColor','none', ...
        'HandleVisibility','off');
end

% False alarm for no attack: no-attack samples above the fixed threshold.
f0_gate=interpCdfAtGate(x_no,y_no,gate);
if f0_gate<1 && gate<x_max
    idx=x_no>=gate & x_no<=x_max;
    x_curve=[gate; x_no(idx); x_max];
    y_end=interpCdfAtGate(x_no,y_no,x_max);
    y_curve=[f0_gate; y_no(idx); y_end];
    fill([x_curve; flipud(x_curve)], ...
        [y_curve; ones(size(x_curve))], ...
        shade_color,'FaceAlpha',shade_alpha,'EdgeColor','none', ...
        'HandleVisibility','off');
end
end

function y=interpCdfAtGate(x_cdf,y_cdf,gate)
idx=find(x_cdf<=gate,1,'last');
if isempty(idx)
    y=0;
else
    y=y_cdf(idx);
end
end

function [gate,nearest_idx,nearest_confusion,pfa_at_gate,pmd_at_gate]=closestAttackGate(D_values)
no_vals=D_values(1,:);
num_cases=size(D_values,1);
best_gate=zeros(1,num_cases);
best_conf=zeros(1,num_cases);
best_pfa=zeros(1,num_cases);
best_pmd=zeros(1,num_cases);

for case_idx=2:num_cases
    [best_gate(case_idx),best_conf(case_idx),best_pfa(case_idx),best_pmd(case_idx)]= ...
        bestBalancedGate(no_vals,D_values(case_idx,:));
end

[nearest_confusion,nearest_idx]=max(best_conf(2:end));
nearest_idx=nearest_idx+1;
gate=best_gate(nearest_idx);
pfa_at_gate=best_pfa(nearest_idx);
pmd_at_gate=best_pmd(nearest_idx);
end

function [gate,confusion,pfa,pmd]=bestBalancedGate(no_vals,attack_vals)
no_vals=sort(no_vals(:));
attack_vals=sort(attack_vals(:));
all_vals=unique([no_vals; attack_vals]);

if numel(all_vals)==1
    candidates=all_vals;
else
    candidates=(all_vals(1:end-1)+all_vals(2:end))/2;
end

n0=numel(no_vals);
n1=numel(attack_vals);
no_le=0;
attack_le=0;
confusion=inf;
gate=candidates(1);
pfa=1;
pmd=0;

for idx=1:numel(candidates)
    cand=candidates(idx);
    while no_le<n0 && no_vals(no_le+1)<=cand
        no_le=no_le+1;
    end
    while attack_le<n1 && attack_vals(attack_le+1)<=cand
        attack_le=attack_le+1;
    end

    pfa_candidate=(n0-no_le)/n0;
    pmd_candidate=attack_le/n1;
    confusion_candidate=0.5*(pfa_candidate+pmd_candidate);

    if confusion_candidate<confusion
        confusion=confusion_candidate;
        gate=cand;
        pfa=pfa_candidate;
        pmd=pmd_candidate;
    end
end
end
