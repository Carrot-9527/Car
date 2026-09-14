% Plot FAR and MDR from the saved AoA-SNR experiment data.
% The original FDR_results is the total decision error over no-attack and
% attack samples. Since attack_ind was generated with equal priors, the
% miss detection rate is MDR = 2*FDR - FAR.
clear; close all; clc;

quick_test=strcmp(getenv('FDR_AOA_QUICK_TEST'),'1');
file_prefix='FDR_AoA_SNR_M64_128_Var0p1';
if quick_test
    file_prefix='FDR_AoA_SNR_M64_128_Var0p1_quick';
end

data_file=[file_prefix,'.mat'];
S=load(data_file);

if isfield(S,'FDR_results')
    FDR_results=S.FDR_results;
elseif isfield(S,'FDR_result')
    FDR_results=S.FDR_result;
else
    error('FDR_results or FDR_result is required in %s.',data_file);
end

if isfield(S,'FalseAlarm_results')
    FAR_results=S.FalseAlarm_results;
elseif isfield(S,'PFA_test_results')
    FAR_results=S.PFA_test_results;
else
    error('FalseAlarm_results or PFA_test_results is required in %s.',data_file);
end

if ndims(FDR_results)~=4 || size(FDR_results,1)~=3 || size(FDR_results,2)~=3
    error('FDR_results in %s should be 3 x 3 x M-count x SNR-count.',data_file);
end

if ~isequal(size(FAR_results),size(FDR_results))
    error('FAR data in %s should have the same size as FDR_results.',data_file);
end

if isfield(S,'M_list')
    M_list=S.M_list;
else
    M_list=1:size(FDR_results,3);
end

if isfield(S,'EbN0_list')
    EbN0_list=S.EbN0_list;
else
    EbN0_list=1:size(FDR_results,4);
end

attack_prior=0.5;
no_attack_prior=1-attack_prior;
MDR_results=(FDR_results-no_attack_prior*FAR_results)/attack_prior;

raw_MDR_min=min(MDR_results(:));
raw_MDR_max=max(MDR_results(:));
if raw_MDR_min<0 || raw_MDR_max>1
    warning('Raw MDR values are outside [0, 1] before clipping: min = %.4g, max = %.4g.', ...
        raw_MDR_min,raw_MDR_max);
end
MDR_results=max(min(MDR_results,1),0);

case_order=[1,2,3];
case_plot_labels={'Ideal SR','Error-aware SR','Error-mismatched SR'};
attack_plot_labels={'PSA','DCISA','DCISA+PSA'};
attack_file_suffix={'PSA','UPM','UPM_PSA'};

far_spread=max(max(FAR_results,[],1)-min(FAR_results,[],1),[],'all');
if far_spread>1e-12
    warning('FAR differs across attack rows by up to %.4g; plotting their mean as the no-attack FAR.',far_spread);
end
FAR_no_attack=squeeze(mean(FAR_results(:,case_order,:,:),1,'omitnan'));

plot_modeling_curves(EbN0_list,FAR_no_attack,M_list,case_plot_labels, ...
    'FAR','No Attack', ...
    'FAR_AoA_SNR_M64_128_Var0p1_NoAttack_Modeling');

for attack_idx=1:numel(attack_plot_labels)
    MDR_attack=squeeze(MDR_results(attack_idx,case_order,:,:));
    plot_modeling_curves(EbN0_list,MDR_attack,M_list,case_plot_labels, ...
        'MDR',attack_plot_labels{attack_idx}, ...
        sprintf('MDR_AoA_SNR_M64_128_Var0p1_%s',attack_file_suffix{attack_idx}));
end

function plot_modeling_curves(x_data,y_data,M_list,case_labels,y_label,fig_title,file_prefix)
fig=figure('Color','w','Name',fig_title, ...
    'Units','centimeters','Position',[4 4 13 9]);
ax=axes(fig);
if isprop(ax,'Toolbar')
    ax.Toolbar.Visible='off';
end
set(ax,'YScale','log');
hold(ax,'on');

case_colors=[0.0000 0.4470 0.7410;
    0.8500 0.3250 0.0980;
    0.4660 0.6740 0.1880];
line_styles={'-','--'};
marker_styles={'o','s'};
legend_text=cell(1,numel(M_list)*numel(case_labels));
legend_idx=1;

[y_min,y_max]=get_log_ylim(y_data);

for m_idx=1:numel(M_list)
    for case_idx=1:numel(case_labels)
        y=squeeze(y_data(case_idx,m_idx,:)).';
        y_plot=y;
        y_plot(~isfinite(y_plot) | y_plot<=0)=y_min;
        semilogy(ax,x_data,y_plot, ...
            'LineStyle',line_styles{min(m_idx,numel(line_styles))}, ...
            'Marker',marker_styles{min(m_idx,numel(marker_styles))}, ...
            'Color',case_colors(case_idx,:), ...
            'LineWidth',1.7, ...
            'MarkerSize',5, ...
            'MarkerFaceColor','none');
        legend_text{legend_idx}=sprintf('M = %d, %s',M_list(m_idx),case_labels{case_idx});
        legend_idx=legend_idx+1;
    end
end

hold(ax,'off');
grid(ax,'on');
box(ax,'on');
xlim(ax,[min(x_data),max(x_data)]);
ylim(ax,[y_min,y_max]);
set_log_order_ticks(ax,y_min,y_max);
xlabel(ax,'SNR (dB)');
ylabel(ax,y_label);
title(ax,fig_title,'FontName','Times New Roman','FontSize',10,'Interpreter','none');
legend(ax,legend_text,'Location','best','NumColumns',2, ...
    'FontName','Times New Roman','FontSize',8, ...
    'Box','on','Interpreter','none');
set(ax,'FontName','Times New Roman','FontSize',11, ...
    'YScale','log', ...
    'YMinorTick','on', ...
    'YMinorGrid','on', ...
    'MinorGridLineStyle',':', ...
    'GridLineStyle','-', ...
    'LineWidth',1);
set(ax,'LooseInset',max(get(ax,'TightInset'),0.02));

savefig(fig,[file_prefix,'.fig']);
try
    exportgraphics(fig,[file_prefix,'.pdf'],'ContentType','vector');
catch
    set(fig,'PaperPositionMode','auto');
    print(fig,[file_prefix,'.pdf'],'-dpdf','-painters');
end
close(fig);
end

function set_log_order_ticks(ax,y_min,y_max)
tick_exp=floor(log10(y_min)):ceil(log10(y_max));
tick_values=10.^tick_exp;
tick_mask=tick_values>=y_min & tick_values<=y_max;
tick_values=tick_values(tick_mask);
tick_exp=tick_exp(tick_mask);

if isempty(tick_values)
    tick_exp=floor(log10(y_min)):ceil(log10(y_max));
    tick_values=10.^tick_exp;
end

yticks(ax,tick_values);
yticklabels(ax,arrayfun(@(x)sprintf('10^{%d}',x),tick_exp,'UniformOutput',false));
end

function [y_min,y_max]=get_log_ylim(y_data)
y_all=y_data(:);
y_all=y_all(isfinite(y_all) & y_all>0);
if isempty(y_all)
    y_min=1e-4;
    y_max=1;
    return;
end

pad_decade=0.12;
y_min=10^(log10(min(y_all))-pad_decade);
y_max=10^(log10(max(y_all))+pad_decade);
y_min=max(y_min,realmin);
y_max=min(max(y_max,y_min*1.2),1);
end
