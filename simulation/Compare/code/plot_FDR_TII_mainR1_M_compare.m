% Plot M-sweep FDR curves for PSA, DCISA, and PSA+DCISA.
close all force;
clearvars; clc;

script_dir=fileparts(mfilename('fullpath'));
release_root=fileparts(script_dir);
data_file=fullfile(release_root,'data','FDR_TII_mainR1_M_compare.mat');
load(data_file,'M_list','snr_db','FDR_results','scenario_labels','method_labels');
method_labels(strcmp(method_labels,'Proposed TII'))={'Proposed Method'};
method_labels(strcmp(method_labels,'Random-pilot'))={'Random-pilot/MDL'};
results_dir=fullfile(release_root,'results');
pdf_output_dir=fullfile(results_dir,'pdf');
png_output_dir=fullfile(results_dir,'png');
fig_output_dir=fullfile(results_dir,'fig');
output_dirs={pdf_output_dir,png_output_dir,fig_output_dir};
for output_idx=1:numel(output_dirs)
    if ~isfolder(output_dirs{output_idx})
        mkdir(output_dirs{output_idx});
    end
end
figure_size_pixels=[650 390];
figure_size_inches=[6.5 3.9];

colors=[0.0000 0.4470 0.7410;
        0.8500 0.3250 0.0980;
        0.9290 0.6940 0.1250;
        0.4940 0.1840 0.5560;
        0.4660 0.6740 0.1880];
marker_styles={'o','s','^','d','v'};
line_styles={'-','--',':','-.','-'};

for scenario_idx=1:numel(scenario_labels)
    scenario_data=squeeze(FDR_results(scenario_idx,:,:));
    positive_values=scenario_data(scenario_data>0);
    if isempty(positive_values)
        y_min=1e-3;
        y_max=1;
    else
        y_min=min(positive_values(:));
        y_max=max(positive_values(:));
    end
    if y_max<=y_min
        y_min=max(0.85*y_min,1e-4);
        y_max=min(1.20*y_max,1);
    else
        log_span=log10(y_max)-log10(y_min);
        log_pad=max(0.06*log_span,0.025);
        y_min=max(10^(log10(y_min)-log_pad),1e-4);
        y_max=min(10^(log10(y_max)+log_pad),1);
    end

    figure('Color','w','Name',scenario_labels{scenario_idx}, ...
        'Position',[100 100 figure_size_pixels]);
    hold on;
    plot_order=[5 1 2 3 4];
    plot_handles=gobjects(numel(method_labels),1);
    for draw_idx=1:numel(plot_order)
        method_idx=plot_order(draw_idx);
        y=squeeze(FDR_results(scenario_idx,method_idx,:));
        y=max(y,y_min);
        plot_handles(method_idx)=semilogy(M_list,y,'LineWidth',1.8, ...
            'LineStyle',line_styles{method_idx}, ...
            'Marker',marker_styles{method_idx},'MarkerSize',5.8, ...
            'MarkerFaceColor','none','MarkerEdgeColor',colors(method_idx,:), ...
            'Color',colors(method_idx,:),'DisplayName',method_labels{method_idx});
    end
    hold off;
    grid on;
    ax=gca;
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
    ax.XMinorTick='off';
    box on;
    xlabel('M','FontSize',13);
    ylabel('FDR','FontSize',13);
    title(scenario_labels{scenario_idx}, ...
        'FontSize',13,'FontWeight','normal','Interpreter','none');
    lgd=legend(plot_handles,method_labels,'Location','southwest','FontSize',9,'Box','on');
    lgd.Color='w';
    lgd.EdgeColor='k';
    lgd.LineWidth=0.8;
    xlim([min(M_list),max(M_list)]);
    xticks(M_list);
    ylim([y_min,y_max]);
    yticks('auto');
    yticklabels('auto');
    set(gca,'FontName','Times New Roman','FontSize',12);

    if strcmp(scenario_labels{scenario_idx},'PSA+DCISA')
        % Enlarge the upper cluster around M=128 without covering the main
        % descending curves. The three 0.5-valued baselines remain exactly
        % coincident; distinct line and marker styles expose that overlap.
        fig=gcf;
        fig.Position=[100 100 figure_size_pixels];
        lgd.Location='southwest';
        lgd.FontSize=8.5;
        drawnow;

        % Stack the inset above the legend in the empty lower-left region.
        % Both panels stay inside the main axes without covering data.
        inset_pos=[0.16 0.43 0.29 0.20];
        zoom_ax=axes('Position',inset_pos); %#ok<LAXES>
        hold(zoom_ax,'on');
        zoom_order=[5 2 3 1 4];
        inset_marker_sizes=[5.8 6.6 5.4 5.8 7.4];
        for draw_idx=1:numel(zoom_order)
            method_idx=zoom_order(draw_idx);
            y=squeeze(FDR_results(scenario_idx,method_idx,:));
            plot(zoom_ax,M_list,y,'LineWidth',1.65, ...
                'LineStyle',line_styles{method_idx}, ...
                'Marker',marker_styles{method_idx}, ...
                'MarkerSize',inset_marker_sizes(method_idx), ...
                'MarkerFaceColor','none','MarkerEdgeColor',colors(method_idx,:), ...
                'Color',colors(method_idx,:));
        end
        hold(zoom_ax,'off');
        grid(zoom_ax,'on');
        box(zoom_ax,'on');
        xlim(zoom_ax,[126 130]);
        xticks(zoom_ax,[126 128 130]);
        ylim(zoom_ax,[0.465 0.505]);
        yticks(zoom_ax,[0.47 0.48 0.49 0.50]);
        zoom_ax.GridAlpha=0.24;
        zoom_ax.LineWidth=0.9;
        zoom_ax.TickDir='in';
        zoom_ax.Layer='top';
        set(zoom_ax,'FontName','Times New Roman','FontSize',9.2);

        % Clear black arrow from the inset edge to the M=128 cluster. The
        % short route starts at the inset's upper-right edge so it does not
        % cross the descending Proposed/Random-pilot curves.
        main_pos=ax.Position;
        target_x=main_pos(1)+main_pos(3)*(128-min(M_list))/(max(M_list)-min(M_list));
        target_y=main_pos(2)+main_pos(4)* ...
            (log10(0.497)-log10(y_min))/(log10(y_max)-log10(y_min));
        start_x=inset_pos(1)+0.96*inset_pos(3);
        start_y=inset_pos(2)+inset_pos(4)-0.01;
        annotation(fig,'arrow',[start_x target_x],[start_y target_y], ...
            'Color','k','LineWidth',1.25,'HeadLength',9,'HeadWidth',9);

    end

    safe_name=regexprep(scenario_labels{scenario_idx},'[^A-Za-z0-9]','_');
    artifact_base=sprintf('FDR_M_%s_final',safe_name);
    fig=gcf;
    set(fig,'Units','pixels','Position',[100 100 figure_size_pixels], ...
        'PaperUnits','inches','PaperSize',figure_size_inches, ...
        'PaperPosition',[0 0 figure_size_inches], ...
        'PaperPositionMode','manual','InvertHardcopy','off');
    drawnow;
    savefig(fig,fullfile(fig_output_dir,sprintf('%s.fig',artifact_base)));
    print(fig,fullfile(png_output_dir,sprintf('%s.png',artifact_base)), ...
        '-dpng','-r300');
    print(fig,fullfile(pdf_output_dir,sprintf('%s.pdf',artifact_base)), ...
        '-dpdf','-painters','-r300');
end
