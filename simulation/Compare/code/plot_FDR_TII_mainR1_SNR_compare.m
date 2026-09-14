% Plot SNR-sweep FDR curves for PSA, DCISA, and PSA+DCISA.
close all force;
clearvars; clc;

script_dir=fileparts(mfilename('fullpath'));
release_root=fileparts(script_dir);
data_file=fullfile(release_root,'data','FDR_TII_mainR1_SNR_compare.mat');
load(data_file,'EbN0','FDR_results','scenario_labels','method_labels');
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
    else
        y_min=min(positive_values(:));
    end
    y_min=max(0.72*y_min,1e-4);
    y_max=1;

    figure('Color','w','Name',scenario_labels{scenario_idx}, ...
        'Position',[100 100 figure_size_pixels]);
    hold on;
    % Draw solid curves first so dashed/dotted curves remain visible where
    % methods overlap. Proposed is drawn after Random-pilot so it remains
    % visible when the two solid curves coincide.
    plot_order=[5 1 2 3 4];
    plot_handles=gobjects(numel(method_labels),1);
    for draw_idx=1:numel(plot_order)
        method_idx=plot_order(draw_idx);
        y=squeeze(FDR_results(scenario_idx,method_idx,:));
        y=max(y,y_min);
        plot_handles(method_idx)=semilogy(EbN0,y,'LineWidth',1.8, ...
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
    xlabel('SNR/dB','FontSize',13);
    ylabel('FDR','FontSize',13);
    title(scenario_labels{scenario_idx}, ...
        'FontSize',13,'FontWeight','normal','Interpreter','none');
    lgd=legend(plot_handles,method_labels,'Location','southwest','FontSize',9,'Box','on');
    lgd.Color='w';
    lgd.EdgeColor='k';
    lgd.LineWidth=0.8;
    xlim([min(EbN0),max(EbN0)]);
    ylim([y_min,y_max]);
    yticks('auto');
    yticklabels('auto');
    set(gca,'FontName','Times New Roman','FontSize',12);

    if strcmp(scenario_labels{scenario_idx},'DCISA')
        % Keep every visual element inside the plotting frame.  The
        % northeast area lies above the approximately 0.5-valued cluster.
        fig=gcf;
        fig.Position=[100 100 figure_size_pixels];
        lgd.Location='east';
        lgd.NumColumns=1;
        % Match the legend typography and natural compact spacing used by
        % the other two SNR figures.
        lgd.FontSize=9;
        drawnow;

        % Leave enough clearance for the inset's left-side y tick labels,
        % not merely for the inset box itself.
        main_pos=ax.Position;
        inset_pos=[main_pos(1)+0.065 main_pos(2)+0.02 0.32 0.34];
        zoom_ax=axes('Position',inset_pos); %#ok<LAXES>
        hold(zoom_ax,'on');
        zoom_order=[1 2];
        for draw_idx=1:numel(zoom_order)
            method_idx=zoom_order(draw_idx);
            y=squeeze(FDR_results(scenario_idx,method_idx,:));
            semilogy(zoom_ax,EbN0,y,'LineWidth',1.70, ...
                'LineStyle',line_styles{method_idx}, ...
                'Marker',marker_styles{method_idx},'MarkerSize',5.4, ...
                'MarkerFaceColor','none','MarkerEdgeColor',colors(method_idx,:), ...
                'Color',colors(method_idx,:));
        end
        hold(zoom_ax,'off');
        grid(zoom_ax,'on');
        box(zoom_ax,'on');
        xlim(zoom_ax,[2 3]);
        xticks(zoom_ax,[2 2.5 3]);
        ylim(zoom_ax,[0.255 0.338]);
        yticks(zoom_ax,[0.26 0.28 0.30 0.32 0.33]);
        zoom_ax.YScale='log';
        zoom_ax.YMinorGrid='on';
        zoom_ax.GridAlpha=0.22;
        zoom_ax.MinorGridAlpha=0.16;
        zoom_ax.LineWidth=0.9;
        zoom_ax.TickDir='in';
        zoom_ax.Layer='top';
        set(zoom_ax,'FontName','Times New Roman','FontSize',9.5);

        % Point from the inset toward the corresponding 2--3 dB region.
        main_pos=ax.Position;
        target_x=main_pos(1)+main_pos(3)*(2.5-min(EbN0))/(max(EbN0)-min(EbN0));
        target_y=main_pos(2)+main_pos(4)* ...
            (log10(0.295)-log10(y_min))/(log10(y_max)-log10(y_min));
        start_x=inset_pos(1)+0.96*inset_pos(3);
        start_y=inset_pos(2)+0.82*inset_pos(4);
        annotation(fig,'arrow',[start_x target_x],[start_y target_y], ...
            'Color',[0.15 0.15 0.15],'LineWidth',1.1,'HeadLength',7,'HeadWidth',7);
    end

    safe_name=regexprep(scenario_labels{scenario_idx},'[^A-Za-z0-9]','_');
    artifact_base=sprintf('FDR_SNR_%s_final',safe_name);
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
