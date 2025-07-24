function [] = exportPlot(x,y, a,b,type, plotTitle)
    hfig = figure("Name","Picture");  % save the figure handle in a variable

    % Set global interpreter and font settings first
    set(findall(hfig,'-property','Interpreter'),'Interpreter','latex') 
    set(findall(hfig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
    set(findall(hfig,'-property','FontSize'),'FontSize',11) % adjust fontsize to your document
    set(findall(hfig,'-property','FontName'),'FontName','Arial Narrow')
    set(findall(hfig,'-property','Box'),'Box','off') % optional

    switch type
        case 'nyquist'
            plot(x, y, '.', 'DisplayName','Actual Data','MarkerSize',20,'Color',"#3D1A69");
            %plot(x, y,'-o','Color',"#3D1A69","LineWidth",4); % Plot Z_real vs -Z_imag
            hold on;
            plot(a, b, '.', 'DisplayName','Actual Data','MarkerSize',20,'Color',"#fc0303");

            title(plotTitle, 'Interpreter', 'latex', 'FontSize', 32)
            xlabel('Z_{real} \Omega','FontSize', 20,'FontName', 'Arial Narrow'); 
            ylabel('-Z_{imag} \Omega','FontSize', 20,'FontName', 'Arial Narrow');
            legend("Water","Water + Benadryl","Location","best");
            ylim([0 7e4]);
            xlim([0 6.5e4]);

            fname = plotTitle;

            picturewidth = 20; % set this parameter and keep it forever
            hw_ratio = 0.65; % feel free to play with this ratio    
            set(hfig,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])
            pos = get(hfig,'Position');
            set(hfig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])

            print(hfig,fname,'-dpng','-painters')
        
        case 'KK'
            h = semilogx(x,y);
            set(h,'LineWidth',2.5,'LineStyle','--','Color',"#3D1A69")
            hold on; 
            h = semilogx(a,b);
            set(h,'LineWidth',2.5,'LineStyle','-','Color',"#fc0303")
            grid on;

            title(plotTitle, 'Interpreter', 'latex', 'FontSize', 32)
            xlabel('Frequency (Hz)','FontSize', 20,'FontName', 'Arial Narrow'); 
            ylabel('Reactance (\Omega)','FontSize', 20,'FontName', 'Arial Narrow');

            legend('Measurement','Kramers-Kronig','Location','');
            %set(h,'Box','on','Color','w','Location','NorthEast','FontSize',12,'FontWeight','n','FontAngle','n')

            xlim([1 max(freq)])

            fname = plotTitle;

            picturewidth = 20; % set this parameter and keep it forever
            hw_ratio = 0.65; % feel free to play with this ratio    
            set(hfig,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])
            pos = get(hfig,'Position');
            set(hfig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])

            print(hfig,fname,'-dpng','-painters')

        otherwise 
            warning('Unexpected plot type')
    end 
end
