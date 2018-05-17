function [] = perfAnalysis(x_axis,error,fig_x, fig_y,fig_theta)
 
% cla;

    figure(fig_x);
    bar(x_axis,error(:,1),'b');

    title('error |mu(x hat) − mu(x true)|')
    xlabel('control command') % x-axis label
    ylabel('error value') % y-axis label
    legend('Incremental')
    hold on
   
    figure(fig_y);
    bar(x_axis,error(:,2),'b');
    title('error |mu(y hat) − mu(y true)|')
    xlabel('control command') % x-axis label
    ylabel('error value') % y-axis label
    legend('Incremental')
    hold on
    
    figure(fig_theta)
    bar(x_axis,error(:,3),'b');
    title('error |mu(theta hat) − mu(theta true)|')
    xlabel('control command') % x-axis label
    ylabel('error value') % y-axis label
    legend('Incremental')
    hold on
    
    
    
    
%     figure(fig_x);
%     plot(x_axis,error(:,1),'-*',...
%                         'LineWidth',1,...
%                         'MarkerEdgeColor','r',...
%                         'MarkerFaceColor',[.49 1 .63],...
%                         'MarkerSize',5);
% 
%     title('error |mu(x hat) − mu(x true)|')
%     xlabel('control command') % x-axis label
%     ylabel('error value') % y-axis label
%     legend('Incremental')
%     hold on
%    
%     figure(fig_y);
%     plot(x_axis,error(:,2),'-*',...
%                         'LineWidth',1,...
%                         'MarkerEdgeColor','g',...
%                         'MarkerFaceColor',[.49 1 .63],...
%                         'MarkerSize',5);
%     title('error |mu(y hat) − mu(y true)|')
%     xlabel('control command') % x-axis label
%     ylabel('error value') % y-axis label
%     legend('Incremental')
%     hold on
%     
%     figure(fig_theta)
%     plot(x_axis,error(:,3),'-*',...
%                         'LineWidth',1,...
%                         'MarkerEdgeColor','m',...
%                         'MarkerFaceColor',[.49 1 .63],...
%                         'MarkerSize',5);
%     title('error |mu(theta hat) − mu(theta true)|')
%     xlabel('control command') % x-axis label
%     ylabel('error value') % y-axis label
%     legend('Incremental')
%     hold on
%     
%     

  
