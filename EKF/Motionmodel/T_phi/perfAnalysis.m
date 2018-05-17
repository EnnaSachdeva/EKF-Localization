function [] = perfAnalysis(x_axis,update,actual,think, fig_x,fig_y,fig_theta)
 
% cla;

pose_x=[update(:,1) actual(:,1) think(:,1)];
pose_y=[update(:,2) actual(:,2) think(:,2)];
pose_theta=[update(:,3) actual(:,3) think(:,3)];

% err_think_true=[abs(actual(:,1)-think(:,1)) abs(update(:,1)-actual(:,1)) abs(actual(:,2)-think(:,2)) abs(update(:,2)-actual(:,2)) abs(actual(:,3)-think(:,3)) abs(update(:,3)-actual(:,3))]



    figure(fig_x);
    h=bar(x_axis,pose_x);

    title('robot poses x')
    xlabel('control command') % x-axis label
    ylabel('error value') % y-axis label
    l{1}='updated after EKF'; l{2}='actual, where it actually is';  l{3}='think, where it thinks';  
    legend(h,l); 
   
    
    figure(fig_y);
    h=bar(x_axis,pose_y);

    title('robot poses y')
    xlabel('control command') % x-axis label
    ylabel('error value') % y-axis label
    l{1}='updated after EKF'; l{2}='actual, where it actually is';  l{3}='think, where it thinks';  
    legend(h,l); 
   
    
    
    figure(fig_theta);
    h=bar(x_axis,pose_theta);

    title('robot poses theta')
    xlabel('control command') % x-axis label
    ylabel('error value') % y-axis label
    l{1}='updated after EKF'; l{2}='actual, where it actually is';  l{3}='think, where it thinks';  
    legend(h,l); 
   
    
%     
    
    
%     figure(fig_y);
%     bar(x_axis,error(:,2),'b');
%     title('error |mu(y hat) − mu(y true)|')
%     xlabel('control command') % x-axis label
%     ylabel('error value') % y-axis label
%     legend('Incremental')
%     hold on
%     
%     figure(fig_theta)
%     bar(x_axis,error(:,3),'b');
%     title('error |mu(theta hat) − mu(theta true)|')
%     xlabel('control command') % x-axis label
%     ylabel('error value') % y-axis label
%     legend('Incremental')
%     hold on
    
    
    
    
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

  
