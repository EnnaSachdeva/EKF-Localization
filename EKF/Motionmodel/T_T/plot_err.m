function [error] = plot_err(m,estimatedPose,robotPose,fig_x, fig_y,fig_theta)
 
% cla;
error= [estimatedPose.x-robotPose.x;estimatedPose.y-robotPose.y;estimatedPose.theta-robotPose.theta];
%     plot(m,error(1),'g*',m,error(2),'b*',m,error(3),'r*');
%   title(['sane points plot : ',num2str(alpha),'  confidence'])
%     plot(m,error,'g*'); 
    figure(fig_x);
    errorbar(m,robotPose.x, error(1),'.');
    hold on
    
    figure(fig_y);
    errorbar(m,robotPose.y ,error(2),'.');
    hold on
    
    figure(fig_theta)
    errorbar(m,robotPose.theta,error(3),'.');
    hold on
    
%     xlabel('x coordinates of data points') % x-axis label
%     ylabel('y coordinates of data points') % y-axis label
% % %     legend('data points')
    
end

