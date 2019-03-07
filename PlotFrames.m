% Title: Function to plot initital frame and transformed frame
% Author: Shreyash Annapureddy
% Date: 7/03/2016

function PlotFrames(T,init_frame)
    desired_frame = T*init_frame;
    
    plot3(init_frame(1,:),init_frame(2,:),init_frame(3,:),'r');
    hold on;
    
    plot3(desired_frame(1,:),desired_frame(2,:),desired_frame(3,:),'b');
    hold off;
    
    grid on
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
end