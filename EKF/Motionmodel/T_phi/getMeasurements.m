function measurements = getMeasurements(pose, params, sensingRange)
% GETMEASUREMENTS  Fires the robot's sensor and obtains measurements. Each
% measurement is made to point landmarks in the robot's sensing range and
% consists of the following three components - id, range, and bearing. Id
% is an integer corresponding to the index of the landmark in the map.
% Range is the distance from the robot's location to the landmark. Bearing
% is the relative rotation of the landmark (measured in the counter
% clockwise direction) with respect to the robot's heading. It is taken as
% that angle which is less than pi.


% Matrix to hold measurements
measurements = zeros(size(params.map, 1),3);

% For each landmark in the map
for i = 1:size(params.map, 1)
    % X and Y coordinates of the current landmark
    m_x = params.map(i,1);
    m_y = params.map(i,2);
    % TODO: Check if the landmark is within the robot's sensing range and
    % add it to the measurements. Note the format of each measurement from
    % the long comment at the beginning of this function. This is different
    % (but, only slightly) from what we did in class.
    bearing= atan2((m_y-pose.y),(m_x-pose.x));
    
    %%%%%%%%% Normalize the angles between -pi to +pi  
    if(bearing< -pi)
        bearing=bearing+2*pi;
    else
        if(bearing > pi)
        bearing=bearing-2*pi;
        end
    end
    range=sqrt((m_x-pose.x).^2+(m_y-pose.y).^2);
     
    if range<=sensingRange
          measurements(i,:)=[i,range,bearing];
    else  
          measurements(i,:)=[i,0,0];
    end
   
end

end
