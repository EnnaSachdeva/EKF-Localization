function [ theta ] = normalize_angle( theta )
    if theta< -pi
            theta=theta+2*pi;
    else if (theta > pi)
            theta=theta-2*pi;
        else theta=theta;
    end
 end

