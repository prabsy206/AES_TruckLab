function [stop_flag, obstacle_dis] = collisionCheck(scan, min_dis)

stop_flag = 0;
obstacle_dis = 15;
fov_threshold = 60;
ranges = scan.Ranges;
anglesDeg = rad2deg(scan.Angles);
    for j=1:length(anglesDeg)
        if anglesDeg(j)>=180-fov_threshold || anglesDeg(j)<=-180+fov_threshold
            if ranges(j)<=min_dis && ranges(j)~=0
                stop_flag = 1;
                obstacle_dis = ranges(j);
                break
            end
        end
    end
end