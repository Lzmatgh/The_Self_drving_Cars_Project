function target_path = AvoidObstacles_jjc(TestTrack,Xobs)

    bl = TestTrack.bl;
    br = TestTrack.br;
    cline = TestTrack.cline;
    theta = TestTrack.theta;


    %%
    target_path = [cline;theta];

    % if no obs, set our target_path as cline
    if size(Xobs) == 0
        target_path = [cline;theta]; 
    else
    %if see obs

        idx_end = size(cline,2);% total track number
        idx = 2; % track index
        linear_interp_points = 20; % linear interp number
        obs_idx = 1; % obs passed number

        Obs_exist = false; % whether detected obs
        Obs_detected = zeros(size(Xobs{1})); % save obs detected

        avoid_obs_num = 1; % pass obs by using this num's forward


        while idx < idx_end
            % linear interp between current points and previous points
            x = linspace(cline(1,idx-1),cline(1,idx),linear_interp_points);
            y = linspace(cline(2,idx-1),cline(2,idx),linear_interp_points);


            % detect whether there is any obs between two points

            if obs_idx <= size(Xobs,2)

                % get circle around obs
                Obs = Xobs{obs_idx};
                Obs_center = mean(Obs);
                Obs_radius = norm(mean(Obs)-Obs(1,:))+0.1;

                % detect whether the line intersect with the obs circle
                for i = 1:length(x)
                    if norm([x(i),y(i)] - Obs_center)< Obs_radius
                        Obs_exist = true;
                        Obs_detected = Obs;
                        continue
                    end
                end

            end

            % if Obs exist in this idx
            if Obs_exist
                % find detected obs center
                Obs_center_detected = mean(Obs_detected);

                % find the min distance from obs center to right/left 
    %             left_distance = norm(bl(:,idx)-Obs_center_detected);
    %             right_distance = norm(br(:,idx)-Obs_center_detected);
                [~, left_distance] = knnsearch(bl', Obs_center_detected);
                [~, right_distance] = knnsearch(br', Obs_center_detected);

                % avoid obs by changing lane
                if right_distance < left_distance % if object is on right
                    target_path(1:2,idx-1:idx+avoid_obs_num) = (2*bl(:,idx-1:idx+avoid_obs_num)+cline(:,idx-1:idx+avoid_obs_num)) ./ 3;

                else % if object is on left
                    target_path(1:2,idx-1:idx+avoid_obs_num) = (2*br(:,idx-1:idx+avoid_obs_num)+cline(:,idx-1:idx+avoid_obs_num)) ./ 3;
                end

                obs_idx  = obs_idx + 1; % passed the obs
            end

            % recover values
            idx = idx + 1;

            Obs_exist = false;
            Obs_detected = zeros(size(Xobs{1}));


        end

        % Set Heading
        theta_new = atan2(diff(target_path(2,:)),diff(target_path(1,:)));
        theta_new(end+1) = theta_new(end);

        target_path(3,:) = theta_new;

    end


end