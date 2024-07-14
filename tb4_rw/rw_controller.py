def controller_calc(self, state):
        global iteration, state_arr, diff_arr, goal_pose_arr, velocity, omega, goal_pose_arr_rel, state_arr_rel, waypoint_number, goal_pose_arr_new, new_start

        state_arr[0][0] = state.pose.pose.position.x
        state_arr[0][1] = state.pose.pose.position.y
        quaternion = (state.pose.pose.orientation.x,
                      state.pose.pose.orientation.y,
                      state.pose.pose.orientation.z,
                      state.pose.pose.orientation.w
                     )
        euler_angles = euler.quat2euler(quaternion, 'sxyz')
        state_arr[0][2] = euler_angles[0]  # get the "yaw" data

        # store initial state
        if iteration == 0:
            for i in range(3):
                state_arr[1][i] = state_arr[0][i]
            iteration += 1
        
        # store new goal waypoint pose 
        if new_start == 0:
            for i in range(3):
                goal_pose_arr_rel[0][i] = goal_pose_arr_new[waypoint_number][i] + state_arr[1][i]
            if (waypoint_number + 1) < goal_pose_arr_new.shape[0]:
                print(f"Now going to waypoint #{waypoint_number + 1} \n ")
            elif (waypoint_number + 1) == goal_pose_arr_new.shape[0]:
                print("Now going to end goal destination \n ")
            new_start += 1

        # calculate the difference
        for j in range(3):
            diff_arr[0][j] = goal_pose_arr_rel[0][j] - state_arr[0][j]
            state_arr_rel[0][j] = state_arr[0][j] - state_arr[1][j]
        
        angle_to_goal = np.arctan2(diff_arr[0][1], diff_arr[0][0])

        # if at final goal coordinates and oriented correct, DONE
        if np.sqrt((state_arr[0][0]-goal_pose_arr_rel[0][0])**2 + (state_arr[0][1]-goal_pose_arr_rel[0][1])**2) < 0.02 and (waypoint_number + 1) == goal_pose_arr_new.shape[0] and abs(diff_arr[0][2]) <= 0.1:    
            print("Goal destination reached!? \n")
            # stop the robot when the goal destination is reached (after going through waypoints)
            velocity = 0
            omega = 0
            quit()
        
        # if at final goal coordinates, correct the orientation if needed
        if np.sqrt((state_arr[0][0]-goal_pose_arr_rel[0][0])**2 + (state_arr[0][1]-goal_pose_arr_rel[0][1])**2) <= 0.02 and abs(diff_arr[0][2]) > 0.05 and (waypoint_number + 1) == goal_pose_arr_new.shape[0]:
            velocity = 0
            spindir = (diff_arr[0][2])/abs(diff_arr[0][2])
            angularSpeed = min(0.02, abs((diff_arr[0][2])/20))
            omega = spindir*angularSpeed

        # if robot reached waypoint, say that and start going to next goal
        if np.sqrt((state_arr[0][0]-goal_pose_arr_rel[0][0])**2 + (state_arr[0][1]-goal_pose_arr_rel[0][1])**2) < 0.02 and (waypoint_number + 1) < goal_pose_arr_new.shape[0]:
            print(f"waypoint #{waypoint_number + 1} reached!? \n")
            # start going to next waypoint
            new_start = 0
            waypoint_number += 1


        # if robot isn't near destination, turn robot until it's pointing at current goal coordinates
        if np.sqrt((state_arr[0][0]-goal_pose_arr_rel[0][0])**2 + (state_arr[0][1]-goal_pose_arr_rel[0][1])**2) >= 0.02 and abs(angle_to_goal - state_arr[0][2]) > 0.1:
            scale = 20
            spinway = (angle_to_goal - state_arr[0][2])/(abs(angle_to_goal - state_arr[0][2]))
            angSpeed = min(0.05, abs((angle_to_goal - state_arr[0][2])/scale))
            omega = spinway*angSpeed
            velocity = 0
        
        # if robot is pointing in the right direction and isn't at goal coordinates, move forwards
        if np.sqrt((state_arr[0][0]-goal_pose_arr_rel[0][0])**2 + (state_arr[0][1]-goal_pose_arr_rel[0][1])**2) >= 0.02 and abs(omega) <= 0.01:
            velocity = 0.05
