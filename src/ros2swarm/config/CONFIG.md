## Configuration Parameters per Pattern

### Movement Pattern 

#### Basic 

##### Aggregation (*aggregation_pattern.yaml*)

| Parameter                             | Value                   | Description                                                                                                             |
| :-----------                          | :----------             | :----------                                                                                                             |
| aggregation_max_range                 | float                   | maximum distance of obstacles to be considered for aggregation                                                          |
| aggregation_min_range                 | float                   | minimum distance of obstacles to be considered for aggregation                                                          |
| aggregation_front_attraction          | float                   | linear forward component added to direction vector for robot movement                                                   |
| aggregation_time_scale                | float                   | scaling factor (time in seconds) to extend time in group for every detected neighbor in group                           |
| aggregation_base_stay_time            | float                   | minimum time in seconds robots stay in group                                                                            |
| aggregation_group_distance            | float                   | maximum distance in meters between robots to be considered in a group                                                   |
| aggregation_object_reduction          | MEAN, NEAREST           | defines range value used to represent robot (mean distance, nearest distance)                                           |
| aggregation_group_reference_selection | MEAN, NEAREST, FARTHEST | defines which robot in group is considered for defining movement vector to leave group (nearest, mean, farthest object) |
| aggregation_object_threshold          | float                   | maximum length difference between two ranges to map them to the same object                                             |
| aggregation_object_min_width          | int                     | minimum number of sensor measurements (ranges) a robot is wide                                                          |
| aggregation_object_max_width          | int                     | maximum number of sensor measurements (ranges) a robot is wide                                                          |
| aggregation_stay_in_growing_groups    | true, false             | if true, robots will continue waiting in group if group size increased while waiting                                    | 
| max_translational_velocity            | float                   | maximum allowed linear speed of robot                                                                                   |  
| max_rotational_velocity               | float                   | maximum allowed angular speed of robot                                                                                  |     


##### Attraction (*attraction_pattern.yaml*)
    
| Parameter                   | Value       | Description                                                                            |
| :----------                 | :---------- | :----------                                                                            |
| attraction_max_range        | float       | maximum distance of obstacles to be considered for dispersion                          |
| attraction_min_range        | float       | minimum distance of obstacles to be considered for dispersion                          |               
| attraction_front_attraction | float       | constant linear speed added to repulsive vector                                        | 
| attraction_threshold        | int         | number of sensor measurements within [min_range, max_range] to trigger behavior        | 
| attraction_linear_if_alone  | float       | linear speed of robot when no other robots are detected within [min_range, max_range]  | 
| attraction_angular_if_alone | float       | angular speed of robot when no other robots are detected within [min_range, max_range] | 
| max_translational_velocity  | float       | maximum allowed linear speed of robot                                                  |  
| max_rotational_velocity     | float       | maximum allowed angular speed of robot                                                 | 


##### Attraction 2 (*attraction_pattern2.yaml*)

| Parameter                    | Value       | Description                                                                            |
| :----------                  | :---------- | :----------                                                                            |
| attraction2_max_range        | float       | maximum distance of obstacles to be considered for dispersion                          |
| attraction2_min_range        | float       | minimum distance of obstacles to be considered for dispersion                          |               
| attraction2_front_attraction | float       | constant linear speed added to repulsive vector                                        | 
| attraction2_threshold        | int         | number of sensor measurements within [min_range, max_range] to trigger behavior        | 
| attraction2_linear_if_alone  | float       | linear speed of robot when no other robots are detected within [min_range, max_range]  | 
| attraction2_angular_if_alone | float       | angular speed of robot when no other robots are detected within [min_range, max_range] | 
| max_translational_velocity   | float       | maximum allowed linear speed of robot                                                  |  
| max_rotational_velocity      | float       | maximum allowed angular speed of robot                                                 | 


##### Dispersion (*dispersion_pattern.yaml*)

| Parameter                                  | Value       | Description                                                                            |
| :----------                                | :---------- | :----------                                                                            |
| dispersion_max_range                       | float       | maximum distance of obstacles to be considered for dispersion                          |
| dispersion_min_range                       | float       | minimum distance of obstacles to be considered for dispersion                          |               
| dispersion_front_attraction                | float       | constant linear speed added to repulsive vector                                        | 
| dispersion_threshold                       | int         | number of sensor measurements within [min_range, max_range] to trigger behavior        | 
| dispersion_linear_if_alone                 | float       | linear speed of robot when no other robots are detected within [min_range, max_range]  | 
| dispersion_angular_if_alone                | float       | angular speed of robot when no other robots are detected within [min_range, max_range] | 
| dispersion_allow_dynamic_max_range_setting | true, false | if true parameter max_range can be set via topic max_distance                          | 
| max_translational_velocity                 | float       | maximum allowed linear speed of robot                                                  |  
| max_rotational_velocity                    | float       | maximum allowed angular speed of robot                                                 | 


##### Drive (*drive_pattern.yaml*)

| Parameter             | Value       | Description                                                           |
| :----------           | :---------- | :----------                                                           |
| drive_timer_period    | float       | time in seconds between publishing current Twist() message to cmd_vel |
| drive_linear          | float       | linear velocity of robot                                              |
| drive_angular         | float       | angular velocity of robot                                             |


##### Minimalist Flocking (*minimalist_flocking_pattern.yaml*)

| Parameter                                     | Value                        | Description                                                                 |
| :----------                                   | :----------                  | :----------                                                                 |
| minimalist_flocking_translational_velocity    | float                        | translational velocity of robots when moving forward                        |
| minimalist_flocking_rotational_left_velocity  | float                        | rotational velocity of robots when turning left                             |
| minimalist_flocking_rotational_right_velocity | float                        | rotational velocity of robots when turning right                            |
| minimalist_flocking_drive_timer_period        | int                          | number of updates/cycles robot stays in a state (i.e., executes a behavior) |
| minimalist_flocking_zone1_threshold           | float                        | max. distance (range) for zone 1                                            | 
| minimalist_flocking_zone2_threshold           | float                        | max. distance (range) for zone 2                                            | 
| minimalist_flocking_zone3_threshold           | float                        | min. distance (range) for zone 3                                            | 
| minimalist_flocking_zone4_threshold           | float                        | min. distance (range) for zone 4                                            | 
| minimalist_flocking_zone2_robot_threshold     | int                          | max. ranges considered to be a robot, otherwise considered a wall           | 
| minimalist_flocking_robot_threshold           | int                          | min. number of sensor measurements to be considered a robot                 | 
| minimalist_flocking_zone_borders              | [float, float, float, float] | array of angles (radian) defining zone borders                              | 
| max_range                                     | float                        | max. sensor range                                                           | 
| min_range                                     | float                        | min. sensor range                                                           | 


##### Random Walk (*random_walk_pattern.yaml*)

| Parameter                                                  | Value       | Description                                                                   |
| :----------                                                | :---------- | :----------                                                                   |
| random_walk_timer_period                                   | float       | time in seconds between publishing current Twist() message to cmd_vel         |
| random_walk_linear                                         | float       | linear velocity of robot                                                      |
| random_walk_angular                                        | float       | maximum angular speed of robot                                                |
| random_walk_rot_interval                                   | float       | time in seconds robot turns randomly drawn from [0, random_walk_rot_interval] |  
| random_walk_lin_interval_min, random_walk_lin_interval_max | float       | time in seconds robot drives straight randomly drawn from [random_walk_lin_interval_min, random_walk_lin_interval_max]|



#### Combined 

##### Discussed Dispersion (*discussed_dispersion_pattern.yaml*)

| Parameter                                        | Value       | Description                                                                                      |
| :----------                                      | :---------- | :----------                                                                                      |
| discussed_dispersion_timer_period                | float       | time in seconds between updating opinion and publishing current Twist() message to cmd_vel       |
| discussed_dispersion_discussion_time             | float       |  initial time in seconds in which only majority rule is executed (no robot movement)             |
| discussed_dispersion_discussion_opinion_multiply | float       | value to map opinion (int) to distance in meters                                                 |
| discussed_dispersion_discussion_base_distance    | float       | minimum dispersion distance, total distance = discussed_dispersion_discussion_base_distance + (opinion x discussed_dispersion_discussion_opinion_multiply) |
| dispersion parameters                            |             | see movement patterns - basic - dispersion, set dispersion_allow_dynamic_max_range_setting: True |
| majority rule parameters                         |             | see voting patterns - basic - majority rule                                                      |



### Voting Pattern 

#### Basic 

##### Majority Rule (*majority_rule_pattern.yaml*)

| Parameter                                            | Value       | Description                                                                                  |
| :----------                                          | :---------- | :----------                                                                                  |
| majority_rule_choose_start_value_at_random           | true, false | initialize robot start value randomly within min/max opinion value                           |
| majority_rule_initial_value                          | int         | if majority_rule_choose_start_value_at_random is False, this value is set as initial opinion |
| majority_rule_min_opinion, majority_rule_max_opinion | int         | if majority_rule_choose_start_value_at_random is True, the initial value is initalized randomly in [majority_rule_min_opinion, majority_rule_max_opinion] |
| majority_rule_timer_period                           | float       | time in seconds between opinion updates                                                      |


##### Voter Model (*voter_model_pattern.yaml*)

| Parameter                                        | Value       | Description                                                                                  |
| :----------                                      | :---------- | :----------                                                                                  |
| voter_model_choose_start_value_at_random         | true, false | initialize robot start value randomly within min/max opinion value                           |
| voter_model_initial_value                        | int         | if majority_rule_choose_start_value_at_random is False, this value is set as initial opinion |
| voter_model_min_opinion, voter_model_max_opinion | int         | if majority_rule_choose_start_value_at_random is True, the initial value is initalized randomly in [voter_model_min_opinion, voter_model_max_opinion] |
| voter_model_timer_period                         | float       | time in seconds between opinion updates                                                      |
    
    
    
#### Combined 


