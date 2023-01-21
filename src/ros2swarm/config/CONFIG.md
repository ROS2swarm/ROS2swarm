## Configuration Parameters per Pattern

### Movement Pattern 

#### Basic 

##### Aggregation (aggregation_pattern.yaml)

##### Attraction (attraction_pattern.yaml)
    
| Parameter             | Value       | Description |
| -----------           | ----------- | ----------- |
| attraction_max_range    | float     | maximum distance of obstacles to be considered for dispersion |
| attraction_min_range | float | minimum distance of obstacles to be considered for dispersion |               
| attraction_front_attraction | float | constant linear speed added to repulsive vector | 
| attraction_threshold | int | number of sensor measurements within [min_range, max_range] to trigger behavior | 
| attraction_linear_if_alone | float | linear speed of robot when no other robots are detected within [min_range, max_range] | 
| attraction_angular_if_alone | float | angular speed of robot when no other robots are detected within [min_range, max_range] | 
| max_translational_velocity | float | maximum allowed linear speed of robot |  
| max_rotational_velocity | float | maximum allowed angular speed of robot | 


##### Attraction 2 (attraction_pattern2.yaml)

| Parameter             | Value       | Description |
| -----------           | ----------- | ----------- |
| attraction2_max_range    | float     | maximum distance of obstacles to be considered for dispersion |
| attraction2_min_range | float | minimum distance of obstacles to be considered for dispersion |               
| attraction2_front_attraction | float | constant linear speed added to repulsive vector | 
| attraction2_threshold | int | number of sensor measurements within [min_range, max_range] to trigger behavior | 
| attraction2_linear_if_alone | float | linear speed of robot when no other robots are detected within [min_range, max_range] | 
| attraction2_angular_if_alone | float | angular speed of robot when no other robots are detected within [min_range, max_range] | 
| max_translational_velocity | float | maximum allowed linear speed of robot |  
| max_rotational_velocity | float | maximum allowed angular speed of robot | 


##### Dispersion (dispersion_pattern.yaml)

| Parameter             | Value       | Description |
| -----------           | ----------- | ----------- |
| dispersion_max_range    | float     | maximum distance of obstacles to be considered for dispersion |
| dispersion_min_range | float | minimum distance of obstacles to be considered for dispersion |               
| dispersion_front_attraction | float | constant linear speed added to repulsive vector | 
| dispersion_threshold | int | number of sensor measurements within [min_range, max_range] to trigger behavior | 
| dispersion_linear_if_alone | float | linear speed of robot when no other robots are detected within [min_range, max_range] | 
| dispersion_angular_if_alone | float | angular speed of robot when no other robots are detected within [min_range, max_range] | 
| dispersion_allow_dynamic_max_range_setting | true, false | if true parameter max_range can be set via topic max_distance | 
| max_translational_velocity | float | maximum allowed linear speed of robot |  
| max_rotational_velocity | float | maximum allowed angular speed of robot | 


##### Drive (drive_pattern.yaml)

| Parameter             | Value       | Description |
| -----------           | ----------- | ----------- |
| drive_timer_period    | float       | time in seconds between publishing current Twist() message to cmd_vel |
| drive_linear          | float       | linear velocity of robot  |
| drive_angular         | float       | angular velocity of robot |


##### Minimalist Flocking (minimalist_flocking_pattern.yaml)


##### Random Walk (random_walk_pattern.yaml)

| Parameter                                            | Value       | Description |
| -----------                                          | ----------- | ----------- |
| random_walk_timer_period                             | float       | time in seconds between publishing current Twist() message to cmd_vel |
| random_walk_linear                                   | float       | linear velocity of robot |
| random_walk_angular                                  | float       | maximum angular speed of robot |
| random_walk_rot_interval                             | float       | time in seconds robot turns randomly drawn from [0, random_walk_rot_interval] |
| random_walk_lin_interval_min, random_walk_lin_interval_max | float | time in seconds robot drives straight randomly drawn from [random_walk_lin_interval_min, random_walk_lin_interval_max]|



#### Combined 

##### Discussed Dispersion (discussed_dispersion_pattern.yaml)



### Voting Pattern 

#### Basic 

##### Majority Rule (majority_rule_pattern.yaml)

| Parameter                                            | Value       | Description |
| -----------                                          | ----------- | ----------- |
| majority_rule_choose_start_value_at_random           | true, false | initialize robot start value randomly within min/max opinion value |
| majority_rule_initial_value                          | int         | if majority_rule_choose_start_value_at_random is False, this value is set as initial opinion |
| majority_rule_min_opinion, majority_rule_max_opinion | int         | if majority_rule_choose_start_value_at_random is True, the initial value is initalized randomly in [majority_rule_min_opinion, majority_rule_max_opinion] |
| majority_rule_timer_period                           | float       | time in seconds between opinion updates |


##### Voter Model (voter_model_pattern.yaml)

| Parameter                                        | Value       | Description |
| -----------                                      | ----------- | ----------- |
| voter_model_choose_start_value_at_random         | true, false | initialize robot start value randomly within min/max opinion value |
| voter_model_initial_value                        | int         | if majority_rule_choose_start_value_at_random is False, this value is set as initial opinion |
| voter_model_min_opinion, voter_model_max_opinion | int         | if majority_rule_choose_start_value_at_random is True, the initial value is initalized randomly in [voter_model_min_opinion, voter_model_max_opinion] |
| voter_model_timer_period                         | float       | time in seconds between opinion updates |
    
    
    
#### Combined 


