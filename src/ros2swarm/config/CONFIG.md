## Configuration Parameters per Pattern

### Movement Pattern 

#### Basic 

##### Drive (drive_pattern.yaml)

| Parameter             | Value       | Description |
| -----------           | ----------- | ----------- |
| drive_timer_period    | float       | time in seconds between publishing current Twist() message to cmd_vel |
| drive_linear          | float       | linear velocity of robot  |
| drive_angular         | float       | angular velocity of robot |

##### Random Walk (random_walk_pattern.yaml)

| Parameter                                            | Value       | Description |
| -----------                                          | ----------- | ----------- |
| random_walk_timer_period                             | float       | time in seconds between publishing current Twist() message to cmd_vel |
| random_walk_linear                                   | float       | linear velocity of robot |
| random_walk_angular                                  | float       | |
| random_walk_rot_interval                             | float       | |
| random_walk_lin_interval_min                         | float       | |
| random_walk_lin_interval_max                         | float       | |



#### Combined 




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


