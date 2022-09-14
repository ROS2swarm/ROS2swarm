import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from utils import get_data_per_exp

COLORS = {'a':'green','b':'red','c':'blue'}
COLOR = ['green','red','blue']

def plot_two_exp(name=""):



    OFFSET = 50
    fig, axs = plt.subplots(nrows=2, ncols=1)
    df_item_list, df_item_removed, name_lists = get_data_per_exp()


    '''File and dataframe loading'''
    for file_name in range(len(name_lists[0])): #Search for log_items files
        if(name != "" and name in name_lists[0][file_name]):
            exp_id_item = file_name

    for file_name in range(len(name_lists[1])):
        if(name != "" and name in name_lists[1][file_name]): # Search for log_items_removed files
            exp_id_remove = file_name


    file_name_items = name_lists[0][exp_id_item]
    file_name_items_removed = name_lists[1][exp_id_remove]



    exp_item_df = df_item_list[exp_id_item] # log_items dataframe
    exp_removed_df = df_item_removed[exp_id_remove].fillna(-1) # log_items_removed dataframe without NaN



    '''System time to simulation time'''
    start_time = int(exp_removed_df['end_time'].iloc[0])  # first recorded time in the simulation
    exp_removed_df['diff_time'] = exp_removed_df['start_time'] - start_time # Convert system time to simulation time
    exp_removed_df['diff_time'][:6] = exp_removed_df['end_time'][:6] - start_time # Same but the 6 first elements are set with end_time due to unavailability of start_time
    exp_removed_df['real_time_taken'] = exp_removed_df['end_time'] - exp_removed_df['start_time'] # Same to time_taken

    exp_item_df['diff_time'] = exp_item_df['time'] - start_time # Same for log_items


    '''Plot parameters'''
    print(start_time)
    print(exp_item_df.head(5))

    axs[0].set_xlim(0,exp_item_df['diff_time'].iloc[-1])
    axs[1].set_xlim(0, exp_item_df['diff_time'].iloc[-1])
    axs[0].set_ylim(0,30)
    axs[1].set_ylim(0, 6)

    axs[0].set_title(f"Remaining items per types over time ({file_name_items})")
    axs[1].set_title(f"Number of working robots by tasks type over time ({file_name_items_removed})")
    print(f"Choosen File: {file_name_items} | {file_name_items_removed}")

    axs[0].set_xlabel('Time (second)')
    axs[1].set_xlabel('Time (second)')

    axs[0].set_ylabel('Number of items')
    axs[1].set_ylabel('Number of robots')

    '''Plot Number of remaining items by type across time'''
    for col in exp_item_df.columns:
        if('items' in col):
            item_name = col.split('_')[1]
            axs[0].plot(exp_item_df['diff_time'],exp_item_df[col].values,label=f"{col}",color=COLORS[item_name])
    axs[0].legend()
    axs[0].grid()


    '''Prepare second experience visualization'''
    robot_dic = {}

    for i in range(len(exp_removed_df['robot_name'])):
        robot_name = exp_removed_df['robot_name'].iloc[i]

        if(not robot_name in robot_dic.keys()): #robot_dic{"/robot_namespace_0":}
            robot_dic[robot_name] = []
        if(int(exp_removed_df['start_time'].iloc[i]) != -1):
            robot_dic[robot_name].append([int(exp_removed_df['diff_time'].iloc[i]),int(exp_removed_df['diff_time'].iloc[i]) + int(exp_removed_df['real_time_taken'].iloc[i]),int(exp_removed_df['item_picked'].iloc[i])])
            # robot_dic{"/robot_namespace_0":[starting_time, starting_time + time_taken, item_type]}

    '''Time management to match work period'''
    corrected_timestamps = []
    for elem in range(len(exp_removed_df['end_time'])):

        if(exp_removed_df['start_time'].iloc[elem] != -1):
            val = exp_removed_df['start_time'].iloc[elem]
            corrected_timestamps.append(val - start_time)
        else:
            val = exp_removed_df['end_time'].iloc[elem]
            corrected_timestamps.append(val - start_time)

    total_second =  (int(exp_removed_df['end_time'].iloc[len(exp_removed_df)-1]) - int(exp_removed_df['end_time'].iloc[0]))
    print(f"Total simulation time: {total_second} seconds | {round(total_second/60)} mins")

    number_of_working_robots = [[],[],[]] # one sub array per item type
    max_length = len(exp_removed_df['diff_time'])-1
    timesteps = np.arange(exp_removed_df['diff_time'].iloc[0],exp_removed_df['diff_time'].iloc[max_length]+exp_removed_df['real_time_taken'].iloc[max_length]+OFFSET)
    #timesteps = [0,simulation_time + OFFSET]

    '''Counting number of working robots per time period and their task type'''
    for timestep in timesteps:
        counter =  [0,0,0]
        for key in robot_dic.keys():
            for working_time in robot_dic[key]:
                if(working_time[0] <= timestep <= working_time[1]):
                    counter[working_time[2]] += 1

        for sub_array in range(len(number_of_working_robots)):
            number_of_working_robots[sub_array].append(counter[sub_array])

    mapping_label={0:'items_a',1:'items_b',2:'items_c'}
    for sub_array in range(len(number_of_working_robots)):
        #print(number_of_working_robots[sub_array])

        axs[1].plot(timesteps,number_of_working_robots[sub_array], label=mapping_label[sub_array],color=COLOR[sub_array])



    axs[1].legend()
    axs[1].grid()
    plt.show()


plot_two_exp(name="breakdown")

