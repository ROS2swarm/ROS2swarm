import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from utils import get_data_per_exp

COLORS = {'a':'green','b':'red','c':'blue'}
COLOR = ['green','red','blue']

def plot_two_exp(exp_id = 0, name=""):

    N = 2
    OFFSET = 50
    fig, axs = plt.subplots(nrows=2, ncols=1)



    df_item_list, df_item_removed, name_lists = get_data_per_exp()

    exp_id_item = exp_id
    exp_id_remove = exp_id

    for file_name in range(len(name_lists[0])):
        if(name != "" and name in name_lists[0][file_name]):
            exp_id_item = file_name

    for file_name in range(len(name_lists[1])):
        if(name != "" and name in name_lists[1][file_name]):
            exp_id_remove = file_name
    print(name_lists)
    print(f"Choosen File: {name_lists[0][exp_id_item]} | {name_lists[1][exp_id_remove]}")


    id_item = exp_id_item
    id_remove = exp_id_remove

    exp_item_df = df_item_list[id_item]
    exp_removed_df = df_item_removed[id_remove].fillna(-1)

    start_time = int(exp_removed_df['end_time'].iloc[0])
    #print(start_time)

    exp_removed_df['diff_time'] = exp_removed_df['start_time'] - start_time
    exp_removed_df['diff_time'][:6] = exp_removed_df['end_time'][:6] - start_time

    exp_removed_df['real_time_taken'] = exp_removed_df['end_time'] - exp_removed_df['start_time']

    #exp_removed_df['real_time_taken'][:6] = exp_removed_df['start_time'][:6] - exp_removed_df['end_time']


    #print(exp_removed_df['diff_time'])


    '''print(exp_removed_df[:6].head(5))
    exp_removed_df = exp_removed_df.fillna(-1)
    print(exp_removed_df[:6].head(5))
    exit("end")'''

    start_time_item = int(exp_item_df['time'].iloc[0])
    exp_item_df['diff_time'] = exp_item_df['time'] - start_time
    print(start_time)
    print(exp_item_df.head(5))

    axs[0].set_xlim(0,exp_item_df['diff_time'].iloc[-1])
    axs[1].set_xlim(0, exp_item_df['diff_time'].iloc[-1])


    axs[0].set_ylim(0,30)
    axs[1].set_ylim(0, 6)

    #exp_item_df['diff_time'].iloc[-1] += OFFSET

    for col in exp_item_df.columns:
        if('items' in col):
            item_name = col.split('_')[1]
            axs[0].plot(exp_item_df['diff_time'],exp_item_df[col].values,label=f"{col}",color=COLORS[item_name])

    print(len(exp_item_df['diff_time']))
    #axs[0].title(f'{name_lists[0][exp_id]} - Number of remaining items by type across time')

    axs[0].legend()
    axs[0].grid()



    robot_dic = {}

    for i in range(len(exp_removed_df['robot_name'])):
        robot_name = exp_removed_df['robot_name'].iloc[i]

        if(not robot_name in robot_dic.keys()):
            robot_dic[robot_name] = []
        if(int(exp_removed_df['start_time'].iloc[i]) != -1):
            robot_dic[robot_name].append([int(exp_removed_df['diff_time'].iloc[i]),int(exp_removed_df['diff_time'].iloc[i]) + int(exp_removed_df['real_time_taken'].iloc[i]),int(exp_removed_df['item_picked'].iloc[i])])



    #print(int(exp_removed_df['end_time'].iloc[0]),int(exp_removed_df['end_time'].iloc[len(exp_removed_df)-1]))

    #timesteps = np.arange(int(exp_removed_df['end_time'].iloc[0]),int(exp_removed_df['end_time'].iloc[len(exp_removed_df)-1]))
    corrected_timestamps = []
    for elem in range(len(exp_removed_df['end_time'])):

        if(exp_removed_df['start_time'].iloc[elem] != -1):
            val = exp_removed_df['start_time'].iloc[elem]
            corrected_timestamps.append(val - start_time)
        else:
            val = exp_removed_df['end_time'].iloc[elem]
            corrected_timestamps.append(val - start_time)

    '''plt.plot(corrected_timestamps)
    plt.show()
    exit("end")'''


    #timesteps =
    total_second =  (int(exp_removed_df['end_time'].iloc[len(exp_removed_df)-1]) - int(exp_removed_df['end_time'].iloc[0]))
    print(f"Total simulation time: {total_second} seconds | {round(total_second/60)} mins")

    number_of_working_robots = [[],[],[]] # one sub array per item type
    max_length = len(exp_removed_df['diff_time'])-1
    timesteps = exp_removed_df['diff_time']
    timesteps = np.arange(exp_removed_df['diff_time'].iloc[0],exp_removed_df['diff_time'].iloc[max_length]+exp_removed_df['real_time_taken'].iloc[max_length]+OFFSET)


    print(len(timesteps),len(exp_removed_df['end_time']))

    #timesteps = np.arange(int(exp_removed_df['start_time'].iloc[0]),int(exp_removed_df['end_time'].iloc[len(exp_removed_df) - 1]))


    for timestep in timesteps:
        counter =  [0,0,0]
        for key in robot_dic.keys():
            for working_time in robot_dic[key]:
                if(working_time[0] <= timestep <= working_time[1]):
                    counter[working_time[2]] += 1

        for sub_array in range(len(number_of_working_robots)):
            number_of_working_robots[sub_array].append(counter[sub_array])



    x_axis = []
    count = 0
    for z in range(start_time,start_time+total_second):
        x_axis.append(())

    #axs[1].yticks(np.arange(0, 6))
    #plt.xticks(x_axis)

    for sub_array in range(len(number_of_working_robots)):
        #print(number_of_working_robots[sub_array])
        axs[1].plot(timesteps,number_of_working_robots[sub_array], label=sub_array,color=COLOR[sub_array])

    #axs[1].title(f'{name_lists[0][exp_id]} - Number of working robots across time')


    axs[1].legend()
    axs[1].grid()

    plt.show()


plot_two_exp(exp_id=2, name="spike")

