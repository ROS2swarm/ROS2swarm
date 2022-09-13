import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from utils import get_data_per_exp

COLORS = {'a':'green','b':'red','c':'blue'}
COLOR = ['green','red','blue']

def plot_single_exp(exp_id = 0, plot_id = 0):

    df_item_list, df_item_removed, name_lists = get_data_per_exp()

    exp_item_df = df_item_list[exp_id]
    exp_removed_df = df_item_removed[exp_id]

    if(plot_id == 0):
        for col in exp_item_df.columns:
            item_name = col.split('_')[1]
            plt.plot(exp_item_df[col].values,label=f"{col}",color=COLORS[item_name])
        plt.title(f'{name_lists[0][exp_id]} - Number of remaining items by type across time')

    if(plot_id == 1):
        robot_dic = {}

        for i in range(len(exp_removed_df['robot_name'])):
            robot_name = exp_removed_df['robot_name'].iloc[i]

            if(not robot_name in robot_dic.keys()):
                robot_dic[robot_name] = []

            robot_dic[robot_name].append([int(exp_removed_df['start_time'].iloc[i]),int(exp_removed_df['start_time'].iloc[i]) + int(exp_removed_df['time taken'].iloc[i]),int(exp_removed_df['item_picked'].iloc[i])])

        timesteps = np.arange(int(exp_removed_df['start_time'].iloc[0]),int(exp_removed_df['end_time'].iloc[len(exp_removed_df)-1]))
        #print(timesteps)
        number_of_working_robots = [[],[],[]] # one sub array per item type

        for timestep in timesteps:
            counter =  [0,0,0]
            for key in robot_dic.keys():
                for working_time in robot_dic[key]:
                    if(working_time[0] <= timestep <= working_time[1]):
                        counter[working_time[2]] += 1

            for sub_array in range(len(number_of_working_robots)):
                number_of_working_robots[sub_array].append(counter[sub_array])

        plt.yticks(np.arange(0,6))
        for sub_array in range(len(number_of_working_robots)):

            plt.plot(timesteps, number_of_working_robots[sub_array], label=sub_array,color=COLOR[sub_array])

        plt.title(f'{name_lists[0][exp_id]} - Number of working robots across time')


    plt.legend()
    plt.grid()

    plt.show()


plot_single_exp(plot_id=1)

