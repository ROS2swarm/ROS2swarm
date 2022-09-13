import os
import pandas as pd




def get_data_per_exp(exp_id=0,max_length=0):
    prev_path = os.getcwd()
    os.chdir('..')

    exp_pattern = ['AS','AH','RZ']
    list_experiment_items = []
    list_experiment_removed = []

    name_list_item = []
    name_list_removed = []

    for file in os.listdir():

        if('results' in file):

            for result in os.listdir(f"{file}/"):
                df = pd.read_csv(f'{file}/{result}')
                #print(f"Read: {result}")

                if('removed' in result):
                    list_experiment_removed.append(df)
                    name_list_removed.append(result)

                else:
                    list_experiment_items.append(df)
                    name_list_item.append(result)
                '''sub_exp = []
                for col in df.columns:
                    sub_exp.append(df[col].values)
                '''

    os.chdir(prev_path)
    return list_experiment_items,list_experiment_removed,[name_list_item,name_list_removed]




get_data_per_exp(exp_id=0,max_length=0)






