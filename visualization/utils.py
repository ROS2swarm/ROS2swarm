import os
import pandas as pd




def get_data_per_exp(exp_id,max_length):
    prev_path = os.getcwd()
    os.chdir('..')

    exp_pattern = ['AZ','AH','RZ']
    list_experiment_items = []
    list_experiment_removed = []


    for file in os.listdir():

        if('results' in file):

            for result in os.listdir(f"{file}/"):
                df = pd.read_csv(f'{file}/{result}')
                print(f"Read: {result}")

                if('removed' in result):
                    list_experiment_removed.append(df)

                else:
                    list_experiment_items.append(df)
                '''sub_exp = []
                for col in df.columns:
                    sub_exp.append(df[col].values)
                '''



    os.chdir(prev_path)
    return list_experiment_items,list_experiment_removed




get_data_per_exp(exp_id=0,max_length=0)






