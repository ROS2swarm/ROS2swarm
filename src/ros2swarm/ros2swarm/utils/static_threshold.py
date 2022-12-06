#    Copyright 2021 Marian Begemann
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import random
import numpy as np

class StaticThreshold:

    @staticmethod
    def choose_task(task_list: list, n, threshold_list: list):
        """
        Choose a task based on the task demand

        :param task_list The list of tasks, each number in the list represent the task demand (stimulus, e.g. number of items to take) and the size of the list is the total number of different tasks
        :param n Parameter setting the responsiveness of the stimulus
        :param threshold_list List of the thresholds for each type of task

        :return: the index of the chosen task or -1 if no task was chosen
        """

        # calculate the probabilities for the stimulus (the normalized demand of each task)
        # this goes in to the function T
        task_demand = []
        prob = []
        choose = []
        for i in range(len(task_list)):
            task_demand.append(task_list[i] / sum(task_list)) # the stimulus
            prob.append(StaticThreshold.responseT(task_demand[i], n, threshold_list, i)) #get values from response function
            choose.append(i) #append to list to randomly choose which one to probabilistically check first

        task_chosen = 0

        while (len(choose) > 0 and task_chosen == 0): #choose one probability randomly and remove tasks probabilistically
            chosen = random.choice(choose)
            choose.remove(chosen)
            rand = random.random()
            if (rand < prob[chosen]): # compare the chosen task probability with a random number 0-1 to remove a task of that type
                task_chosen = 1
                return task_list.index(task_list[chosen])

        return -1

    @staticmethod
    def responseT(stimIntensity, n, threshold_list: list, task_type): #the response function takes stimulus (task demand) and calculates response value
        output = stimIntensity ** n / (stimIntensity ** n + threshold_list[task_type])
        return output
