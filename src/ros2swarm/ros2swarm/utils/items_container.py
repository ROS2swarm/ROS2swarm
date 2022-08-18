#    Copyright 2022 Antoine Sion
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
import numpy as np

class ItemsContainer:
    def __init__(self, items_types):

        self.items_list = [ 0 for i in range(items_types)]
        self.initialize_randomly(5)

    def initialize_randomly(self, upper_bound):
        for index in range(len(self.items_list)):
            self.items_list[index] = np.random.randint(1,upper_bound+1)

    def add_items(self, index, quantity):
        self.items_list[index] += quantity

    def take_item(self, index):
        if(self.items_list[index] > 0):
            self.items_list[index] -= 1
            return True
        else:
            return False

    def get_items_list(self):
        return self.items_list

    def add_items_randomly(self):
        type_to_add = np.random.randint(0,len(self.items_list))
        number_to_add = np.random.randint(1,6)
        self.add_items(type_to_add, number_to_add)
