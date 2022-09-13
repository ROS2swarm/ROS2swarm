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
import rclpy
from rclpy.node import Node

from ros2swarm.utils.items_container import ItemsContainer
from communication_interfaces.msg import IntListMessage
from communication_interfaces.srv import ItemService

import csv
import string

class ItemsMaster(Node):
    def __init__(self):
        super().__init__('items_master')

        self.items_types = 3
        self.add_periodically = False
        self.get_logger().info('Initializing list containing %i items' % self.items_types)
        self.container = ItemsContainer(self.items_types)
        self.get_logger().info('List initialized successfully')

        self.filepath_log = "log_items.csv"
        items_string_list = []
        for item_type in range(self.items_types):
            items_string_list.append('items_' + string.ascii_lowercase[item_type])
        items_string_list.append('time')
        with open(self.filepath_log, 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            # write the header
            writer.writerow(items_string_list)

        self.publisher_ = self.create_publisher(IntListMessage, '/items_list', 10)
        self.timer_publish = self.create_timer(0.5, self.timer_publish_callback) #publishing the list of items on the topic every 0.5 seconds

        self.srv_ = self.create_service(ItemService, '/item_service', self.item_service_callback) #service used by robots to request to take an item: if the service return False, the item is not available

        self.timer_add_items = self.create_timer(10, self.timer_add_items_callback) #every minute (check the frequency at wich items are added or adapt the number of robots)

        self.timer_add_once = self.create_timer(900, self.add_items_once_callback)

    def timer_publish_callback(self):
        msg = IntListMessage()
        msg.data = self.container.get_items_list()
        self.publisher_.publish(msg)
        #writing items log data
        with open(self.filepath_log, 'a', encoding='UTF8') as f:
            writer = csv.writer(f)
            # write the data
            stringList = []
            for int in msg.data:
                stringList.append(str(int))
            time_string = str(self.get_clock().now().to_msg()).split('sec=')[1]
            time_string = time_string.split(',')[0]
            stringList.append(time_string)
            writer.writerow(stringList)


    def item_service_callback(self, request, response):
        response.success = self.container.take_item(request.item_index)
        return response

    def timer_add_items_callback(self):
        if(self.add_periodically):
            self.container.add_items_randomly()

    def add_items_once_callback(self):
        if(False):
            self.container.add_items(0, 10)
            self.timer_add_once.cancel()


def main(args=None):
    rclpy.init(args=args)

    items_master = ItemsMaster()

    rclpy.spin(items_master)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    items_master.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
