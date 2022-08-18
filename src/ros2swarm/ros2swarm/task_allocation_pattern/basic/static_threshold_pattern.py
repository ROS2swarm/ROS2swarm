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
from ros2swarm.abstract_pattern import AbstractPattern
from communication_interfaces.msg import IntListMessage
from communication_interfaces.srv import ItemService
from ros2swarm.utils import setup_node


class StaticThresholdPattern(AbstractPattern):
    """
    Implementation of the Static Threshold approach.

    """

    def __init__(self):
        """Initialize the static threshold pattern node."""
        super().__init__('static_threshold_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                # ('parameter_name', None),
            ])

        # self.parameter_name = self.get_parameter(
        #     "parameter_name").get_parameter_value().integer_value

        self.has_item = False
        self.item_type_hold = None

        self.item_type_to_take = 0

        self.cli_item_service = self.create_client(ItemService, '/item_service')

        while not self.cli_item_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_item_service = ItemService.Request()

        #taking items from items_master
        self.take_item()
        self.item_type_to_take = 1
        self.take_item()
        self.item_type_to_take = 2
        self.take_item()



    def send_request(self, item_index):
        self.req_item_service.item_index = item_index
        self.future = self.cli_item_service.call_async(self.req_item_service)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def take_item(self):
        #TODO need to add a big if to check if we are in the zone where we can take an object
        response = self.send_request(self.item_type_to_take)
        if(response):
            self.has_item = True
            self.item_type_hold = self.item_type_to_take
            self.get_logger().info('Got item %s' % self.item_type_to_take)
        else:
            self.has_item = False
            self.item_type_hold = None
            self.get_logger().info('Did not get item %s' % self.item_type_to_take)

    def destroy_node(self):
        """Call the super destroy method."""
        super().destroy_node()

def main(args=None):
    """Create a node for the static threshold pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, StaticThresholdPattern)


if __name__ == '__main__':
    main()
