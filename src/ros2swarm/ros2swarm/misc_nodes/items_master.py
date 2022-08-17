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

class ItemsMaster(Node):

    def __init__(self):
        super().__init__('items_master')
        #TODO implement multiple services in this node
        #Service1 : giving the list of items to a robot if requested
        #Service2 : giving an item to a robot if he requests it
        #TODO implement an array for storing the items (struct = [10,5,6] if 10 items of type A, 5 items of type B, etc)
        #also implement methods for picking an item from the container or adding items


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
