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

from communication_interfaces.msg import OpinionMACMessage


class VoteList:

    @staticmethod
    def update_opinion(opinion_list: list, opinion_msg, robot_id: int):
        """
        Adds the opinion message to the opinion list, if the ID is not the robot ID
        and not already contained.
        Updates the opinion of a stored message if a message with the same ID is already contained.

        :param opinion_list The list of opinions to update
        :param opinion_msg The message to add/update the list
        :param robot_id The id of the message receiving robot

        :return: The updated opinion list
        """
        if opinion_msg.id != robot_id:
            contained_flag = False
            for i in opinion_list:
                if i.id == opinion_msg.id:
                    # default: OpinionMessage
                    i.opinion = opinion_msg.opinion
                    # additional: OpinionMACMessage
                    if isinstance(opinion_msg, OpinionMACMessage):
                        i.mac = opinion_msg.mac

                    contained_flag = True
                    break

            if not contained_flag:
                opinion_list.append(opinion_msg)
        return opinion_list
