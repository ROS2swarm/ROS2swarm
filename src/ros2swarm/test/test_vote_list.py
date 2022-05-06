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

import unittest

from communication_interfaces.msg import OpinionMessage, OpinionMACMessage

from src.ros2swarm.ros2swarm.utils.vote_list import \
    VoteList


class TestVoteList(unittest.TestCase):
    mes1 = None
    mes2 = None
    mes1_opinion2 = None
    macMessage1 = None
    macMessage2 = None
    empty_list = []

    def setUp(self):

        self.mes1 = OpinionMessage()
        self.mes1.id = 1
        self.mes1.opinion = 1

        self.mes2 = OpinionMessage()
        self.mes2.id = 2
        self.mes2.opinion = 2

        self.mes1_opinion2 = OpinionMessage()
        self.mes1_opinion2.id = 1
        self.mes1_opinion2.opinion = 1

        self.macMessage1 = OpinionMACMessage()
        self.macMessage1.id = 1
        self.macMessage1.opinion = 1
        self.macMessage1.mac = "I am a dummy mac"

        self.macMessage2 = OpinionMACMessage()
        self.macMessage2.id = 1
        self.macMessage2.opinion = 1
        self.macMessage2.mac = "I am another"

        self.empty_list = []

    def test_add_own_opinion(self):
        result = VoteList.update_opinion(self.empty_list, self.mes1, self.mes1.id)
        self.assertTrue(len(result) == 0)

    def test_add_opinion(self):
        result = VoteList.update_opinion(self.empty_list, self.mes1, 0)
        self.assertTrue(len(result) == 1)
        self.assertTrue(result[0].id == self.mes1.id)
        self.assertTrue(result[0].opinion == self.mes1.opinion)

    def test_add_second_message(self):
        tmp = VoteList.update_opinion(self.empty_list, self.mes1, 0)
        result = VoteList.update_opinion(tmp, self.mes2, 0)
        self.assertTrue(len(result) == 2)
        self.assertTrue(result[0].id == self.mes1.id)
        self.assertTrue(result[0].opinion == self.mes1.opinion)
        self.assertTrue(result[1].id == self.mes2.id)
        self.assertTrue(result[1].opinion == self.mes2.opinion)

    def test_update_opinion(self):
        tmp = VoteList.update_opinion(self.empty_list, self.mes1, 0)
        result = VoteList.update_opinion(tmp, self.mes1_opinion2, 0)
        self.assertTrue(len(result) == 1)
        self.assertTrue(result[0].id == self.mes1_opinion2.id)
        self.assertTrue(result[0].opinion == self.mes1_opinion2.opinion)

    def test_mac_update(self):
        tmp = VoteList.update_opinion(self.empty_list, self.macMessage1, 0)
        result = VoteList.update_opinion(tmp, self.macMessage2, 0)
        self.assertTrue(len(result) == 1)
        self.assertTrue(result[0].id == self.macMessage2.id)
        self.assertTrue(result[0].opinion == self.macMessage2.opinion)
        self.assertTrue(result[0].mac == self.macMessage2.mac)

    def test_multiple_call_does_not_change_list(self):
        result = VoteList.update_opinion(self.empty_list, self.mes1, 0)
        self.assertTrue(len(result) == 1)
        self.assertTrue(result[0].id == self.mes1.id)
        self.assertTrue(result[0].opinion == self.mes1.opinion)
        result = VoteList.update_opinion(result, self.mes1, 0)
        self.assertTrue(len(result) == 1)
        self.assertTrue(result[0].id == self.mes1.id)
        self.assertTrue(result[0].opinion == self.mes1.opinion)
        result = VoteList.update_opinion(result, self.mes1, 0)
        self.assertTrue(len(result) == 1)
        self.assertTrue(result[0].id == self.mes1.id)
        self.assertTrue(result[0].opinion == self.mes1.opinion)
