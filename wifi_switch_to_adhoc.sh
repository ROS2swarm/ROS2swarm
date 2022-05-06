#!/bin/bash
#    Copyright 2020 Marian Begemann
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
echo starting configuration of the ad-hoc network &&
# phy1 or phy0 is random assigned at startup
var_phy=$(iw list | grep -o -E 'Wiphy (.*)|SSIDs: 4' | sed ':a;N;$!ba;s/\n/ /g' | grep -o -E 'phy0 SS|phy1 SS' | grep -o -E 'phy.') &&
sudo ifconfig wlan0 down &&
sudo iw $var_phy interface add ad0 type ibss &&
sudo ifconfig ad0 up &&
sudo iw ad0 ibss join turtlenet 2412 &&
echo ad-hoc network configured