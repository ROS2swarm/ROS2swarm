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
import subprocess


class WifiFunctions:
    """Provides function for calculations around wifi."""

    @staticmethod
    def get_own_mac_address(wifi_interface_name):
        """
        Return the own mac address for the given interface.

        correspond to terminal: ifconfig ad0 | grep ether | grep -o ..:..:..:..:..:..
        """
        ad0 = subprocess.run(['ifconfig', wifi_interface_name], stdout=subprocess.PIPE).stdout
        grep1 = subprocess.run(['grep', 'ether'], stdout=subprocess.PIPE, input=ad0).stdout
        grep2 = subprocess.run(['grep', '-o', '..:..:..:..:..:..'], stdout=subprocess.PIPE,
                               input=grep1).stdout.decode('utf-8')
        mac_address = grep2
        return mac_address

    @staticmethod
    def get_all_stations_and_strength(wifi_interface_name):
        """Return all stations in the ad-hoc network and their signal strength in dBm."""
        # iw dev ad0 station dump | grep -E 'Station|signal:' |
        # sed -E 's/Station |\(on ad0\)|signal:|\t| dBm| //g'
        stations = subprocess.run(['iw', 'dev', wifi_interface_name, 'station', 'dump'],
                                  stdout=subprocess.PIPE).stdout
        grep1 = subprocess.run(['grep', '-E', "Station|signal:"], stdout=subprocess.PIPE,
                               input=stations).stdout
        # r means r input string
        pattern = r"s/Station |\(on " + wifi_interface_name + r"\)|signal:|\t| dBm| //g"
        sed = subprocess.run(['sed', '-E', pattern],
                             stdout=subprocess.PIPE,
                             input=grep1).stdout.decode('utf-8')
        # sed content has this structure:
        # 74:da:38:eb:4f:0e\n
        # -66\n
        # 74:da:38:e4:5a:55\n
        # -52\n

        lines = sed.splitlines()
        station_to_dBm = []
        for station, dBm in zip(lines[0::2], lines[1::2]):
            station_to_dBm.append((station, dBm))

        return station_to_dBm
