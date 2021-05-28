# Copyright (C) 2020  Alex Sonea

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from typing import List, Dict
import rospy
from sensor_msgs.msg import JointState
from snack import Listbox, SnackScreen
from view_ui import View
from mh5_msgs.msg import DeviceStatus

class JointView(View):
    """View that displays the current state of the joints showing: position,
    velocity and effort as well as temperature and voltage. Position and 
    velocity can be shown in degrees or in radians.

    The view shows the joints in a ``snack.Listbox`` that contains one joint
    per row, followed by the information for that joint. The hot-keys "d", "r"
    and "t" are registered with the MainUI to handle the switch between the
    display modes: "d" displays the position, velocity in degrees and load in
    Nm, "r" displays position, velocity in radians and load in Nm while "t"
    displays the temperature in Celsius degrees and voltage in Volts.

    The view subscribes to:
    
    - ``joint_states`` to retreive joint position, velocity and load
    - ``temperature`` to retreive the joint temperature
    - ``voltage`` to retrieve the joint voltage

    When the view is dismissed (switched to another view) it removes the
    subscriptions to reduce the load on the ROS communication. 
    """
    joint_names: List[str]
    """Ordered list of joints. The information will be listed in the view
    in the same order that the joints are declared here."""
    joint_values: Dict[str, Dict[str, float]]
    """Dictionary that contains the values for each joint. The values are held
    as a dictionary with keys: "pos", "vel", "eff", "temp" and "volt". Values
    are updated by the callbacks from the subscribers and used by the 
    :meth:`update_content`."""
    mode: str
    """The display variant. Could be "r" for radian display, "d" degree display
    or "t" for temp/voltage display as explained in the class defintion."""
    js_subscr: rospy.Subscriber
    """Subscriber to ``joint_stateas``."""
    jtv_subscr: rospy.Subscriber
    """Subscriber to ``device_status``."""

    def __init__(self, screen: SnackScreen,
                       timer: int,
                       title: str = 'Joint State') -> None:
        """Initializes the screen.

        Calls the inherited constructor from :class:`view_ui.View`, then sets
        up the ``joint_names``. It also initializes the display mode to "r".

        Parameters
        ----------
        screen : SnackScreen
            The screen where the content will be bound.
        timer : int
            Refresh time in milliseconds for the content.
        title : str, optional
            Screen title, by default 'Joint State'
        """
        super().__init__(screen, timer, title)

        self.joint_names = [
            'head_p', 'head_y',
            'r_sho_p', 'r_sho_r', 'r_elb_y', 'r_elb_p',
            'r_hip_r', 'r_hip_p', 'r_kne_p', 'r_kne_y', 'r_ank_p', 'r_ank_r',
            'l_sho_p', 'l_sho_r', 'l_elb_y', 'l_elb_p',
            'l_hip_r', 'l_hip_p', 'l_kne_p', 'l_kne_y', 'l_ank_p', 'l_ank_r']
        self.joint_values = {}
        self.mode = 'r'         # radians

    def create_content(self) -> Listbox:
        """Prepares the content for the view, in this case a ``shack.Listbox``
        with 22 rows and 36 characts wide.

        The method also creates the subscribers to the topics that provide
        information for the view: ``joint_states`` for the position, velocity
        and load, ``temperature`` for the joint temperatures and ``voltage``
        for the joint voltages.

        Returns
        -------
        Listbox:
            The content of the view as a ``snack.Listbox``.
        """
        topic = rospy.get_param('~joint_states_topic', 'joint_states')
        self.js_subsr = rospy.Subscriber(topic, JointState, self.joint_values_call_back)
        self.jtv_subsr = rospy.Subscriber('device_status', DeviceStatus, self.joint_temp_volt_call_back)


        lb = Listbox(height=22, width=36)
        for pos in range(22):
            lb.append(f'Text for position {pos:2d}', pos)
        return lb

    def update_content(self) -> None:
        """Updates the view content from the information that is storred in 
        ``joint_values`` by each of the subscribers' callback functions.

        The update depends on the ``mode``.

        The method is called by the timer in the :class:`view_ui.View``.
        """
        for index, joint_name in enumerate(self.joint_names):
            values = self.joint_values.get(joint_name, {})

            if self.mode == 't':
                temp = values.get('temp', 0)
                volt = values.get('volt', 0)
                self.content.replace(f'{joint_name:12s}   {temp:5.1f}°C   {volt:5.1f}V', index)
                continue

            if self.mode == 'd':
                pos = values.get('pos', 0) * 57.29578
                vel = values.get('vel', 0) * 57.29578
                eff = values.get('eff', 0)
                self.content.replace(f'{joint_name:12s} {pos:5.1f}   {vel:5.1f}   {eff:5.1f}', index)
                continue

            if self.mode == 'r':
                pos = values.get('pos', 0)
                vel = values.get('vel', 0)
                eff = values.get('eff', 0)
                self.content.replace(f'{joint_name:12s} {pos:5.2f}   {vel:5.2f}   {eff:5.2f}', index)

    @property
    def hotkeys(self) -> List[str]:
        """Notifies the MainUI the hotkeys this view implements.

        Returns
        -------
        List[str]
            Returns ['d', 'r', 't']
        """
        return ['d', 'r', 't']

    def process_hotkey(self, key:str) -> None:
        """Handles a hotkey. Because the hotkeys handled have the same coding
        as the display mode, it simply stores the hotkey to the ``mode``
        attribute and later the :meth:`update_content` will reflect that
        request.

        Parameters
        ----------
        key : str
            The hotkey pressed by the user.
        """
        if key in self.hotkeys:
            self.mode = key

    def joint_values_call_back(self, msg: JointState) -> None:
        """Callback for handling ``joint_states`` subscription. Uses the
        values from the message by comparing the name of the joint
        and storing the data provided into "pos", "vel" and "eff" keys of the
        ``joint_values`` for that joint.

        Parameters
        ----------
        msg : JointState
            ROS message with the joint states.
        """
        for index, name in enumerate(msg.name):
            if name not in self.joint_values:
                self.joint_values[name] = {}
            self.joint_values[name]['pos'] = msg.position[index]
            self.joint_values[name]['vel'] = msg.velocity[index]
            self.joint_values[name]['eff'] = msg.effort[index]

    def joint_temp_volt_call_back(self, msg: DeviceStatus) -> None:
        """Callback for handling ``device_status`` subscription. Uses the
        values from the message by comparing the name of the joint
        and storing the data provided into "temperatures"  and "voltages" 
        keys for that joint.

        Parameters
        ----------
        msg : DeviceStatus
            ROS message with the joint temperatures.
        """
        for index, name in enumerate(msg.device_names):
            if name in self.joint_values:
                self.joint_values[name]['temp'] = msg.temperatures[index]
                self.joint_values[name]['volt'] = msg.voltages[index]

    # def joint_voltage_call_back(self, msg: BatteryState) -> None:
    #     """Callback for handling ``voltage`` subscription. Uses the
    #     values from the message by comparing the name of the joint
    #     and storing the data provided into "volt" key of the
    #     ``joint_values`` for that joint.

    #     Parameters
    #     ----------
    #     msg : BatteryState
    #         ROS message with the joint voltage.
    #     """
    #     name = msg.header.frame_id
    #     if name not in self.joint_values:
    #         self.joint_values[name] = {}
    #     self.joint_values[name]['volt'] = msg.voltage

    def finish(self) -> None:
        """Handles the request to switch away from the view. To preserve
        resource it deletes the subscriptions to the ``joint_states``,
        ``temperature`` and ``voltage`` topics.
        """
        # self.js_subsr.unregister()
        # self.jt_subsr.unregister()
        # self.jv_subsr.unregister()
        del self.js_subsr
        del self.jtv_subsr
