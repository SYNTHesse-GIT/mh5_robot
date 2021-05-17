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
import subprocess
import rospy
import actionlib
from snack import Listbox, ButtonChoiceWindow, SnackScreen

from view_ui import View
from mh5_msgs.srv import ActivateJoint

class MenuView(View):
    """View that implements an interactive menu. Users can move up and down
    in a menu list, select using the [Enter] key one item or return to the
    previous menu with [Esc]. The selected menu item will trigger the
    execution of a method from :class:`MenuView` with optional parameters.

    The defintion of the menu should be provided from the Paramter Server,
    for instance by using an ``.yaml`` file that is loaded in the launch
    file using (for instance):

        <rosparam file="$(find mh5_ui)/config/menu.yaml" command="load"/>

    The structure of this should be:

        mh5_ui:

            menu:
            
                main: [
                { prompt: System, method: do_navigate, params: [system] },
                { prompt: Robot, method: do_navigate, params: [robot] },
                { prompt: Actions, method: do_navigate, params: [actions] }
                ]

                system: [
                { prompt: Close ROS, method: do_close_ros, params: [] },
                { prompt: Shutdown robot, method: do_shutdown, params: [] }
                ]

    The first two levels (mh5_ui/menu) are expected by the code and should be
    used as such. Each menu is a dictionary where the key is the menu itentifier
    and the values are a list of menu items. Each menu item is a dictionary
    that contains the ``prompt`` (what is printed in the menu), the ``method``
    (what method is to be executed if item is selected) and ``params`` (a list
    of parameters to be passed to the method).

    .. note:: To contain eventual misuses of the class, only methods in the
        :class:`MenuView` can be executed and only if the start with "do_".
        This should prohibit execution (based on YAML file specificatiton) of
        arbitrary methods.
    """
    navigation: List[str]
    """Breadcrumb with menu navigation."""
    current: str
    """Current menu."""
    redraw: bool
    """Indicates that the menu needs to be redrawn."""
    menus: Dict[str, List[Dict]]
    """The menu structure as read from YAML file as described in the class
    description."""

    def __init__(self, screen: SnackScreen, title: str) -> None:
        """Initializes the MenuView.

        Sets the ``navigation`` breadcrumb to ``[]`` and the ``current`` menu to
        'main' (hardcoded so this needs to be in the definition).

        Parameters
        ----------
        screen : SnackScreen
            The ``snack.SnackScreen`` where the view will be shown.
        title : str
            The title to use for the view.
        """
        super().__init__(screen, 0, title)
        self.navigation = []
        self.current = 'main'
        self.redraw = False
        self.menus = rospy.get_param('mh5_ui/menu', {})
        srv = rospy.get_param('~torque_service', 'torque_control/switch_torque')
        self.torque_srv = rospy.ServiceProxy(srv, ActivateJoint)
        srv = rospy.get_param('~reboot_service', 'torque_control/reboot')
        self.reboot_srv = rospy.ServiceProxy(srv, ActivateJoint)
        # self.director = actionlib.SimpleActionClient('director', RunScriptAction)

    def create_content(self) -> None:
        """Constructs the elements of the menu. Uses a ``snack.ListBox`` to
        list the menu elements.

        Returns
        -------
        ``snack.Listbox``
            The content of the menu as a ``snack.Listbox``
        """
        lb = Listbox(height=18, width=24)
        for index, menu_item in enumerate(self.menus[self.current]):
            prompt = menu_item.get('prompt', f'<item {index+1}>')
            lb.append(prompt, index)
        return lb

    def update_content(self) -> None:
        """Updates the view after a navigation. To avoid unnecessary updates
        it will check ``redraw`` attribute and will preform the content
        changes only if this is ``True``.
        """
        if self.redraw:
            self.content.clear()
            for index, menu_item in enumerate(self.menus[self.current]):
                prompt = menu_item.get('prompt', f'<item {index+1}>')
                self.content.append(prompt, index)
            self.redraw = False

    @property
    def hotkeys(self) -> List[str]:
        """Returns the hotkeys handled by the view.

        Returns
        -------
        List(str)
            Returns the keys ``ESC`` and ``ENTER``.
        """
        return ['ESC', 'ENTER']

    def process_hotkey(self, key: str) -> None:
        """Handles the ``Esc`` and ``Enter`` hotkeys.

        ``Esc`` will pop the last item from the ``navigation`` and ask to change
        to that menu.

        ``Enter`` will check that there is a method assigned in the menu
        specification, that this method starts with "do_" (security precaution
        to avoid running arbitrary methods) and that is a callable attribute of
        :class:`ManuView`. If all this is respected, it will call that
        method with the eventual paramters taken from the menu defintion.

        Parameters
        ----------
        key : str
            The key to process. It's ``ESC`` or ``ENTER``.
        """
        if key == 'ENTER':
            sel = self.content.current()
            func = self.menus[self.current][sel].get('method', None)

            if func == None:
            # no method defined
                ButtonChoiceWindow(screen=self.screen, title='Error',
                    text='No method defined for this choice',
                    width=30,buttons=['Dismiss'])
                return

            if func[:3] != "do_":
            # method does not start with do_; security measure to avoid
            # running arbitrary code
                ButtonChoiceWindow(screen=self.screen, title='Error',
                    text=f'Only "do_..." methods can be executed',
                    width=30,buttons=['Dismiss'])
                return

            if not hasattr(self, func):
            # the method does not exist
                ButtonChoiceWindow(screen=self.screen, title='Error',
                    text=f'Method "{func}" does not exist',
                    width=30,buttons=['Dismiss'])
                return

            do_op = getattr(self, func)

            if not callable(do_op):
            # is not a method
                ButtonChoiceWindow(screen=self.screen, title='Error',
                    text=f'"{func}" is not a callable method',
                    width=30,buttons=['Dismiss'])
                return

            # all ok; run the method
            args = self.menus[self.current][sel].get('params', [])
            do_op(*args)
            return

        if key == 'ESC':
            if self.navigation:
                menu = self.navigation.pop()
                self.current = menu
                self.redraw = True


    def do_navigate(self, menu:str) -> None:
        """Navigates to the the requested new menu. If the requested menu
        is not avaialble in the ``menus`` an error message is shown. Before
        navigating the current menu is added to the ``navigation`` breadcrumb.

        Parameters
        ----------
        menu : str
            The new menu to navigate to.
        """
        if menu not in self.menus:
            ButtonChoiceWindow(screen=self.screen, title='Error',
                text=f'"Menu {menu}" does not exist',
                width=30,buttons=['Dismiss'])
            return

        self.navigation.append(self.current)
        self.current = menu
        self.redraw = True

    def close_ros(self):
        pass

    def do_shutdown(self) -> None:
        """Requests the system shutdown. Will be scheduled for 1 minute.
        """
        com = ['sudo', 'shutdown']
        res = subprocess.run(com, text=True, stderr=subprocess.STDOUT)
        #, shell=True, stdout=subprocess.PIPE,
        #                      encoding='utf-8'
        ButtonChoiceWindow(screen=self.screen, title='Result',
                text=f'result: {res.returncode}\nstdout: {res.stdout}',
                width=30, buttons=['Dismiss'])

    def do_action(self, script):
        pass

    def do_change_torque(self, state: bool, group: str) -> None:
        """Calls the "torque_control/switch_torque" service for a
        group of servos.

        Parameters
        ----------
        state : bool
            Desired state (``True`` means activate, ``False`` means deactivate).
        group : str
            Name of group to change state.
        """
        try:
            res = self.torque_srv(group, state)
            ButtonChoiceWindow(screen=self.screen, title='Result',
                text=f'Success: {res.success}\nMessage: {res.message}',
                width=30, buttons=['Dismiss'])
        except rospy.ServiceException as e:
            ButtonChoiceWindow(screen=self.screen, title='Error',
                text=f'Service Exception: {e}',
                width=30, buttons=['Dismiss'])

    def do_reboot(self, state: bool, group: str) -> None:
        """Calls the "torque_control/reboot" service for a
        group of servos.

        Parameters
        ----------
        state : bool
            Desired state (``True`` means reboot).
        group : str
            Name of group to reboot.
        """
        try:
            res = self.reboot_srv(group, state)
            ButtonChoiceWindow(screen=self.screen, title='Result',
                text=f'Success: {res.success}\nMessage: {res.message}',
                width=30, buttons=['Dismiss'])
        except rospy.ServiceException as e:
            ButtonChoiceWindow(screen=self.screen, title='Error',
                text=f'Service Exception: {e}',
                width=30, buttons=['Dismiss'])

    def finish(self):
        """Resets navigation before switching the view.
        """
        # reset the navigation for next time
        self.navigation = []
        self.current = 'main'