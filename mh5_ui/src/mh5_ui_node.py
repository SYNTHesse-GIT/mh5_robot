#!/usr/bin/env python3

# import subprocess
# import time

import rospy
import actionlib

# from sensor_msgs.msg import BatteryState, JointState, Temperature
# from diagnostic_msgs.msg import DiagnosticArray

# from mh5_controller.srv import ChangeTorque
# from mh5_director.msg import RunScriptAction, RunScriptGoal, RunScriptFeedback

from snack import Grid, GridForm, Label, Listbox, Scale, SnackScreen, \
                  Textbox, ButtonChoiceWindow

from main_ui import MainUI
from joint_view import JointView
from status_view import RobotStatusView
from comms_view import CommStatusView
from view_ui import View


# class CommStatusView(View):

#     def __init__(self, screen, timer, title='Comm Status'):
#         super().__init__(screen, timer, title)
#         self.stats = {}
#         self.mode = 't'

#     def comms_call_back(self, msg):
#         for status in msg.status:
#             name = status.name
#             # perc = 0.0
#             # packs = 0
#             # for value in status.values:
#             #     if value.key == 'cum_error_rate_perc':
#             #         perc = float(value.value)
#             #     if value.key == 'cum_packets':
#             #         packs = int(value.value)/1000
#             #     if value.key == 'packets':
#             #         last_packs = int(value.value)
#             #     if value.key == 'errors':
#             #         last_errors = int(value.value)
#             #     if value.key == 'real_rate':
#             #         real_rate = float(value.value)
#             # self.stats[name] = {
#             #     'perc': perc,
#             #     'packs': packs,
#             #     'last_packs': last_packs,
#             #     'last_errors': last_errors,
#             #     'real_rate': 
#             # }
#             keys = [v.key for v in status.values]
#             values = [v.value for v in status.values]
#             self.stats[name] = dict(zip(keys, values))

#     def create_content(self):
#         lb = Listbox(height=18, width=36)
#         lb.append('Name                  Packs[k] %Err ', 0)
#         for pos in range(1, 18):
#             lb.append('', pos)
#         self.comm_subsr = rospy.Subscriber('communication_statistics', DiagnosticArray, self.comms_call_back)
#         return lb

#     def update_content(self):
#         if self.mode == 't':
#             self.content.replace('Name                  Packs[k] %Err ', 0)
#             for index, name in enumerate(self.stats):
#                 stats = self.stats[name]
#                 packs = float(stats.get('cum_packets', '0'))/1000.0
#                 perc = float(stats.get('cum_error_rate_perc', '0'))
#                 self.content.replace(f'{name[:22]:22s} {packs:7.1f} {perc:4.1f}%', index + 1)
#         elif self.mode == 'l':
#             self.content.replace('Name                     Packs Errs', 0)
#             for index, name in enumerate(self.stats):
#                 stats = self.stats[name]
#                 packs = int(stats.get('packets', '0'))
#                 errors = int(stats.get('errors', '0'))
#                 self.content.replace(f'{name[:22]:22s} {packs:7d} {errors:4d}', index + 1)
#         elif self.mode == 'r':
#             self.content.replace('Name                      Act Rate', 0)
#             for index, name in enumerate(self.stats):
#                 stats = self.stats[name]
#                 rate = float(stats.get('real_rate', '0'))
#                 self.content.replace(f'{name[:22]:25s} {rate:7.2f}Hz', index + 1)

#     @property
#     def hotkeys(self):
#         return ['t', 'l', 'r']

#     def process_hotkey(self, key):
#         if key in self.hotkeys:
#             self.mode = key

#     def finish(self):
#         # self.comm_subsr.unregister()
#         del self.comm_subsr


class Menu(View):

    def __init__(self, screen, title):
        super().__init__(screen, 0, title)
        self.navigation = []
        self.current = 'main'
        self.redraw = False
        self.change_torque = rospy.ServiceProxy('change_torque', ChangeTorque)
        self.director = actionlib.SimpleActionClient('director', RunScriptAction)
        self.menus = {
            'main': [
                ('System', self.navigate, ('system',)),
                ('Robot', self.navigate, ('robot',)),
                ('Actions', self.navigate, ('actions',))
            ],
            'system': [
                ('Close ROS', self.close_ros, ()),
                ('Shutdown robot', self.shutdown_robot, ())
            ],
            'robot': [
                ('Torque enable', self.navigate, ('torque_enable',)),
                ('Torque disable', self.navigate, ('torque_disable',))
            ],
            'torque_enable': [
                ('Torque enable head', self.do_change_torque, (True, 'head')),
                ('Torque enable left arm', self.do_change_torque, (True, 'left_arm')),
                ('Torque enable right arm', self.do_change_torque, (True, 'right_arm')),
                ('Torque enable arms', self.do_change_torque, (True, 'arms')),
                ('Torque enable left leg', self.do_change_torque, (True, 'left_leg')),
                ('Torque enable right leg', self.do_change_torque, (True, 'right_leg')),
                ('Torque enable legs', self.do_change_torque, (True, 'legs')),
                ('Torque enable all', self.do_change_torque, (True, 'all'))
            ],
            'torque_disable': [
                ('Torque disable head', self.do_change_torque, (False, 'head')),
                ('Torque disable left arm', self.do_change_torque, (False, 'left_arm')),
                ('Torque disable right arm', self.do_change_torque, (False, 'right_arm')),
                ('Torque disable arms', self.do_change_torque, (False, 'arms')),
                ('Torque disable left leg', self.do_change_torque, (False, 'left_leg')),
                ('Torque disable right leg', self.do_change_torque, (False, 'right_leg')),
                ('Torque disable legs', self.do_change_torque, (False, 'legs')),
                ('Torque disable all', self.do_change_torque, (False, 'all'))
            ],
            'actions': [
                ('Stand up', self.action, ('stand')),
                ('Sit down', self.action_sit_down, ())
            ]
        }

    def create_content(self):
        lb = Listbox(height=18, width=24)
        for index, (menu_item, _, _) in enumerate(self.menus[self.current]):
            lb.append(menu_item, index)
        return lb

    def update_content(self):
        if self.redraw:
            self.content.clear()
            for index, (menu_item, _, _) in enumerate(self.menus[self.current]):
                self.content.append(menu_item, index)
            self.redraw = False

    @property
    def hotkeys(self):
        return ['ESC', 'ENTER']

    def process_hotkey(self, key):
        if key == 'ENTER':
            sel = self.content.current()
            func = self.menus[self.current][sel][1]
            args = self.menus[self.current][sel][2]
            func(*args)
            return None
        if key == 'ESC':
            if self.navigation:
                menu = self.navigation.pop()
                self.current = menu
                self.redraw = True
            return None

    def navigate(self, menu):
        self.navigation.append(self.current)
        self.current = menu
        self.redraw = True

    def close_ros(self):
        pass

    def shutdown_robot(self):
        pass

    def torque_enable(self):
        pass

    def torque_disable(self):
        pass

    def action(self, script):

        def feedback_cb(feedback):
            pass

        command = RunScriptGoal(script)
        
        self.director.send_goal(command, feedback_cb=feedback_cb)

        client.wait_for_result()


    def action_sit_down(self):
        pass

    def do_change_torque(self, state, group):
        result = self.change_torque(state, [], [group])
        not_ok = [joint_name for joint_name, error in zip(result.joints, result.results) if error != 0]
        if not_ok:
            text = f'Error: {not_ok}'
        else:
            text= 'All OK'
        ButtonChoiceWindow(
            screen=self.screen,
            title='Result',
            text=text,
            width=30,
            buttons=['Dismiss']
        )

    def finish(self):
        # reset the navigation for next time
        self.navigation = []
        self.current = 'main'


if __name__ == '__main__':

    rospy.init_node('mh5_edge_ui')

    ui = MainUI()
    ui.add_view(RobotStatusView(ui.screen, 1000, 'MH5 Status'), 's', default_view=True)
    ui.add_view(JointView(ui.screen, 100, 'Joint Status'), 'j')
    ui.add_view(CommStatusView(ui.screen, 100, 'Comm Status'), 'c')
    # ui.add_view(Menu(ui.screen, 'MH5 Main Menu'), 'm')
    ui.screen.pushHelpLine('[s]tatus [j]oints [c]omms [m]enu')
    ui.run()
