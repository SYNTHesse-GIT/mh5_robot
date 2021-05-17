#!/usr/bin/env python3

import rospy

from main_ui import MainUI
from joint_view import JointView
from status_view import RobotStatusView
from comms_view import CommStatusView
from menu_view import MenuView


if __name__ == '__main__':

    rospy.init_node('mh5_edge_ui')

    ui = MainUI()
    ui.add_view(RobotStatusView(ui.screen, 1000, 'MH5 Status'), 's', default_view=True)
    ui.add_view(JointView(ui.screen, 100, 'Joint Status'), 'j')
    ui.add_view(CommStatusView(ui.screen, 100, 'Comm Status'), 'c')
    ui.add_view(MenuView(ui.screen, 'MH5 Main Menu'), 'm')
    ui.screen.pushHelpLine('[s]tatus [j]oints [c]omms [m]enu')
    ui.run()
