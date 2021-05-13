
from sensor_msgs.msg import BatteryState, JointState, Temperature
from snack import Listbox
import rospy


# Grid, GridForm, Label, , Scale, , \
#   SnackScreen                Textbox, ButtonChoiceWindow
from view_ui import View

class JointView(View):

    def __init__(self, screen, timer, title='Joint State'):
        super().__init__(screen, timer, title)

        self.joint_names = [
            'head_p', 'head_y',
            'r_sho_p', 'r_sho_r', 'r_elb_y', 'r_elb_p',
            'r_hip_r', 'r_hip_p', 'r_kne_p', 'r_kne_y', 'r_ank_p', 'r_ank_r',
            'l_sho_p', 'l_sho_r', 'l_elb_y', 'l_elb_p',
            'l_hip_r', 'l_hip_p', 'l_kne_p', 'l_kne_y', 'l_ank_p', 'l_ank_r']
        self.joint_values = {}
        self.mode = 'r'         # radians

    def create_content(self):
        self.js_subsr = rospy.Subscriber('joint_states', JointState, self.joint_values_call_back)
        self.jt_subsr = rospy.Subscriber('temperature', Temperature, self.joint_temperature_call_back)
        self.jv_subsr = rospy.Subscriber('voltage', BatteryState, self.joint_voltage_call_back)

        lb = Listbox(height=22, width=36)
        for pos in range(22):
            lb.append(f'Text for position {pos:2d}', pos)
        return lb

    def update_content(self):
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
    def hotkeys(self):
        return ['d', 'r', 't']

    def process_hotkey(self, key):
        if key in self.hotkeys:
            self.mode = key

    def joint_values_call_back(self, msg):
        for index, name in enumerate(msg.name):
            if name not in self.joint_values:
                self.joint_values[name] = {}
            self.joint_values[name]['pos'] = msg.position[index]
            self.joint_values[name]['vel'] = msg.velocity[index]
            self.joint_values[name]['eff'] = msg.effort[index]

    def joint_temperature_call_back(self, msg):
        name = msg.header.frame_id
        if name not in self.joint_values:
            self.joint_values[name] = {}
        self.joint_values[name]['temp'] = msg.temperature

    def joint_voltage_call_back(self, msg):
        name = msg.header.frame_id
        if name not in self.joint_values:
            self.joint_values[name] = {}
        self.joint_values[name]['volt'] = msg.voltage

    def finish(self):
        # self.js_subsr.unregister()
        # self.jt_subsr.unregister()
        # self.jv_subsr.unregister()
        del self.js_subsr
        del self.jt_subsr
        del self.jv_subsr