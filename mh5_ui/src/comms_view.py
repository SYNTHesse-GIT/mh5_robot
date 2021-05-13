from view_ui import View
from snack import Listbox
from diagnostic_msgs.msg import DiagnosticArray
import rospy

class CommStatusView(View):

    def __init__(self, screen, timer, title='Comm Status'):
        super().__init__(screen, timer, title)
        self.stats = {}
        self.mode = 't'

    def comms_call_back(self, msg):
        for status in msg.status:
            name = status.name
            # perc = 0.0
            # packs = 0
            # for value in status.values:
            #     if value.key == 'cum_error_rate_perc':
            #         perc = float(value.value)
            #     if value.key == 'cum_packets':
            #         packs = int(value.value)/1000
            #     if value.key == 'packets':
            #         last_packs = int(value.value)
            #     if value.key == 'errors':
            #         last_errors = int(value.value)
            #     if value.key == 'real_rate':
            #         real_rate = float(value.value)
            # self.stats[name] = {
            #     'perc': perc,
            #     'packs': packs,
            #     'last_packs': last_packs,
            #     'last_errors': last_errors,
            #     'real_rate': 
            # }
            keys = [v.key for v in status.values]
            values = [v.value for v in status.values]
            self.stats[name] = dict(zip(keys, values))

    def create_content(self):
        lb = Listbox(height=18, width=36)
        lb.append('Name                  Packs[k] %Err ', 0)
        for pos in range(1, 18):
            lb.append('', pos)
        self.comm_subsr = rospy.Subscriber('communication_statistics', DiagnosticArray, self.comms_call_back)
        return lb

    def update_content(self):
        if self.mode == 't':
            self.content.replace('Name                  Packs[k] %Err ', 0)
            for index, name in enumerate(self.stats):
                stats = self.stats[name]
                packs = float(stats.get('cum_packets', '0'))/1000.0
                perc = float(stats.get('cum_error_rate_perc', '0'))
                self.content.replace(f'{name[:22]:22s} {packs:7.1f} {perc:4.1f}%', index + 1)
        elif self.mode == 'l':
            self.content.replace('Name                     Packs Errs', 0)
            for index, name in enumerate(self.stats):
                stats = self.stats[name]
                packs = int(stats.get('packets', '0'))
                errors = int(stats.get('errors', '0'))
                self.content.replace(f'{name[:22]:22s} {packs:7d} {errors:4d}', index + 1)
        elif self.mode == 'r':
            self.content.replace('Name                      Act Rate', 0)
            for index, name in enumerate(self.stats):
                stats = self.stats[name]
                rate = float(stats.get('real_rate', '0'))
                self.content.replace(f'{name[:22]:25s} {rate:7.2f}Hz', index + 1)

    @property
    def hotkeys(self):
        return ['t', 'l', 'r']

    def process_hotkey(self, key):
        if key in self.hotkeys:
            self.mode = key

    def finish(self):
        # self.comm_subsr.unregister()
        del self.comm_subsr
