import subprocess
import time
import psutil
import rospy

from view_ui import View, NameValueScale, NameStatValue

from snack import Grid, Label


# , , SnackScreen, , Listbox, Scale, , \
#     GridForm, Textbox              , ButtonChoiceWindow

class RobotStatusView(View):

    def __init__(self, screen, timer, title='Robot Status'):
        super().__init__(screen, timer, title)
        self.batt_last_change = None
        self.batt_last_change_value = None
        self.batt_last_value = None
        self.batt_last_estimate = None
        self.on_batt_str = '0:00'
        self.rem_batt_str = 'Calc'

    def create_content(self):
        grid = Grid(3, 19)
        w = [16, 6, 14]         # widths for columns
        row = 0                 # current row
        # Voltage
        # grid.setField(Label('Voltage'), 0, row)
        # row += 1
        self.battery = NameValueScale('Battery', 'V', grid, row, w, 9.0, 12.6)
        self.battery.update_value(12.2)
        row += 1
        self.on_battery = NameStatValue('On battery for', '', grid, row, w)
        self.on_battery.update_value(self.on_batt_str)
        row += 1
        self.rem_battery = NameStatValue('Batt remaining', '', grid, row, w)
        self.rem_battery.update_value(self.rem_batt_str)
        row += 1
        self.rpi5v = NameStatValue('5V rail', 'V', grid, row, w)
        self.rpi5v.update_value('5.0')
        row += 1
        self.rpi3v = NameStatValue('3.3V rail', 'V', grid, row, w)
        self.rpi3v.update_value('3.2')
        row += 1
        grid.setField(Label(''), 0, row)
        # Temperature
        # row += 1
        # grid.setField(Label('Temperature'), 0, row)
        row += 1
        self.temp = NameValueScale('RPi temperature', '°', grid, row, w, 25.0, 85.0)
        self.temp.update_value(45.0)
        row += 1
        self.fan = NameStatValue('Fan', '', grid, row, w)
        self.fan.update_value('Off')
        row += 1
        grid.setField(Label(''), 0, row)
        # CPU
        # row += 1
        # grid.setField(Label('CPU'), 0, row)
        row += 1
        self.cpu_speed = NameValueScale('CPU Speed', 'G', grid, row, w, 0.6, 1.5)
        self.cpu_speed.update_value(1.2)
        row += 1
        self.cpu_gov = NameStatValue('CPU Governor', '', grid, row, w)
        self.cpu_gov.update_value('', '')
        row += 1
        self.cpu_1m = NameValueScale('Load [1m]', '', grid, row, w, 0.0, 4.0)
        self.cpu_1m.update_value(2.5)
        row += 1
        self.cpu_5m = NameValueScale('Load [5m]', '', grid, row, w, 0.0, 4.0)
        self.cpu_5m.update_value(2.5)
        row += 1
        self.cpu_15m = NameValueScale('Load [15m]', '', grid, row, w, 0.0, 4.0)
        self.cpu_15m.update_value(2.5)
        row += 1
        # max_mem_str = self.shell_cmd('cat /proc/meminfo | grep MemTotal')
        # self.max_mem = int(max_mem_str.split()[1]) / 1000000.0
        self.cpu_mem = NameValueScale('Memory', '', grid, row, w, 0.0, 100.0)
        self.cpu_mem.update_value(0.0)
        row += 1
        grid.setField(Label(''), 0, row)
        # WiFi
        # row += 1
        # grid.setField(Label('Wi-Fi'), 0, row)
        row += 1
        # get the interface name from AP config
        ap_config = self.shell_cmd('cat /etc/hostapd/hostapd.conf | grep interface')
        self.ap_interf = ap_config.strip().split('=')[1]
        self.ap_stat = NameStatValue('AP', '', grid, row, w)
        self.ap_stat.update_value('', '')
        row += 1
        nets = self.shell_cmd('ls /sys/class/net/ | grep wl*').split('\n')
        nets.remove(self.ap_interf)
        if nets:
            self.wf_interf = nets[0]
        else:
            self.wf_interf = ''
        self.wf_stat = NameStatValue('WAN', '', grid, row, w)
        self.wf_stat.update_value('On', '')
        row += 1
        self.eth_stat = NameStatValue('Eth', '', grid, row, w)
        self.eth_stat.update_value('', '')
        # row += 1
        # grid.setField(Label(''), 0, row)
        return grid

    def shell_cmd(self, command):
        comm = subprocess.run(command, shell=True, stdout=subprocess.PIPE,
                              encoding='utf-8')
        if comm.returncode == 0:
            return comm.stdout.strip()
        else:
            return ''

    def read_sysfs(self, file):
        # try:
        raw = open(file, 'r').read()
        val = raw.strip()
        return val

    def get_interf_status(self, interf):
        inet_line = self.shell_cmd(f'ifconfig {interf} | grep "inet "')
        if inet_line:
            return 'On', inet_line.split(' ')[1]
        else:
            return 'Off', ''

    def update_content(self):
        # voltage
        # 5V railing
        raw =  self.read_sysfs("/sys/class/i2c-dev/i2c-1/device/1-0048/iio:device0/in_voltage1_raw")
        raw = float(raw)
        scale = self.read_sysfs("/sys/class/i2c-dev/i2c-1/device/1-0048/iio:device0/in_voltage1_scale")
        scale = float(scale)
        value = raw * scale / 1000.0 * 2.0      # 1:2 divisor on ADC1
        self.rpi5v.update_value(f'{value:4.1f}')

        # 3.3V railing
        # in_voltage1_raw is giving some bizare readings and we will use the voltage0-voltage1 reading instead
        raw =  self.read_sysfs("/sys/class/i2c-dev/i2c-1/device/1-0048/iio:device0/in_voltage0-voltage1_raw")
        raw = float(raw)
        scale = self.read_sysfs("/sys/class/i2c-dev/i2c-1/device/1-0048/iio:device0/in_voltage0-voltage1_scale")
        scale = float(scale)
        # value contains the 5V calculation from earlier
        # raw * scale is giving howmuch more ADC0 input is vs. ADC1;
        # ADC1 is the previous 5V railing / 2 (because of divisor)
        value = value / 2.0 + (raw * scale) / 1000.0
        self.rpi3v.update_value(f'{value:4.1f}')

        # Battery railing
        raw =  self.read_sysfs("/sys/class/i2c-dev/i2c-1/device/1-0048/iio:device0/in_voltage2_raw")
        raw = float(raw)
        scale = self.read_sysfs("/sys/class/i2c-dev/i2c-1/device/1-0048/iio:device0/in_voltage2_scale")
        scale = float(scale)
        value = raw * scale / 1000.0 * 4.0      # 1:4 divisor on ADC2
        self.battery.update_value(value)

        # estimates
        #
        if self.batt_last_change_value is None:
            self.batt_last_change_value = value
            self.batt_last_estimate = time.time()
            self.batt_last_change = time.time()
        #
        if self.batt_last_value is None:
            self.batt_last_value = value

        now = time.time()
        if value >= self.batt_last_value + 0.5:
            # battery changed
            self.batt_last_change = now
            self.batt_last_estimate = now
            self.batt_last_change_value = value
            self.on_batt_str = '0:00'
            self.rem_batt_str = 'Calc'
        if now > self.batt_last_estimate + 60:
            # update battery status every 60 seconds
            self.batt_last_estimate = now
            on_batt = now - self.batt_last_change
            self.on_batt_str = time.strftime("%-H:%M", time.gmtime(on_batt))
            v_used = self.batt_last_change_value - value
            if v_used > 0.1:
                v_rem = max(value - 9.0, 0)
                t_rem = v_rem * on_batt / v_used
                self.rem_batt_str = time.strftime("%-H:%M", time.gmtime(t_rem))
            else:
                self.rem_batt_str = 'Calc'
        self.on_battery.update_value(self.on_batt_str)
        self.rem_battery.update_value(self.rem_batt_str)
        self.batt_last_value = value

        # temperature
        temp_str = self.read_sysfs('/sys/class/thermal/thermal_zone0/temp')
        self.temp.update_value(int(temp_str)/1000.0)
        fan_str = self.read_sysfs('/sys/class/thermal/cooling_device0/cur_state')
        if fan_str == '1':
            self.fan.update_value('On')
        else:
            self.fan.update_value('Off')

        # CPU
        freq_str = self.read_sysfs('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq')
        self.cpu_speed.update_value(float(freq_str)/1000000.0)
        gov = self.read_sysfs('/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor')
        self.cpu_gov.update_value('', gov)
        load_str = self.read_sysfs('/proc/loadavg').split()
        self.cpu_1m.update_value(float(load_str[0]), format='4.2f')
        self.cpu_5m.update_value(float(load_str[1]), format='4.2f')
        self.cpu_15m.update_value(float(load_str[2]), format='4.2f')
        # memory
        mem_str = psutil.virtual_memory().percent
        # mem_str = self.shell_cmd('cat /proc/meminfo | grep MemFree')
        # mem = int(mem_str.split()[1]) / 1000000.0
        self.cpu_mem.update_value(float(mem_str))

        # wi-fi
        if self.ap_interf:
            stat, ip_addr = self.get_interf_status(self.ap_interf)
            self.ap_stat.update_value(stat, ip_addr)
        else:
            self.ap_stat.update_value('N/A')

        if self.wf_interf:
            stat, ip_addr = self.get_interf_status(self.wf_interf)
            self.wf_stat.update_value(stat, ip_addr)
        else:
            self.wf_stat.update_value('N/A')

        stat, ip_addr = self.get_interf_status('eth0')
        self.eth_stat.update_value(stat, ip_addr)