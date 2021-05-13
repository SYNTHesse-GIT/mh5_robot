from snack import GridForm, Textbox, Scale


# Grid, , SnackScreen, Label, Listbox, Scale, , \
#                   , ButtonChoiceWindow



class View():
    """Base class for a view."""
    def __init__(self, screen, timer, title):
        self.screen = screen
        self.timer = timer
        self.title = title

    def setup(self):
        """Must be caleed by the MainUi before starting the view. This
        creates all the objects of the UI and initializes them. Sets-up
        a ``GridForm`` of size 1x1 and calls ``create_content`` to fill
        the specific content of the view. Subclasses must implement this
        method. It also revisters the hot keys as are reported by the
        ``hotkeys`` property that must be subclassed if the view needs to
        handle keys."""
        self.grid = GridForm(self.screen, self.title, 1, 1)
        self.content = self.create_content()
        self.grid.add(self.content, 0, 0)
        for key in self.hotkeys:
            self.grid.addHotKey(key)
        if self.timer:
            self.grid.setTimer(self.timer)

    def create_content(self):
        """Should be impelemented in subclasses to produce the desired
        view output."""
        return Textbox(width=20, height=4,
                       text='Default text for the main UI', wrap=1)

    @property
    def hotkeys(self):
        """Returns the keys this view handles. If implemented by subclasses
        then also ``process_hotkey`` should be implemented."""
        return []

    def update_content(self):
        """Handles updates to the content of the view. Normally these are
        triggered by the elpsed timer set up by the ``timer`` property. Should
        be implemented in the subclass according to the desired behaviour."""
        pass

    def process_hotkey(self, key):
        """Processes the declared hotkeys. Should be implemented in subclass."""
        pass

    def run(self):
        """Performs a ``run`` of the grid. First calls the ``update_content``
        to trigger updates to the interface and refreshes the screen. After
        running a ``grid.run()`` it will ask the ``process_key`` method to
        process the hotkey pressed (if any) after whicg it returns the hot key
        to the caller program (tipically the MainUI) so that the loop there
        can process it's own hot keys."""
        self.update_content()
        self.screen.refresh()
        key = self.grid.run()
        self.process_hotkey(key)
        return key

    def finish(self):
        """Provides a way for thge view to clear resources before being
        switched from. For intance views that are displaying information
        from ROS topics have the change to unsubscribe from the topics here.
        """
        pass


class NameValueScale():

    def __init__(self, name, unit, grid, row, widths, min_val, max_val):
        self.unit = unit
        self.name = Textbox(widths[0], 1, name)
        grid.setField(self.name, 0, row)
        self.value = Textbox(widths[1], 1, '')
        grid.setField(self.value, 1, row)
        self.min_val = min_val
        self.max_val = max_val
        self.range_val = self.max_val - self.min_val
        self.scale = Scale(widths[2], int(self.range_val * 100))
        grid.setField(self.scale, 2, row)

    def update_value(self, value, format='4.1f'):
        self.value.setText(f'{value:{format}}{self.unit}')
        self.scale.set(int((value - self.min_val) * 100))


class NameStatValue():

    def __init__(self, name, unit, grid, row, widths):
        self.unit = unit
        self.name = Textbox(widths[0], 1, name)
        grid.setField(self.name, 0, row)
        self.stat = Textbox(widths[1], 1, '')
        grid.setField(self.stat, 1, row)
        self.value = Textbox(widths[2], 1, '')
        grid.setField(self.value, 2, row)

    def update_value(self, stat, value=''):
        self.stat.setText(f'{stat:>4s}{self.unit}')
        self.value.setText(value)
