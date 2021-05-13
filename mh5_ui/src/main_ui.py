from snack import SnackScreen


# Grid, GridForm, Label, Listbox, Scale, , \
#                   Textbox, ButtonChoiceWindow

class MainUI():

    def __init__(self):
        # setup the graphics
        self.screen = SnackScreen()
        # for convenience
        self.w = self.screen.width
        self.h = self.screen.height
        # main window
        self.views = {}
        self.current_view = None
        self.done = False

    def add_view(self, view, hot_key, default_view=False):
        self.views[hot_key] = view
        if default_view:
            self.default_view = hot_key

    def change_view(self, hotkey):
        if self.current_view:
            self.current_view.finish()
            self.screen.popWindow()
        self.current_view = self.views[hotkey]
        self.current_view.setup()
        for key in self.views.keys():
            self.current_view.grid.addHotKey(key)
        self.current_view.grid.addHotKey('q')

    def run(self):
        self.change_view(self.default_view)
        while not self.done:
            key = self.current_view.run()
            if key == 'q':
                self.done = True
            if key in self.views:
                self.change_view(key)

        # finish the loop
        self.screen.popWindow()
        self.screen.finish()
