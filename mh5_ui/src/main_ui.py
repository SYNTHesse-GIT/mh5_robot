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

from snack import SnackScreen, Widget


class MainUI():
    """Main UI class that handles the views and switches between them.

    The MainUI setups a SnackScreen, determines the size of the avaialable
    screen and handles the main display loop that processes the hotkeys.
    An additional hot-key 'q' is provided to quit the loop and close the
    display.
    """
    def __init__(self) -> None:
        """Initializes the UI. Allocates the SnackScreen, determines the
        width and height of the screen and initializes the views.
        """
        # setup the graphics
        self.screen = SnackScreen()
        # for convenience
        self.w = self.screen.width
        self.h = self.screen.height
        # main window
        self.views = {}
        self.current_view = None
        self.done = False

    def add_view(self, view: Widget,
                       hot_key: str,
                       default_view: bool = False) -> None:
        """Adds a view (page) to the dictionay of views. Views are held by
        their hotkey.

        Parameters
        ----------
        view : Widget or subclass
            The view (page) to be added. The view must be fully constructed
            and run() must be possible to be executed on that object.
        hot_key : str
            The key associated with the view. The main loop will process
            keys and if they match one of these it will handle the switch
            to that particular view.
        default_view : bool, optional
            Marks this view as the default view whihc means the MainUI will
            use this to sart displaying the interface when executing run()
            for the first time. When you add views to the MainUI the last
            one that uses the ``default_view`` will overwrite the other ones
            and that will be the one to be used. If no view is defined as
            ``default_view`` the MainUI will will the first item in the list
            of hot-keys. Becuase the way the dictionaries work in Python
            this might not be the first view added. By default False
        """
        self.views[hot_key] = view
        if default_view:
            self.default_view = hot_key

    def change_view(self, hotkey: str) -> None:
        """Changes a view to the one specified by the hot-key provided.

        The method will ask the present view to finish() then will popWindow()
        from the screen. It will asign the view represented in the dictionary
        by the ``hotkey`` to the ``current_view``, it will ask to setup(),
        and will setup the hot-keys from that view.

        Parameters
        ----------
        hotkey : str
            The hot-key indentifying that view.
        """
        if self.current_view:
            self.current_view.finish()
            self.screen.popWindow()
        self.current_view = self.views[hotkey]
        self.current_view.setup()
        for key in self.views.keys():
            self.current_view.grid.addHotKey(key)
        self.current_view.grid.addHotKey('q')

    def run(self) -> None:
        """Runs the main loop of the UI. It will activate the ``default_view``
        and then will execute a run() for that view (which for shack means
        to wait for a key press) then handle the hotkeys by switching the
        views if they match the ones associated with the views or finish the
        loop if 'q' was pressed.
        
        .. Note:

            The actual refresh of values inside the views is done by the
            timer that is created by the view itself. See the class View for
            details.
        """
        if not self.default_view:
            self.default_view = list(self.views.keys())[0]
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
