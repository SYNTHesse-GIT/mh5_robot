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

from typing import List
from snack import GridForm, Textbox, Scale, SnackScreen, Widget


class View():
    """Base class for a view."""
    def __init__(self, screen: SnackScreen,
                       timer: int,
                       title: str) -> None:
        """Initializes a new view.

        A view uses a snack.GridForm as a canvas, that is pinned on the screen
        provided and displays a title.

        The constructor only stores the ``screen``, ``timer`` and ``title``
        in the internal variables. You need to specifically call setup() to
        construct the view. setup() will call create_content() that normally
        needs to be overridden by subclasses to present a specific content.

        Parameters
        ----------
        screen : snack.SnackScreen
            The screen where the view will be positioned.
        timer : int
            Refresh time for view in miliseconds. This will trigger the
            update_content().
        title : str
            Title to be presented on the top of the view.
        """
        self.screen = screen
        self.timer = timer
        self.title = title

    def setup(self) -> None:
        """Builds the view content.
        
        Must be called by the MainUi before starting the view. This
        creates all the objects of the UI and initializes them. Sets-up
        a ``GridForm`` of size 1x1 and calls create_content() to fill
        the specific content of the view. It also registers the hot 
        keys as are reported by the ``hotkeys`` property that must be 
        subclassed if the view needs to handle keys.
        """
        self.grid = GridForm(self.screen, self.title, 1, 1)
        self.content = self.create_content()
        self.grid.add(self.content, 0, 0)
        for key in self.hotkeys:
            self.grid.addHotKey(key)
        if self.timer:
            self.grid.setTimer(self.timer)

    def create_content(self) -> Widget:
        """Should be impelemented in subclasses to produce the desired
        view output.
        
        Returns
        -------
        Widget:
            A snack.Widget that will be included in the grid. Note that
            it should be one element only and if you need a more complex
            structure you need to use a GridForm or other classes to contain
            and structure the elements. Have a look at the implementation of
            RobotStatusView, CommsStatusView and JointView.
        """
        return Textbox(width=20, height=4,
                       text='Default text for the main UI', wrap=1)

    @property
    def hotkeys(self) -> List[str]:
        """Returns the keys this view handles. If implemented by subclasses
        then also ``process_hotkey`` should be implemented."""
        return []

    def update_content(self) -> None:
        """Handles updates to the content of the view. Normally these are
        triggered by the elpsed timer set up by the ``timer`` property. Should
        be implemented in the subclass according to the desired behaviour."""
        pass

    def process_hotkey(self, key: str) -> None:
        """Processes the declared hotkeys. Should be implemented in subclass.
        
        Parameters
        ----------
        key : str
            The key to be processed.
        """
        pass

    def run(self) -> str:
        """Performs a ``run`` of the grid.
        
        First calls the update_content() to trigger updates to the interface 
        and refresh() on the screen object. After that it runs the run() of
        the grid object follwied by process_key() method to
        process the hotkey pressed (if any) after which it returns the hot key
        to the caller program (tipically the MainUI) so that the loop there
        can process it's own hot keys.

        Returns
        -------
        str:
            The key pressed for the caller program to handle if necessary
        """
        self.update_content()
        self.screen.refresh()
        key = self.grid.run()
        self.process_hotkey(key)
        return key

    def finish(self) -> None:
        """Provides a way for thge view to clear resources before being
        switched from. For instance views that are displaying information
        from ROS topics have the chance to unsubscribe from the topics here
        to save resources.
        """
        pass


class NameValueScale():
    """A display element that includes a name for the object, a value (+ unit
    of measure if provided) and a Scale (a horizontal bar graph).
    """
    def __init__(self, name: str, 
                       unit: str, 
                       grid: GridForm, 
                       row: int, 
                       widths: List[int], 
                       min_val: float, 
                       max_val: float):
        """Creates a combined display element in one line with a name,
        a value and a horizontal bar graph.

        Parameters
        ----------
        name : str
            The name to be shown on the left side of the display.
        unit : str
            A string to be shown after the value to denote the unit of measure.
        grid : GridForm
            The form where the elements are added to
        row : int
            The row number in the form where the elements will be poistioned.
            All elements are on the same row.
        widths : List[int]
            A list of width for the elements (name, value, scale)
        min_val : float
            The minium value that the element will display. Needed to calibrate
            the bar graph.
        max_val : float
            The maximum value that the element will display. Needed to calibrate
            the bar graph.
        """
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

    def update_value(self, value: float, format: str ='4.1f') -> None:
        """Updates the content of the elements based on the provided
        value.

        Parameters
        ----------
        value : float
            The new value to be displayed. This will be reflected in the
            value field as well as in the bar graph.
        format : str, optional
            The format to display the value in the value field, 
            by default '4.1f'
        """
        self.value.setText(f'{value:{format}}{self.unit}')
        self.scale.set(int((value - self.min_val) * 100))


class NameStatValue():
    """A display element that includes a name for the object, a staus and
    an additional (optional can be '') text.
    """
    def __init__(self, name: str,
                       unit: str,
                       grid: GridForm,
                       row: int,
                       widths: List[int]) -> None:
        """Creates a combined display element in one line with a name,
        a status (+unit of measure if provided) and a value.

        Parameters
        ----------
        name : str
            The name to be shown on the left side of the display.
        unit : str
            [A string to be shown after the value to denote the unit of measure.
        grid : GridForm
            [description]
        row : int
            The row number in the form where the elements will be poistioned.
            All elements are on the same row.
        widths : List[int]
            A list of width for the elements (name, status, value)
        """
        self.unit = unit
        self.name = Textbox(widths[0], 1, name)
        grid.setField(self.name, 0, row)
        self.stat = Textbox(widths[1], 1, '')
        grid.setField(self.stat, 1, row)
        self.value = Textbox(widths[2], 1, '')
        grid.setField(self.value, 2, row)

    def update_value(self, stat: str, value: str ='') -> None:
        """Updates the content of the elements based on the provided
        value.

        Parameters
        ----------
        stat : str
            A string showing the status of the element.
        value : str, optional
            An additional valuee to be shown after the status, by default ''.
        """
        self.stat.setText(f'{stat:>4s}{self.unit}')
        self.value.setText(value)
