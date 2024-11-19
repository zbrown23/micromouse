import re
from pathlib import Path

# TODO: Finish the maze generation if we get to it.

class Cell:
    """
    Defines a cell of the maze.
    """

    def __init__(self, x, y, walls=None, goal=False, start=False):
        if walls is None:
            walls = [0, 0, 0, 0]
        self.x = x
        self.y = y
        self.walls = walls
        self.is_goal = goal
        self.is_start = start


class Maze:
    def __init__(self, width, height):
        self.width = width
        self.height = height

    def plot(self):
        pass


class MazeParser:
    """
    Parse a textfile representing a maze.
    This parser is based on code and mazes found at https://github.com/micromouseonline/mazefiles
    """

    def __init__(self, file: Path, maze: Maze = None):
        self.file = file
        self.maze = maze

    def _read_maze(self):
        rows = self.file.read_text().split('\n')
        i = 0
        for i, row in enumerate(rows):
            if not row.startswith(('o', '|')):
                break
        return rows[:i]

    def _find_tagged_cells(self, rows, tag):
        found = set()
        for i, row in enumerate(reversed(rows[1::2])):
            for j, column in enumerate(row[2::4]):
                if column is tag:
                    found.add((i, j))
        return found

    def _find_cell_walls(self, rows, row, column):
        middle_row = len(rows) - 2 * row - 2
        middle_column = 2 + 4 * column
        north_row = middle_row - 1
        south_row = middle_row + 1
        west_column = middle_column - 2
        east_column = middle_column + 2
        north = rows[north_row][middle_column] == '-'
        east = rows[middle_row][east_column] == '|'
        south = rows[south_row][middle_column] == '-'
        west = rows[middle_row][west_column] == '|'
        return north, east, south, west