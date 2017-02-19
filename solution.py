
class SudokuSolver:
    def __init__(self):

        self.rows = 'ABCDEFGHI'
        self.cols = '123456789'
        self.assignments = []
        self.boxes = self.cross(self.rows, self.cols)
        self.row_units = [self.cross(r, self.cols) for r in self.rows]
        self.column_units = [self.cross(self.rows, c) for c in self.cols]
        self.square_units = [self.cross(rs, cs) for rs in ('ABC','DEF','GHI') for cs in ('123','456','789')]
        self.diag_units = [[x[0] + x[1] for x in zip(self.rows, self.cols)]] + \
                          [[x[0] + x[1] for x in zip(self.rows[::-1], self.cols)]]
        self.unitlist = self.row_units + self.column_units + self.square_units + self.diag_units
        self.units = dict((s, [u for u in self.unitlist if s in u]) for s in self.boxes)
        self.peers = dict((s, set(sum(self.units[s],[]))-set([s])) for s in self.boxes)

    def assign_value(self, values, box, value):
        """
        Please use this function to update your values dictionary!
        Assigns a value to a given box. If it updates the board record it.
        """
        values[box] = value
        if len(value) == 1:
            self.assignments.append(values.copy())
        return values

    def naked_twins(self, values):
        """Eliminate values using the naked twins strategy.
        Args:
            values(dict): a dictionary of the form {'box_name': '123456789', ...}

        Returns:
            the values dictionary with the naked twins eliminated from peers.
        """

        # Find all instances of naked twins
        # Eliminate the naked twins as possibilities for their peers
        for unit in self.unitlist:
            pairs = [p for p in unit if len(values[p]) == 2]

            for peer in pairs:
                twins = [x for x in pairs if (not x == peer) and (values[peer] == values[x])] + [peer]

                if len(twins) == 2:
                    for box in [x for x in unit if not x in twins]:
                        for d in values[peer]:
                            self.assign_value(values, box, values[box].replace(d, ''))

        return values

    def cross(self, A, B):
        "Cross product of elements in A and elements in B."
        return [s + t for s in A for t in B]

    def grid_values(self, grid):
        """
        Convert grid into a dict of {square: char} with '123456789' for empties.
        Args:
            grid(string) - A grid in string form.
        Returns:
            A grid in dictionary form
                Keys: The boxes, e.g., 'A1'
                Values: The value in each box, e.g., '8'. If the box has no value, then the value will be '123456789'.
        """
        chars = []
        digits = '123456789'
        for c in grid:
            if c in digits:
                chars.append(c)
            if c == '.':
                chars.append(digits)
        assert len(chars) == 81
        return dict(zip(self.boxes, chars))

    def display(self, values):
        """
        Display the values as a 2-D grid.
        Args:
            values(dict): The sudoku in dictionary form
        """
        width = 1 + max(len(values[s]) for s in self.boxes)
        line = '+'.join(['-' * (width * 3)] * 3)
        for r in self.rows:
            print(''.join(values[r + c].center(width) + ('|' if c in '36' else '')
                          for c in self.cols))
            if r in 'CF': print(line)
        print


    def eliminate(self, values):
        """
        Go through all the boxes, and whenever there is a box with a value, eliminate this value from the values of all its peers.
        Input: A sudoku in dictionary form.
        Output: The resulting sudoku in dictionary form.
        """
        solved_values = [box for box in values.keys() if len(values[box]) == 1]
        for box in solved_values:
            digit = values[box]
            for peer in self.peers[box]:
                self.assign_value(values, peer, values[peer].replace(digit, ''))
        return values

    def only_choice(self, values):
        """
        Go through all the units, and whenever there is a unit with a value that only fits in one box, assign the value to this box.
        Input: A sudoku in dictionary form.
        Output: The resulting sudoku in dictionary form.
        """
        for unit in self.unitlist:
            for digit in '123456789':
                dplaces = [box for box in unit if digit in values[box]]
                if len(dplaces) == 1:
                    self.assign_value(values, dplaces[0], digit)
                    # values[dplaces[0]] = digit
        return values

    def reduce_puzzle(self, values):
        """
        Iterate eliminate() and only_choice(). If at some point, there is a box with no available values, return False.
        If the sudoku is solved, return the sudoku.
        If after an iteration of both functions, the sudoku remains the same, return the sudoku.
        Input: A sudoku in dictionary form.
        Output: The resulting sudoku in dictionary form.
        """
        solved_values = [box for box in values.keys() if len(values[box]) == 1]
        stalled = False
        while not stalled:
            solved_values_before = len([box for box in values.keys() if len(values[box]) == 1])
            values = self.eliminate(values)
            values = self.only_choice(values)
            values = self.naked_twins(values)
            solved_values_after = len([box for box in values.keys() if len(values[box]) == 1])
            stalled = solved_values_before == solved_values_after
            if len([box for box in values.keys() if len(values[box]) == 0]):
                return False
        return values

    def search(self, values):
        "Using depth-first search and propagation, try all possible values."
        # First, reduce the puzzle using the previous function
        values = self.reduce_puzzle(values)
        if values is False:
            return False  ## Failed earlier
        if all(len(values[s]) == 1 for s in self.boxes):
            return values  ## Solved!
        # Choose one of the unfilled squares with the fewest possibilities
        n, s = min((len(values[s]), s) for s in self.boxes if len(values[s]) > 1)
        # Now use recurrence to solve each one of the resulting sudokus, and
        for value in values[s]:
            new_sudoku = values.copy()
            new_sudoku[s] = value
            attempt = self.search(new_sudoku)
            if attempt:
                return attempt

    def solve(self, grid):
        """
        Find the solution to a Sudoku grid.
        Args:
            grid(string): a string representing a sudoku grid.
                Example: '2.............62....1....7...6..8...3...9...7...6..4...4....8....52.............3'
        Returns:
            The dictionary representation of the final sudoku grid. False if no solution exists.
        """
        return self.search(self.grid_values(grid))

if __name__ == '__main__':
    diag_sudoku_grid = '2.............62....1....7...6..8...3...9...7...6..4...4....8....52.............3'
    solver = SudokuSolver()
    solver.display(solver.solve(diag_sudoku_grid))

    try:
        from visualize import visualize_assignments
        visualize_assignments(solver.assignments)

    except SystemExit:
        pass
    except:
        print('We could not visualize your board due to a pygame issue. Not a problem! It is not a requirement.')
