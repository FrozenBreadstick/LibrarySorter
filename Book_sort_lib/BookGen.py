import numpy

class BookGen:
    def __init__(self, space, book_size_range=[1, 2, 3]):
        """
        Initialize the BookGen class with space and range of book sizes.

        :param space: The available space.
        :type space: int
        :param book_size_range: The range of book sizes. Default is [1, 2, 3].
        :type book_size_range: list of int
        """
        self.space = space
        self.book_size_range = book_size_range
        self.all_combinations = []

    def find_combinations(self, n, current_combination):
        """
        Recursive helper function to find all possible book combinations.

        :param n: Remaining space to fill.
        :type n: int
        :param current_combination: Current combination of book sizes.
        :type current_combination: list of int
        """
        if n == 0:
            self.all_combinations.append(list(current_combination))
            return
        if n < 0:
            return

        for size in self.book_size_range:
            current_combination.append(size)
            self.find_combinations(n - size, current_combination)
            current_combination.pop()  # Backtrack

    def generate_combinations(self):
        """
        Generates all possible combinations of books for the given space.

        :return: List of all combinations.
        :rtype: list of lists
        """
        self.all_combinations = []  # Reset combinations
        self.find_combinations(self.space, [])
        return self.all_combinations

# Example Usage
if __name__ == "__main__":
    space = 5
    book_gen = BookGen(space, book_size_range=[1, 2, 3])
    combinations = book_gen.generate_combinations()

    print(f"All possible combinations for a space of {space}:")
    for combination in combinations:
        print(combination)
