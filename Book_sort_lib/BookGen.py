import numpy
import swift
import platform #To check the OS
import spatialgeometry as geometry
from spatialmath import * #matrix math

class BookGen:
    def __init__(self, space, book_List, book_size_range=[1, 2, 3]):
        """
        Initialize the BookGen class with space and range of book sizes.

        :param space: The available space.
        :type space: int
        :param book_size_range: The range of book sizes. Default is [1, 2, 3].
        :type book_size_range: list of int
        :param book_List: The list of books
        :type book_List: list of int
        """
        self.space = space
        self.book_size_range = book_size_range
        self.all_combinations = []
        self.booksList = book_List
        

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

    def add_book_to_env(self, env, bookList):
        """
        Placeholder method to add books to the environment.
        
        :param env: The environment to add books to.
        :type env: object
        :param bookList: The list of books to add.
        :type bookList: list of int
        """
        exact_path_book1 = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/Models/Assests/SmallBook.stl'
        exact_path_book2 = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/Models/Assests/MediumBook.stl'
        exact_path_book3 = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/Models/Assests/LargeBook.stl'
        
        book1_mesh = geometry.Mesh(filename=exact_path_book1,
                                    pose=SE3(0,0,0),
                                    color=(0.4,0.04,0.04), 
                                    collision=True)
        if platform.system() == 'Windows':
            pass
        else:
         for book in bookList:
             if book == 1:
                 # Add book 1
                 pass
             if book == 2:
                 # Add book 2
                 pass
             if book == 3: 
                 # Add book 3
                 pass
                
       
  
    
    

 # Example Usage
if __name__ == "__main__":
        space = 7
        
        book_gen = BookGen(space, [1, 2, 3])
        combinations = book_gen.generate_combinations()
        print(f"Total possible combinations for a space of {space}: {len(combinations)}")
        print(f"All possible combinations for a space of {space}:")
        for combination in combinations:
            print(combination)
 