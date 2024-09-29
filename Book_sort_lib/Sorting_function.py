import numpy

def run(availableSpace):

    # Check to ensure input is a positive integer
    if not isinstance(availableSpace, int) or not availableSpace < 0:#NEED FIXXXXXXX
        raise ValueError("availableSpace must be a positive integer")
    
    aavailableSpace_=availableSpace #Work out away to get this value be scaned by the main robot 

def find_combinations(n, books, current_combination, all_combinations):
    # Base case: If we filled exactly the space
    if n == 0:
        all_combinations.append(list(current_combination))  # Append a copy of the current combination
        return
    
    # Base case: If we exceeded the space, backtrack
    if n < 0:
        return

    # Try placing books of different sizes (1, 2, or 3)
    for size in books:
        # Add the book size to the current combination
        current_combination.append(size)
        # Recur with the remaining space
        find_combinations(n - size, books, current_combination, all_combinations)
        # Backtrack and remove the last added book size
        current_combination.pop()

def get_book_combinations(space, books=[1, 2, 3]):
    all_combinations = []
    find_combinations(space, books, [], all_combinations)
    return all_combinations


if __name__ == "__main__":
   # Example usage:
 space = 5
 combinations = get_book_combinations(space)
 print(f"All possible combinations for a space of {space}:")
 for combo in combinations:
    print(combo) 
