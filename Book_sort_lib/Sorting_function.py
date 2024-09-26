import numpy

def run(availableSpace):

    # Check to ensure input is a positive integer
    if not isinstance(availableSpace, int) or not availableSpace < 0:#NEED FIXXXXXXX
        raise ValueError("availableSpace must be a positive integer")
    
    aavailableSpace_=availableSpace #Work out away to get this value be scaned by the main robot 
if __name__ == "__main__":
    run(30)