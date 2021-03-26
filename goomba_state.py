class state(object):  # state object to handle some utuility functions for states

    def __init__(self):
        print("Processing current state:", str(self))

    def left_event(self, event):  # handles events delegated to this state
        pass

    def right_event(self, event):  # handles events delegated to this state
        pass

    def front_event(self, event):  # handles events delegated to this state
        pass

    def cliff_event(self, event):  # handles events delegated to this state
        pass

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__
    
    def __add__()