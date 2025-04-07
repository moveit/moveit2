from typing import str

class ExecutionStatus:
    def __init__(self) -> None:
        """
        Execution status of the planned robot trajectory.
        """
        ...

    @property
    def status(self) -> str:
        """
        Get the execution status of the robot trajectory as a string.

        Returns:
            A string representation of the execution status.
        """
        ...
        
    def __bool__(self) -> bool:
        """
        Return True if the execution was successful, False otherwise.

        Returns:
            bool: True if execution status indicates success.
        """
        ...
