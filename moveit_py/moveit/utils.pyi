from typing import List, Optional


def create_params_file_from_dict(params: dict, node_name: str) -> str:
    """
    Create a temporary YAML parameter file from a dictionary.

    Args:
        params: A dictionary containing the parameters.
        node_name: The name of the ROS node.

    Returns:
        The path to the temporary parameter file.
    """
    ...

def get_launch_params_filepaths(cli_args: Optional[List[str]] = None) -> List[str]:
    """
    Retrieve file paths specified by the --params-file argument in the CLI arguments.

    Args:
        cli_args: A list of command-line arguments. Defaults to sys.argv if None.

    Returns:
        A list of file paths specified after the --params-file arguments.
    """
    ...
