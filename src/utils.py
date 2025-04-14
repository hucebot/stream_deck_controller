import yaml

def read_config(file_path):
    """
    Reads a YAML configuration file and returns the contents as a dictionary.

    Args:
        file_path (str): The path to the YAML configuration file.

    Returns:
        dict: The contents of the YAML file as a dictionary.
    """
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config