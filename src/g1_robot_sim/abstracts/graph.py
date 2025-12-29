import yaml
import re
from typing import Tuple, List, Dict
from isaacsim.core.utils.extensions import enable_extension

# Enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.physics.ui")
# Enable Action Graph
enable_extension("omni.kit.widget.graph")
enable_extension("omni.graph.core")
enable_extension("omni.graph.action")
enable_extension("omni.graph.action_nodes")
enable_extension("omni.graph.bundle.action")
enable_extension("omni.graph.window.core")
enable_extension("omni.graph.window.action")
enable_extension("isaacsim.examples.browser")

# https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.ros2.bridge/docs/index.html
# https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.nodes/docs/index.html


class Graph:
    def __init__(self, config_path: str, *args, **kwargs):
        """
        Initializes the Graph class from a YAML configuration file and replaces
        placeholders with the arguments passed in the constructor.
        
        Args:
            config_path (str): Path to the YAML configuration file.
            *args: Additional positional arguments (e.g., prim_path).
            **kwargs: Additional keyword arguments (e.g., resolution).
        """
        self.yaml_file = config_path
        self.args = args
        self.kwargs = kwargs
        
        # Load YAML file
        self.graph_data = self._load_yaml()

        # Replace placeholders in the graph data with the arguments passed in
        self.graph_data = self._replace_placeholders(self.graph_data)

        # Extract nodes, connections, and values
        self.og_key_create_node = self.graph_data.get('nodes', [])
        self.og_keys_connect = self.graph_data.get('connections', [])
        self.og_keys_set_values = self.graph_data.get('values', [])

    def _load_yaml(self) -> Dict:
        """Loads and parses the YAML file."""
        with open(self.yaml_file, 'r') as file:
            return yaml.safe_load(file)

    def _replace_placeholders(self, graph_data: Dict) -> Dict:
        """
        Recursively replaces placeholders in the graph data with the corresponding
        arguments passed in the constructor.
        
        Args:
            graph_data (dict): The graph data (nodes, connections, values).
        
        Returns:
            dict: Graph data with placeholders replaced by actual values.
        """
        # If the data is a list, process each item in the list
        if isinstance(graph_data, list):
            return [self._replace_placeholder_in_item(item) for item in graph_data]
        # If the data is a dictionary, process each value
        elif isinstance(graph_data, dict):
            return {key: self._replace_placeholders(value) for key, value in graph_data.items()}
        # If it's not a list or dictionary, process the item itself (it should be a string)
        else:
            return self._replace_placeholder_in_item(graph_data)

    def _replace_placeholder_in_item(self, item):
        """
        Replace placeholders in a single item (either a string or part of a list/tuple) while preserving types.

        Args:
            item: The item with placeholders to replace. Can be a string, list, or tuple.
        
        Returns:
            The item with placeholders replaced by actual values, preserving types.
        """
        # If the item is a list, process each element in the list
        if isinstance(item, list):
            return [self._replace_placeholder_in_item(sub_item) for sub_item in item]
        
        # If the item is a string, perform the replacement manually without using `replace()`
        elif isinstance(item, str):
            # Pattern to match ${variable_name} including dots and underscores
            pattern = r"\$\{([a-zA-Z0-9._]+)\}"
            
            # Find all matches (placeholders) in the string
            match = re.findall(pattern, item)

            if not match:
                return item
            
            # Split the string into parts around the placeholders
            new_item = re.split(pattern, item)[1:]

            match_ = match[0]
            if match_ in self.kwargs:
                # Add the value from kwargs (preserving the type)
                new_item = self.kwargs[match_]
            else:
                raise ValueError(f"Missing value for placeholder: {match_}")
            
            return new_item

        # If the item is a tuple, recursively replace placeholders in each element
        elif isinstance(item, tuple):
            return tuple(self._replace_placeholder_in_item(sub_item) for sub_item in item)
        
        # If it's neither a string, list, nor tuple, return the item as is
        return item

    def get_graph(self) -> Tuple[List[Tuple[str, str]], List[Tuple[str, str]], List[Tuple[str, str]]]:
        """
        Returns the graph structure containing nodes, connections, and values.
        
        Args:
            None
        
        Returns:
            Tuple: Contains three lists (nodes, connections, values).
        """
        return self.og_key_create_node, self.og_keys_connect, self.og_keys_set_values
