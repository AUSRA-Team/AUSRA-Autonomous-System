# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Dict
from typing import List
from typing import Text
from typing import Optional

import yaml
import tempfile
import launch
import copy


class DictItemReference:
    def __init__(self, dictionary, key):
        self.dictionary = dictionary
        self.dictKey = key

    def key(self):
        return self.dictKey

    def setValue(self, value):
        self.dictionary[self.dictKey] = value


class RewrittenYaml(launch.Substitution):
    """
    Substitution that modifies the given YAML file.

    Used in launch system
    """

    def __init__(self,
        source_file: launch.SomeSubstitutionsType,
        param_rewrites: Dict,
        root_key: Optional[launch.SomeSubstitutionsType] = None,
        key_rewrites: Optional[Dict] = None,
        convert_types = False,
        namespace: Optional[launch.SomeSubstitutionsType] = None) -> None:
        super().__init__()
        """
        Construct the substitution

        :param: source_file the original YAML file to modify
        :param: param_rewrites mappings to replace
        :param: root_key if provided, the contents are placed under this key
        :param: key_rewrites keys of mappings to replace
        :param: convert_types whether to attempt converting the string to a number or boolean
        :param: namespace if provided, prefixes each top-level node key with this namespace
                         This ensures parameters are found by nodes launched in a namespace
                         e.g., 'controller_server' becomes 'ausra_1.controller_server'
        """

        from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__param_rewrites = {}
        self.__key_rewrites = {}
        self.__convert_types = convert_types
        self.__root_key = None
        self.__namespace = None
        for key in param_rewrites:
            self.__param_rewrites[key] = normalize_to_list_of_substitutions(param_rewrites[key])
        if key_rewrites is not None:
            for key in key_rewrites:
                self.__key_rewrites[key] = normalize_to_list_of_substitutions(key_rewrites[key])
        if root_key is not None:
            self.__root_key = normalize_to_list_of_substitutions(root_key)
        if namespace is not None:
            self.__namespace = normalize_to_list_of_substitutions(namespace)

    @property
    def name(self) -> List[launch.Substitution]:
        """Getter for name."""
        return self.__source_file

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: launch.LaunchContext) -> Text:
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
        param_rewrites, keys_rewrites = self.resolve_rewrites(context)
        
        # First, do text-level replacement for placeholder patterns like <robot_namespace>
        # This handles cases where the placeholder is part of a string value
        with open(yaml_filename, 'r') as f:
            yaml_content = f.read()
        
        # Perform text-level substitutions for placeholders within strings
        for key, value in param_rewrites.items():
            if key.startswith('<') and key.endswith('>'):
                # This is a placeholder pattern - do text replacement
                yaml_content = yaml_content.replace(key, value)
        
        # Now parse the YAML with placeholders replaced
        data = yaml.safe_load(yaml_content)
        
        # Apply remaining param rewrites (for exact key matches)
        self.substitute_params(data, param_rewrites)
        self.substitute_keys(data, keys_rewrites)
        
        # Handle namespace prefixing for multi-robot support
        # This prefixes each top-level key (node name) with the namespace
        # so that namespaced nodes can find their parameters
        if self.__namespace is not None:
            namespace = launch.utilities.perform_substitutions(context, self.__namespace)
            if namespace:
                data = self.prefix_keys_with_namespace(data, namespace)
        
        if self.__root_key is not None:
            root_key = launch.utilities.perform_substitutions(context, self.__root_key)
            if root_key:
                data = {root_key: data}
        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def prefix_keys_with_namespace(self, data, namespace):
        """
        Prefix all top-level keys (node names) with the namespace.
        
        For multi-robot setups where nodes are launched in a namespace like /ausra_1,
        the nodes look for parameters under their fully qualified name.
        
        This transforms:
            controller_server:
              ros__parameters:
                ...
        
        Into:
            ausra_1:
              controller_server:
                ros__parameters:
                  ...
        
        This allows the same YAML to be used for multiple robots by just changing
        the namespace parameter at launch time.
        """
        if not isinstance(data, dict):
            return data
        
        # Create the namespaced structure
        namespaced_data = {}
        non_namespaced_items = {}
        
        for key, value in data.items():
            # Keep /** wildcard at root level (applies globally)
            if key == '/**':
                namespaced_data[key] = value
            # For node-level entries (those with ros__parameters), wrap under namespace
            elif isinstance(value, dict) and 'ros__parameters' in value:
                # This is a node's parameter block - needs namespace prefix
                if namespace not in namespaced_data:
                    namespaced_data[namespace] = {}
                namespaced_data[namespace][key] = copy.deepcopy(value)
            # Handle nested node structures (like local_costmap.local_costmap)
            elif isinstance(value, dict):
                # Check if any child has ros__parameters
                has_ros_params = False
                for child_key, child_val in value.items():
                    if isinstance(child_val, dict) and 'ros__parameters' in child_val:
                        has_ros_params = True
                        break
                
                if has_ros_params:
                    # This is a nested node structure, wrap under namespace
                    if namespace not in namespaced_data:
                        namespaced_data[namespace] = {}
                    namespaced_data[namespace][key] = copy.deepcopy(value)
                else:
                    # Keep as-is
                    namespaced_data[key] = value
            else:
                # Keep non-dict items at root
                namespaced_data[key] = value
        
        return namespaced_data

    def resolve_rewrites(self, context):
        resolved_params = {}
        for key in self.__param_rewrites:
            resolved_params[key] = launch.utilities.perform_substitutions(context, self.__param_rewrites[key])
        resolved_keys = {}
        for key in self.__key_rewrites:
            resolved_keys[key] = launch.utilities.perform_substitutions(context, self.__key_rewrites[key])
        return resolved_params, resolved_keys

    def substitute_params(self, yaml, param_rewrites):
        # substitute leaf-only parameters
        for key in self.getYamlLeafKeys(yaml):
            if key.key() in param_rewrites:
                raw_value = param_rewrites[key.key()]
                key.setValue(self.convert(raw_value))

        # substitute total path parameters
        yaml_paths = self.pathify(yaml)
        for path in yaml_paths:
            if path in param_rewrites:
                # this is an absolute path (ex. 'key.keyA.keyB.val')
                rewrite_val = self.convert(param_rewrites[path])
                yaml_keys = path.split('.')
                yaml = self.updateYamlPathVals(yaml, yaml_keys, rewrite_val)


    def updateYamlPathVals(self, yaml, yaml_key_list, rewrite_val):
        for key in yaml_key_list:
            if key == yaml_key_list[-1]:
                yaml[key] = rewrite_val
                break
            key = yaml_key_list.pop(0)
            if isinstance(yaml, list):
                yaml[int(key)] = self.updateYamlPathVals(yaml[int(key)], yaml_key_list, rewrite_val)
            else:
                yaml[key] = self.updateYamlPathVals(yaml.get(key, {}), yaml_key_list, rewrite_val)
        return yaml

    def substitute_keys(self, yaml, key_rewrites):
        if len(key_rewrites) != 0:
            for key in list(yaml.keys()):
                val = yaml[key]
                if key in key_rewrites:
                    new_key = key_rewrites[key]
                    yaml[new_key] = yaml[key]
                    del yaml[key]
                if isinstance(val, dict):
                    self.substitute_keys(val, key_rewrites)

    def getYamlLeafKeys(self, yamlData):
        try:
            for key in yamlData.keys():
                for k in self.getYamlLeafKeys(yamlData[key]):
                    yield k
                yield DictItemReference(yamlData, key)
        except AttributeError:
            return

    def pathify(self, d, p=None, paths=None, joinchar='.'):
        if p is None:
            paths = {}
            self.pathify(d, "", paths, joinchar=joinchar)
            return paths
        pn = p
        if p != "":
            pn += joinchar
        if isinstance(d, dict):
            for k in d:
                v = d[k]
                self.pathify(v, str(pn) + str(k), paths, joinchar=joinchar)
        elif isinstance(d, list):
            for idx, e in enumerate(d):
                self.pathify(e, pn + str(idx), paths, joinchar=joinchar)
        else:
            paths[p] = d

    def convert(self, text_value):
        if self.__convert_types:
            # try converting to int or float
            try:
                return float(text_value) if '.' in text_value else int(text_value)
            except ValueError:
                pass

        # try converting to bool
        if text_value.lower() == "true":
            return True
        if text_value.lower() == "false":
            return False

        # nothing else worked so fall through and return text
        return text_value
