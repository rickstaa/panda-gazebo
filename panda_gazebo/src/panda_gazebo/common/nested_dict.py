"""This dictionary class overloads the collections defaultdict class such that you can
create dictionary new dictionary keys and values even when the top key doesn't exist.
"""

from collections import defaultdict


class NestedDict(defaultdict):
    """A dictionary class that allows you to create dictionary fields that are multiple
    layers deep.
    """

    def __init__(self, value=None):
        """Initiate nested dict object"""
        super(NestedDict, self).__init__(NestedDict)

        # Store value
        self.value = value
