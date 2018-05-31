import imp
# This module provides an interface to the mechanisms used to implement the import statement. It defines the following constants and functions
import os.path as osp


def load(name):
    pathname = osp.join(osp.dirname(__file__), name)
    return imp.load_source('', pathname)
