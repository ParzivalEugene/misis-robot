"""
An Apple CGL context for offscreen rendering on macOS.
"""
from __future__ import annotations
import _thread
import atexit as atexit
import contextlib as contextlib
import ctypes as ctypes
from mujoco.cgl.cgl import CGLOpenGLProfile as _PROFILE
from mujoco.cgl.cgl import CGLPixelFormatAttribute as _ATTRIB
import os as os
import threading as threading
import weakref as weakref
from . import cgl
__all__: list[str] = ['GLContext', 'atexit', 'cgl', 'contextlib', 'ctypes', 'os', 'threading', 'weakref']
class GLContext:
    """
    An EGL context for headless accelerated OpenGL rendering on GPU devices.
    """
    @staticmethod
    def as_current(*args, **kwds):
        """
        Temporarily makes this context current, restoring any prior context.
        """
    def __del__(self):
        ...
    def __init__(self, max_width, max_height):
        ...
    def free(self):
        """
        Frees resources associated with this context.
        """
    def make_current(self):
        ...
_current_context: _thread._local
