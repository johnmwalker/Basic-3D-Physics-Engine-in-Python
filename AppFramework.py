# -*- coding: utf-8 -*-
"""
Created on Sun May 13 11:11:56 2018

@author: beoka
"""

import time

import ModernGL as mgl
import numpy as np
from PyQt5 import QtOpenGL, QtWidgets
import sys


class WindowInfo:
    def __init__(self):
        self.size = (0, 0)
        self.mouse = (0, 0)
        self.wheel = 0
        self.time = 0
        self.ratio = 1.0
        self.viewport = (0, 0, 0, 0)
        self.keys = np.full(256, False)
        self.old_keys = np.copy(self.keys)

    def key_down(self, key):
        return self.keys[key]

    def key_pressed(self, key):
        return self.keys[key] and not self.old_keys[key]

    def key_released(self, key):
        return not self.keys[key] and self.old_keys[key]


class AppWindow(QtOpenGL.QGLWidget):
    def __init__(self, size, title):
        fmt = QtOpenGL.QGLFormat()
        fmt.setVersion(3, 3)
        fmt.setProfile(QtOpenGL.QGLFormat.CoreProfile)
        fmt.setSwapInterval(1)
        fmt.setSampleBuffers(True)
        fmt.setDepthBufferSize(24)

        super(AppWindow, self).__init__(fmt, None)
        self.setFixedSize(size[0], size[1])
        self.move(QtWidgets.QDesktopWidget().rect().center() - self.rect().center())
        self.setWindowTitle(title)

        self.start_time = time.clock()
        self.example = lambda: None
        self.ex = None

        self.wnd = WindowInfo()
        self.wnd.viewport = (0, 0) + size
        self.wnd.ratio = size[0] / size[1]
        self.wnd.size = size

    def keyPressEvent(self, event):
        self.wnd.keys[event.nativeVirtualKey() & 0xFF] = True

    def keyReleaseEvent(self, event):
        self.wnd.keys[event.nativeVirtualKey() & 0xFF] = False

    def mouseMoveEvent(self, event):
        self.wnd.mouse = (event.x(), event.y())

    def wheelEvent(self, event):
        self.wnd.wheel += event.angleDelta().y()

    def paintGL(self):
        if self.ex is None:
            self.ex = self.example()
        self.wnd.time = time.clock() - self.start_time
        self.ex.render()
        self.wnd.old_keys = np.copy(self.wnd.keys)
        self.wnd.wheel = 0
        self.update()


def runApp(example):
    app = QtWidgets.QApplication(sys.argv)
    widget = AppWindow(example.WINDOW_SIZE, getattr(example, 'WINDOW_TITLE', example.__name__))
    example.wnd = widget.wnd
    widget.example = example
    widget.show()
    return app, widget
#    del app


class AppParent:
    WINDOW_SIZE = (1280, 720)
    wnd = None  # type: WindowInfo

    def render(self):
        pass