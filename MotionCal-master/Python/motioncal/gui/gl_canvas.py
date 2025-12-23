"""
OpenGL canvas widget for MotionCal

Integrates the sphere renderer into PyQt6
"""

from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from PyQt6.QtCore import QSize
from OpenGL.GL import *

from ..visualization.sphere_renderer import SphereRenderer
from ..calibration.quality import quality_reset, quality_update


class GLCanvas(QOpenGLWidget):
    """
    OpenGL widget for magnetometer visualization

    Ports: MyCanvas from gui.cpp
    """

    def __init__(self, parent=None):
        """Initialize OpenGL canvas"""
        super().__init__(parent)

        self.renderer = SphereRenderer()
        self.magcal = None
        self.current_orientation = None
        self.can_save = False

        # Set minimum size
        self.setMinimumSize(480, 480)

    def set_calibration(self, magcal):
        """
        Set calibration data to render

        Args:
            magcal: MagCalibration instance
        """
        self.magcal = magcal

    def set_orientation(self, orientation):
        """
        Set current sensor orientation

        Args:
            orientation: Quaternion with current orientation
        """
        self.current_orientation = orientation

    def set_can_save(self, can_save: bool):
        """
        Set whether calibration quality is good

        Args:
            can_save: True if calibration is good
        """
        self.can_save = can_save

    def initializeGL(self):
        """Initialize OpenGL state"""
        self.renderer.init_gl()

    def paintGL(self):
        """Render frame"""
        if self.magcal is None:
            # Clear to background color
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            return

        # Render calibration visualization (orientation can be None)
        self.renderer.render(
            self.magcal,
            self.current_orientation,
            self.can_save,
            quality_reset,
            quality_update
        )

    def resizeGL(self, width: int, height: int):
        """Handle resize"""
        self.renderer.resize(width, height)

    def minimumSizeHint(self) -> QSize:
        """Return minimum size hint"""
        return QSize(480, 480)

    def sizeHint(self) -> QSize:
        """Return preferred size hint"""
        return QSize(480, 480)
