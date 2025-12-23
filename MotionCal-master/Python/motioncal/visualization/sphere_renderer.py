"""
OpenGL 3D sphere visualization

Ports visualize.c - Renders calibrated magnetometer data as spheres
"""

import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from ..utils.constants import MAGBUFFSIZE
from .transforms import quaternion_to_rotation_matrix, rotate_point


class SphereRenderer:
    """
    OpenGL renderer for magnetometer calibration visualization

    Displays 650 calibrated magnetometer points as spheres on a unit sphere,
    rotated by current sensor orientation.
    """

    def __init__(self):
        """Initialize sphere renderer state"""
        self.sphere_list = None
        self.sphere_lowres_list = None
        self.initialized = False

        # Axis inversion flags (from visualize.c)
        self.invert_q0 = False
        self.invert_q1 = False
        self.invert_q2 = False
        self.invert_q3 = True  # Default in C code
        self.invert_x = False
        self.invert_y = False
        self.invert_z = False

    def init_gl(self):
        """
        Initialize OpenGL state and display lists

        Ports: visualize_init from visualize.c
        """
        if self.initialized:
            return

        # Enable depth testing
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)

        # Enable lighting
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)

        # Light position
        light_pos = [1.0, 1.0, 1.0, 0.0]
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos)

        # Ambient light
        ambient = [0.3, 0.3, 0.3, 1.0]
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambient)

        # Background color (dark gray)
        glClearColor(0.2, 0.2, 0.2, 1.0)

        # Create sphere display lists
        self._create_sphere_lists()

        self.initialized = True

    def _create_sphere_lists(self):
        """Create OpenGL display lists for high and low resolution spheres"""
        # High resolution sphere (for foreground points)
        self.sphere_list = glGenLists(1)
        glNewList(self.sphere_list, GL_COMPILE)
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)
        gluSphere(quad, 0.15, 16, 14)  # radius, slices, stacks
        gluDeleteQuadric(quad)
        glEndList()

        # Low resolution sphere (for background points)
        self.sphere_lowres_list = glGenLists(1)
        glNewList(self.sphere_lowres_list, GL_COMPILE)
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)
        gluSphere(quad, 0.15, 12, 10)  # radius, slices, stacks
        gluDeleteQuadric(quad)
        glEndList()

    def render(self, magcal, current_orientation, can_save, quality_reset_func, quality_update_func):
        """
        Render the magnetometer calibration visualization

        Ports: display_callback from visualize.c

        Args:
            magcal: MagCalibration instance
            current_orientation: Quaternion with sensor orientation
            can_save: bool, True if calibration quality is good
            quality_reset_func: Callback to reset quality metrics
            quality_update_func: Callback to update quality metrics
        """
        if not self.initialized:
            self.init_gl()

        # Reset quality metrics
        quality_reset_func()

        # Clear buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Reset modelview matrix
        glLoadIdentity()

        # Scaling and offset
        xscale = 0.05
        yscale = 0.05
        zscale = 0.05
        xoff = 0.0
        yoff = 0.0
        zoff = -7.0

        # Get rotation matrix from quaternion
        if current_orientation is not None:
            orientation = current_orientation
            # Apply inversions (for coordinate system alignment)
            if self.invert_q0:
                orientation.q0 *= -1.0
            if self.invert_q1:
                orientation.q1 *= -1.0
            if self.invert_q2:
                orientation.q2 *= -1.0
            if self.invert_q3:
                orientation.q3 *= -1.0

            rotation_matrix = quaternion_to_rotation_matrix(orientation)
        else:
            # Use identity matrix if no orientation yet
            rotation_matrix = np.eye(3, dtype=np.float32)

        # Set color based on calibration quality
        if can_save:
            glColor3f(0.0, 1.0, 0.0)  # Green = good calibration
        else:
            glColor3f(1.0, 0.0, 0.0)  # Red = needs more data

        # Draw all valid magnetometer points as spheres
        draw_sensor = False
        for i in range(MAGBUFFSIZE):
            if magcal.valid[i]:
                draw_sensor = True

                # Get raw magnetometer reading
                rawx = magcal.BpFast[0, i]
                rawy = magcal.BpFast[1, i]
                rawz = magcal.BpFast[2, i]

                # Apply calibration (import here to avoid circular dependency)
                from ..data.apply_calibration import apply_calibration
                point = apply_calibration(rawx, rawy, rawz, magcal)

                # Update quality metrics
                quality_update_func(point)

                # Rotate point by sensor orientation
                draw_x, draw_y, draw_z = rotate_point(point, rotation_matrix)

                # Apply axis inversions
                if self.invert_x:
                    draw_x *= -1.0
                if self.invert_y:
                    draw_y *= -1.0
                if self.invert_z:
                    draw_z *= -1.0

                # Draw sphere at point location
                glPushMatrix()
                # Note: Y and Z swapped for display coordinates
                glTranslatef(
                    draw_x * xscale + xoff,
                    draw_z * yscale + yoff,
                    draw_y * zscale + zoff
                )

                # Use high-res sphere for foreground, low-res for background
                if draw_y >= 0.0:
                    glCallList(self.sphere_list)
                else:
                    glCallList(self.sphere_lowres_list)

                glPopMatrix()

        # Draw sensor orientation indicator (triangle)
        if draw_sensor:
            self._draw_sensor_indicator(xscale, yscale, zscale, xoff, yoff, zoff)

    def _draw_sensor_indicator(self, xscale, yscale, zscale, xoff, yoff, zoff):
        """
        Draw triangle indicator showing sensor orientation

        Yellow triangle points in sensor's forward direction
        Blue triangle points in sensor's up direction
        """
        sensor_scale_x = 1.0
        sensor_scale_y = 1.0
        sensor_scale_z = 1.0
        sensor_x = 0.0
        sensor_y = 0.0
        sensor_z = -7.0

        glPushMatrix()
        glTranslatef(sensor_x, sensor_y, sensor_z)
        glScalef(sensor_scale_x, sensor_scale_y, sensor_scale_z)

        # Disable lighting for indicator
        glDisable(GL_LIGHTING)

        # Yellow forward indicator
        glColor3f(1.0, 1.0, 0.0)
        glBegin(GL_TRIANGLES)
        glVertex3f(0.0, 0.3, 0.0)
        glVertex3f(-0.15, -0.15, 0.0)
        glVertex3f(0.15, -0.15, 0.0)
        glEnd()

        # Blue up indicator
        glColor3f(0.0, 0.0, 1.0)
        glBegin(GL_TRIANGLES)
        glVertex3f(0.0, 0.0, 0.3)
        glVertex3f(-0.15, 0.0, -0.15)
        glVertex3f(0.15, 0.0, -0.15)
        glEnd()

        # Re-enable lighting
        glEnable(GL_LIGHTING)

        glPopMatrix()

    def resize(self, width, height):
        """
        Handle window resize

        Ports: resize_callback from visualize.c

        Args:
            width: New window width
            height: New window height
        """
        if height == 0:
            height = 1

        glViewport(0, 0, width, height)

        # Set projection matrix
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()

        # Perspective projection
        aspect = float(width) / float(height)
        glFrustum(-0.1 * aspect, 0.1 * aspect, -0.1, 0.1, 0.2, 20.0)

        # Back to modelview
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def cleanup(self):
        """Clean up OpenGL resources"""
        if self.sphere_list is not None:
            glDeleteLists(self.sphere_list, 1)
            self.sphere_list = None

        if self.sphere_lowres_list is not None:
            glDeleteLists(self.sphere_lowres_list, 1)
            self.sphere_lowres_list = None

        self.initialized = False
