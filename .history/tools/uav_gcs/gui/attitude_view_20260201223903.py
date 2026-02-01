"""
UAV Ground Control Station - 3D Attitude Viewer

OpenGL-based 3D visualization của UAV attitude.
Sử dụng VisPy cho rendering hiệu quả.
"""

import numpy as np
import math
from typing import Tuple

from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from PySide6.QtCore import Qt

# Try to import VisPy, fallback to simple display if not available
VISPY_AVAILABLE = False
try:
    from vispy import scene
    from vispy.scene import visuals
    from vispy.visuals.transforms import MatrixTransform
    VISPY_AVAILABLE = True
except ImportError:
    pass
except Exception:
    pass


def quaternion_to_rotation_matrix(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """
    Convert quaternion to 4x4 rotation matrix.
    
    Convention: w, x, y, z (scalar first)
    """
    # Normalize
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n < 1e-10:
        return np.eye(4)
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    
    # Build rotation matrix
    rot = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw), 0],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw), 0],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy), 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)
    
    return rot


class AttitudeViewer(QWidget):
    """
    3D UAV attitude visualization widget.
    
    Hiển thị:
    - Mô hình UAV 3D với rotation theo quaternion
    - Grid reference
    - Trục tọa độ
    - Roll/Pitch/Yaw text overlay
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self._setup_ui()
        self._setup_3d_scene()
        
        # Current attitude
        self._qw = 1.0
        self._qx = 0.0
        self._qy = 0.0
        self._qz = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
    
    def _setup_ui(self):
        """Setup widget layout"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 3D canvas will be added here
        self._canvas_container = QWidget()
        layout.addWidget(self._canvas_container, 1)
        
        # Attitude text overlay
        info_layout = QHBoxLayout()
        info_layout.setContentsMargins(5, 5, 5, 5)
        
        self._roll_label = QLabel("Roll: 0.0°")
        self._pitch_label = QLabel("Pitch: 0.0°")
        self._yaw_label = QLabel("Yaw: 0.0°")
        
        for label in [self._roll_label, self._pitch_label, self._yaw_label]:
            label.setStyleSheet("font-family: monospace; font-size: 12px;")
            info_layout.addWidget(label)
        
        info_layout.addStretch()
        layout.addLayout(info_layout)
        
        # Quaternion display
        self._quat_label = QLabel("Q: [1.000, 0.000, 0.000, 0.000]")
        self._quat_label.setStyleSheet("font-family: monospace; font-size: 10px; color: gray;")
        layout.addWidget(self._quat_label)
    
    def _setup_3d_scene(self):
        """Setup VisPy 3D scene"""
        # Create canvas
        self._canvas = scene.SceneCanvas(
            keys='interactive',
            bgcolor='#1e1e1e',
            parent=self._canvas_container
        )
        
        # Add canvas to container
        canvas_layout = QVBoxLayout(self._canvas_container)
        canvas_layout.setContentsMargins(0, 0, 0, 0)
        canvas_layout.addWidget(self._canvas.native)
        
        # Create view with camera
        self._view = self._canvas.central_widget.add_view()
        self._view.camera = scene.TurntableCamera(
            elevation=30,
            azimuth=45,
            distance=5,
            fov=60
        )
        
        # Add grid
        grid = scene.visuals.GridLines(color=(0.3, 0.3, 0.3, 1))
        self._view.add(grid)
        
        # Add axes
        self._add_axes()
        
        # Add UAV model (simple box for now)
        self._add_uav_model()
    
    def _add_axes(self):
        """Add coordinate axes"""
        axis_length = 2.0
        
        # X axis (red)
        x_axis = scene.visuals.Line(
            pos=np.array([[0, 0, 0], [axis_length, 0, 0]]),
            color='red',
            width=2
        )
        self._view.add(x_axis)
        
        # Y axis (green)
        y_axis = scene.visuals.Line(
            pos=np.array([[0, 0, 0], [0, axis_length, 0]]),
            color='green',
            width=2
        )
        self._view.add(y_axis)
        
        # Z axis (blue)
        z_axis = scene.visuals.Line(
            pos=np.array([[0, 0, 0], [0, 0, axis_length]]),
            color='blue',
            width=2
        )
        self._view.add(z_axis)
    
    def _add_uav_model(self):
        """Add UAV model (simplified quadrotor shape)"""
        # Main body (box)
        body_vertices = np.array([
            # Front face
            [-0.3, -0.1, -0.05], [0.3, -0.1, -0.05], [0.3, 0.1, -0.05], [-0.3, 0.1, -0.05],
            # Back face
            [-0.3, -0.1, 0.05], [0.3, -0.1, 0.05], [0.3, 0.1, 0.05], [-0.3, 0.1, 0.05],
        ], dtype=np.float32)
        
        body_faces = np.array([
            [0, 1, 2], [0, 2, 3],  # Front
            [4, 6, 5], [4, 7, 6],  # Back
            [0, 4, 5], [0, 5, 1],  # Bottom
            [2, 6, 7], [2, 7, 3],  # Top
            [0, 3, 7], [0, 7, 4],  # Left
            [1, 5, 6], [1, 6, 2],  # Right
        ], dtype=np.uint32)
        
        body_colors = np.array([
            [0.2, 0.4, 0.8, 1.0],  # Blue body
        ] * len(body_vertices), dtype=np.float32)
        
        self._body_mesh = scene.visuals.Mesh(
            vertices=body_vertices,
            faces=body_faces,
            vertex_colors=body_colors
        )
        self._body_mesh.transform = MatrixTransform()
        self._view.add(self._body_mesh)
        
        # Arms (4 lines from center)
        arm_length = 0.6
        arm_positions = np.array([
            # Front-Right
            [0, 0, 0], [arm_length * 0.7, -arm_length * 0.7, 0],
            # Front-Left
            [0, 0, 0], [arm_length * 0.7, arm_length * 0.7, 0],
            # Back-Right
            [0, 0, 0], [-arm_length * 0.7, -arm_length * 0.7, 0],
            # Back-Left
            [0, 0, 0], [-arm_length * 0.7, arm_length * 0.7, 0],
        ], dtype=np.float32)
        
        self._arms = scene.visuals.Line(
            pos=arm_positions,
            color='gray',
            width=3,
            connect='segments'
        )
        self._arms.transform = MatrixTransform()
        self._view.add(self._arms)
        
        # Front indicator (arrow)
        front_arrow = np.array([
            [0.3, 0, 0], [0.5, 0, 0],
            [0.5, 0, 0], [0.45, 0.05, 0],
            [0.5, 0, 0], [0.45, -0.05, 0],
        ], dtype=np.float32)
        
        self._front_arrow = scene.visuals.Line(
            pos=front_arrow,
            color='red',
            width=3,
            connect='segments'
        )
        self._front_arrow.transform = MatrixTransform()
        self._view.add(self._front_arrow)
    
    def update_attitude(self, qw: float, qx: float, qy: float, qz: float,
                       roll: float, pitch: float, yaw: float):
        """
        Update UAV attitude.
        
        Args:
            qw, qx, qy, qz: Quaternion (normalized)
            roll, pitch, yaw: Euler angles in radians
        """
        self._qw, self._qx, self._qy, self._qz = qw, qx, qy, qz
        self._roll, self._pitch, self._yaw = roll, pitch, yaw
        
        # Update rotation matrix
        rot_matrix = quaternion_to_rotation_matrix(qw, qx, qy, qz)
        
        # Apply to all UAV components
        for visual in [self._body_mesh, self._arms, self._front_arrow]:
            visual.transform.matrix = rot_matrix
        
        # Update labels
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        self._roll_label.setText(f"Roll: {roll_deg:+.1f}°")
        self._pitch_label.setText(f"Pitch: {pitch_deg:+.1f}°")
        self._yaw_label.setText(f"Yaw: {yaw_deg:+.1f}°")
        self._quat_label.setText(f"Q: [{qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f}]")
        
        # Request redraw
        self._canvas.update()
    
    def reset_view(self):
        """Reset camera to default view"""
        self._view.camera.elevation = 30
        self._view.camera.azimuth = 45
        self._view.camera.distance = 5
