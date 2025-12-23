"""
Sensor fusion base interface

Provides a common interface for sensor fusion algorithms.
Currently implements Mahony AHRS.
"""

from .mahony import fusion_init, fusion_update, fusion_read

# Re-export fusion functions for easy import
__all__ = ['fusion_init', 'fusion_update', 'fusion_read']
