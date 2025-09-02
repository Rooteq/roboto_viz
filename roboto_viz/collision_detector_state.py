#!/usr/bin/env python3
"""
Collision Detector State message definition
This is a simple Python-based message structure to handle CollisionDetectorState
until the proper ROS2 message interface is available.
"""

class CollisionDetectorState:
    """
    Simple CollisionDetectorState message definition
    
    # Name of configured polygons
    string[] polygons
    # List of detections for each polygon  
    bool[] detections
    """
    
    def __init__(self):
        self.polygons = []     # List of polygon names (string[])
        self.detections = []   # List of detection bools (bool[])
