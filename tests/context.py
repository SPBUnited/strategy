# pylint: skip-file

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import bridge.processors.const as const
import bridge.processors.robot as robot
import bridge.processors.auxiliary as aux
import bridge.processors.field as field
import bridge.processors.router as router
import bridge.processors.strategy as strategy
import bridge.processors.waypoint as wp
import bridge.processors.quickhull as qh
import bridge.processors.tau as tau