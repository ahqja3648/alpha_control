#!/usr/bin/env python3
import os, sys
PKG_ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
if PKG_ROOT not in sys.path: sys.path.insert(0,PKG_ROOT)
from nodes.parking_planner_node import main
if __name__=='__main__': main()
