#!/usr/bin/env python3
"""Compare nonempty variable-length MergeBrushIndices arrays on live-slot paths.

Orders that would access a destroyed native vector slot are checked for atomic
rejection in the portable API and are excluded from original-code execution.
"""
from compare_merge_brush_pair import main

if __name__ == '__main__': main(general=True)
