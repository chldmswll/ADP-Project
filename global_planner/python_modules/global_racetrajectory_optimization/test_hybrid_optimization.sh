#!/bin/bash
# Test script for MINTIME + MINCURV + CLOTHOID hybrid optimization

echo "=========================================="
echo "Hybrid Trajectory Optimization Test"
echo "=========================================="
echo ""

# Navigate to the global_racetrajectory_optimization directory
cd /home/misys/forza_ws/race_stack/planner/global_planner/python_modules/global_racetrajectory_optimization

echo "[1] Checking if clothoid_spline.py exists..."
if [ -f "global_racetrajectory_optimization/helper_funcs_glob/src/clothoid_spline.py" ]; then
    echo "✓ clothoid_spline.py found"
else
    echo "✗ clothoid_spline.py NOT found"
    echo "  Please ensure hybrid_optimizer.py is in place"
fi

echo ""
echo "[2] Checking main_globaltraj.py configuration..."
grep -n "use_hybrid_optimization" main_globaltraj.py | head -3

echo ""
echo "[3] Running trajectory optimization with hybrid method..."
echo ""

python3 main_globaltraj.py

echo ""
echo "[4] Check the output directory..."
echo "   Output files: outputs/"
ls -lah outputs/ | tail -5

echo ""
echo "=========================================="
echo "Test Complete!"
echo "=========================================="
