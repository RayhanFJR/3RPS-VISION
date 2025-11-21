import time
import numpy as np

def test_angle_calculation_speed():
    """Benchmark angle calculation"""
    from vision.AngleCalculator import AngleCalculator
    
    start = time.time()
    
    for i in range(10000):
        result = AngleCalculator.calculate_angle(
            (100, 200), (150, 300), (200, 250)
        )
    
    elapsed = time.time() - start
    per_call = elapsed / 10000 * 1000  # ms
    
    print(f"Angle calculation: {per_call:.3f} ms per call")
    assert per_call < 1.0  # Should be <1ms

def test_buffer_operations_speed():
    """Benchmark buffer operations"""
    from utils.DataBuffer import DataBuffer
    
    buf = DataBuffer(max_size=1000)
    
    start = time.time()
    
    for i in range(100000):
        buf.add(float(i))
        _ = buf.get_average()
    
    elapsed = time.time() - start
    
    print(f"Buffer ops: {elapsed:.3f}s for 100k iterations")
    assert elapsed < 5.0  # Should complete in <5s

# Run: pytest tests/performance_test.py -v