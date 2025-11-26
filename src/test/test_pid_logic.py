import pytest
from morphing_airfoil.pid import PIDController

def test_pid_initialization():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, min_out=0, max_out=100)
    assert pid.kp == 1.0
    assert pid.prev_error == 0.0
    assert pid.integral == 0.0

def test_proportional_control():
    # If error is 10, Kp is 0.5, output should be 5
    pid = PIDController(kp=0.5, ki=0.0, kd=0.0, min_out=-100, max_out=100)
    
    setpoint = 20.0
    measurement = 10.0
    dt = 1.0
    
    output = pid.update(setpoint, measurement, dt_actual=dt)
    assert output == 5.0

def test_saturation_limits():
    # Test that output clamps to max_out
    pid = PIDController(kp=100.0, ki=0.0, kd=0.0, min_out=0, max_out=10)
    
    setpoint = 100.0
    measurement = 0.0
    dt = 1.0
    
    output = pid.update(setpoint, measurement, dt_actual=dt)
    assert output == 10.0 # Should be clamped at 10, not 10000

def test_integral_accumulation():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, min_out=-100, max_out=100)
    
    setpoint = 10.0
    measurement = 0.0
    dt = 1.0
    
    # First step: error=10, integral=10, output=10
    out1 = pid.update(setpoint, measurement, dt_actual=dt)
    assert out1 == 10.0
    
    # Second step: error=10, integral=20, output=20
    out2 = pid.update(setpoint, measurement, dt_actual=dt)
    assert out2 == 20.0

def test_derivative_kick():
    # Test reaction to sudden change
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, min_out=-100, max_out=100)
    
    setpoint = 10.0
    # prev_error starts at 0. 
    # Current error = 10. derivative = (10-0)/1 = 10. output = 10.
    out1 = pid.update(setpoint, 0.0, dt_actual=1.0)
    assert out1 == 10.0