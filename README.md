# PX4 Offboard Acceleration/Velocity Control (px4_acc_ctrl)

This repository provides a high-performance C++17/ROS 2 interface for PX4-based UAVs, bridging advanced AI guidance laws with low-level flight control.

## Project Structure
- **src/velocity_tracking.cpp**: Core control logic for interception and tracking.
- **scripts/ai_telemetry_agent.py**: AI agent for real-time telemetry analysis (identifying stability issues).
- **config/control_params.yaml**: Tunable parameters for EWMA anomaly detection and PID gains.
- **test/**: Comprehensive unit tests for control logic verification.

## Features
- **AI-Driven Iteration**: Designed to iterate on control laws using Codex/GPT models.
- **Robustness**: EWMA filtering for outlier rejection in visual servoing.
- **Safety**: Velocity ramping and dynamic yaw smoothing for flight envelope protection.
