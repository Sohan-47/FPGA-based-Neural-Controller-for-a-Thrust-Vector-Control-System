# FPGA Neural Network Control System: Cart-Pole Baseline

> **Part of a larger project series on FPGA-based neural control systems**  
> This repository demonstrates the foundational implementation: training, quantizing, and deploying a neural network controller on FPGA hardware using the cart-pole balancing problem as a benchmark.

## Project Vision

This project serves as the **baseline implementation** for developing FPGA-based neural network control systems for dynamic stabilization problems. The cart-pole (inverted pendulum) problem provides an ideal starting point to:

- Validate the complete training-to-hardware deployment pipeline
- Develop fixed-point quantization techniques for resource-constrained hardware
- Establish hardware verification methodologies with physics-in-the-loop simulation
- Create a scalable architecture for more complex control problems

**Future Direction:** This baseline will be extended to thrust vector control (TVC) systems for rocket stabilization - see [Roadmap](#roadmap) section.

## Overview

This implementation trains a neural network using reinforcement learning (PPO) to balance an inverted pendulum, then deploys the trained model directly onto FPGA hardware using Verilog with fixed-point arithmetic. The system demonstrates real-time control with continuous force output, running entirely in hardware.

**Key Features:**
- Continuous action space (±10N force control)
- 8-neuron hidden layer with ReLU activation
- Hardware-optimized fixed-point implementation (Q11 format, scale factor: 2048)
- Physics simulation timestep: 20ms (50 Hz control loop)
- Real-time cocotb hardware verification
- Complete end-to-end pipeline from training to visualization

## System Architecture

```
Training (Python) → Weight Export → FPGA (Verilog) → Cocotb Simulation → Visualization
     ↓                    ↓                ↓                  ↓                ↓
  PPO Agent      Fixed-Point Conv.    Neural Net      Physics Loop      Animation
  (Gymnasium)    (Q7 weights)         (Single-cycle)   (20ms steps)      (50 FPS)
```

## File Descriptions

### 1. `smoothControl.py` - Neural Network Training

**Purpose:** Train a PPO (Proximal Policy Optimization) agent to balance the cart-pole using continuous force control.

**Key Components:**
- **SimpleContinuousCartPole**: Custom Gymnasium wrapper that converts the discrete CartPole environment to continuous action space (±1.0 scaled to ±10N)
- **Physics Engine**: Implements cart-pole dynamics with parameters matching the hardware simulation:
  - Cart mass: 1.0 kg
  - Pole mass: 0.1 kg
  - Pole length: 0.5 m
  - Gravity: 9.8 m/s²
  - Timestep (tau): 0.02 s
- **Neural Network**: 8-neuron hidden layer with ReLU activation
- **SimpleLogCallback**: Tracks episode rewards and automatically saves the best performing model
- **Weight Export**: Converts trained floating-point weights to fixed-point integers (scale: 128) for FPGA implementation

**Training Goal:** Maximize balance duration (max 500 steps = 10 seconds)

**Output:** 
- `best_model.zip`: Saved PPO model
- Quantized weights and biases printed to console for hardware implementation

### 2. `chipS.v` - FPGA Neural Network Controller

**Purpose:** Hardware implementation of the trained neural network in Verilog.

**Architecture:**
- **Inputs** (4x 16-bit signed, Q11 fixed-point):
  - `x_in`: Cart position (rocket position)
  - `x_dot_in`: Cart velocity
  - `theta_in`: Pole angle (rocket tilt)
  - `theta_dot_in`: Pole angular velocity
  
- **Output** (16-bit signed, Q11 fixed-point):
  - `force_out`: Control force (±10N range)

**Network Structure:**
- **Layer 1**: 8 neurons with ReLU activation
  - Input weights: 32 values (4 inputs × 8 neurons)
  - Biases: 8 values
  - Arithmetic: Q11 input × Q7 weight → Q18, then shift right 7 bits → Q11
  
- **Layer 2**: Linear output layer
  - Weights: 8 values
  - Bias: 1 value
  - Output: Continuous force value

**Fixed-Point Scaling:**
- Input/Output scale: 2048 (Q11) - 2048 = 1.0 in physical units
- Weight scale: 128 (Q7)
- Bias handling: Direct addition (no additional shift during training export)

**Design Decisions:**
- Single-cycle combinatorial logic with registered output
- Overflow protection with saturation
- ReLU implemented as conditional assignment
- Minimal resource usage for scalability

### 3. `testBenchS.py` - Cocotb Hardware Verification

**Purpose:** Verify the FPGA implementation using a cycle-accurate simulation with physics feedback loop.

**Simulation Flow:**
1. Initialize cart-pole with small perturbation (0.2 rad ≈ 11.5°)
2. For each timestep:
   - Send current state to hardware (Q11 format)
   - Read force output from hardware
   - Update physics model with the applied force
   - Log data for visualization
3. Check for failure conditions (tilt > 15°)
4. Run for 1000 steps (20 seconds simulated time)

**Key Functions:**
- `to_fixed()`: Convert floating-point to Q11 fixed-point (16-bit signed)
- `update_physics()`: Simulate cart-pole dynamics with applied force
- Logging: Periodic console output and CSV export

**Physics Parameters:** (Match training environment exactly)
- Gravity: 9.8 m/s²
- Cart mass: 1.0 kg
- Pole mass: 0.1 kg  
- Pole length: 0.5 m
- Integration timestep: 0.02 s

**Output:**
- `flight_data.csv`: Timestep, position, angle, force data
- Console logs with periodic status updates
- Pass/fail assertion based on stability

### 4. `SimulationRecord.py` - Visualization Tool

**Purpose:** Create an animated visualization of the hardware simulation results.

**Features:**
- Reads `flight_data.csv` generated by cocotb testbench
- Real-time animation at 50 FPS (matches 20ms physics timestep)
- Visual elements:
  - Blue cart with accurate position tracking
  - Red pole showing angle deflection (visual length: 1.0m)
  - Force arrow indicating control magnitude and direction
  - Real-time statistics display (step, position, tilt, force)
  - Ground line and grid for reference

**Rendering:**
- Cart dimensions: 0.5m × 0.3m
- Pole visual length: 1.0m (scaled for visibility)
- Track bounds: ±2.4m (matches environment limits)
- Force arrow scaling: 0.1m per Newton

## Workflow

### Step 1: Train the Neural Network
```bash
python smoothControl.py
```
- Trains PPO agent for up to 50,000 timesteps
- Saves best model automatically
- Prints quantized weights for hardware implementation

### Step 2: Update Hardware Weights
Copy the printed weights and biases from `smoothControl.py` output into `chipS.v`'s initial block.

### Step 3: Run Hardware Simulation
```bash
make  # or your cocotb makefile command
```
- Verifies FPGA implementation
- Generates `flight_data.csv`

### Step 4: Visualize Results
```bash
python SimulationRecord.py
```
- Plays back the hardware control session
- Visual verification of balance performance

## Technical Details

### Fixed-Point Arithmetic

**Q11 Format (Scale: 2048):**
- Range: -16.0 to +15.999
- Resolution: 1/2048 ≈ 0.000488
- Example: Physical value 1.5 → 3072 in hardware

**Q7 Format (Scale: 128):**
- Used for weights and biases
- Range: -256.0 to +255.992
- Resolution: 1/128 ≈ 0.0078

### Arithmetic Operations
- Multiplication: Q11 × Q7 = Q18, then shift right 7 → Q11
- Addition: Q11 + Q11 = Q11 (matching scales)
- Bias: Added directly in Q7, then scaled during multiplication phase

### Why These Design Choices?

**50 Hz Control Rate:**
- Sufficient for cart-pole benchmark validation
- Easily scalable to kHz rates for future applications
- Matches many real-world embedded control systems

**8-Neuron Architecture:**
- Minimal complexity while achieving reliable performance
- Low resource usage (~100-200 LUTs on typical FPGA)
- Provides foundation for scaling to larger networks

**Q11 Fixed-Point:**
- Good balance between range and precision for control applications
- Fits comfortably in 16-bit arithmetic
- Industry-standard format for embedded DSP

## Performance Metrics

- **Training:** Typical convergence around 10,000-30,000 timesteps
- **Best Performance:** 500 steps (maximum episode length)
- **Hardware Verification:** 1000 steps (20 seconds) with <15° tilt threshold
- **Control Frequency:** 50 Hz (every 20ms)
- **Force Range:** ±10 Newtons continuous
- **Resource Usage:** Minimal (<200 LUTs estimated for basic implementation)

## Dependencies

**Python:**
- gymnasium
- stable-baselines3
- numpy
- pandas
- matplotlib
- cocotb (for hardware simulation)
- torch (for stable-baselines3)

**Hardware:**
- Verilog simulator (Icarus Verilog, Verilator, etc.)
- cocotb framework

## Results

Simulation output can be viewed by running [SimulationRecord.py]

## Roadmap

This cart-pole implementation establishes the foundation for more advanced control systems:

### Phase 1: Cart-Pole Baseline (Current)
- [x] PPO training pipeline
- [x] Fixed-point quantization methodology
- [x] FPGA implementation in Verilog
- [x] Hardware verification with cocotb
- [x] Visualization tools

### Phase 2: Thrust Vector Control (Planned)
- [ ] Extend to 2-axis gimbal control
- [ ] IMU sensor integration (gyro + accelerometer)
- [ ] Multi-output neural network (2 TVC angles)
- [ ] 3D rigid body dynamics simulation

**Why This Progression?**
- Cart-pole physics directly translates to single-axis rocket stabilization
- Continuous force control → Thrust vectoring control
- Fixed-point arithmetic validated here scales to embedded flight computers
- Same training → hardware pipeline applies to TVC with increased complexity

This project demonstrates:
- **End-to-end ML deployment**: From training to hardware implementation
- **Fixed-point quantization**: Practical techniques for embedded ML
- **Hardware verification**: Physics-in-the-loop testing methodology
- **Control systems**: Real-time feedback control implementation
- **FPGA design**: Direct Verilog implementation (no HLS black boxes)


## Contributing

This is a baseline implementation for educational purposes. Suggestions for improvements, optimizations, or extensions are welcome!

## Author

Sohan Bag - 2nd Year ECE Student  
Part of a series on FPGA-based neural control systems

## Acknowledgments

- Cart-Pole Environment: OpenAI Gymnasium
- Reinforcement Learning: Proximal Policy Optimization (PPO) via Stable-Baselines3
- Hardware Verification: cocotb framework

---

**Note:** The terminology "rocket balanced" appears throughout the codebase as this project serves as the foundation for future thrust vector control applications. The physics and control principles transfer directly from cart-pole balancing to rocket stabilization.
