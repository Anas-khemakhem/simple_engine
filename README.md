# ⚡ Advanced 2D Computational Kinematics & Aerodynamics Engine

<div align="center">

![C++](https://shields.io)
![Raylib](https://shields.io)
![CMake](https://shields.io)
![License](https://shields.io)

**A high-performance C++ simulation engine implementing symplectic numerical integration, iterative constraint solvers, and lift-induced aerodynamics.**

</div>

---

## 🚀 Overview
This project is a strictly logic-driven, algorithmic physics engine designed for deterministic kinematic simulations. Moving beyond simple particle systems, it features a **Sequential Impulse Solver** to handle rigid/soft body constraints and a **Symplectic Forward-Predictor** for real-time trajectory verification.

Rendering is decoupled from the physics core, utilizing a Raylib-based 2D-into-3D projection plane to observe complex physical interactions at high fidelity.

## 🧠 Technical Implementation

### 1. Numerical Integration: Symplectic Euler
To ensure long-term energy conservation in oscillatory systems (like mass-spring lattices), the engine utilizes **Semi-Implicit (Symplectic) Euler** integration. By updating velocity before position, the solver remains volume-preserving in phase space, preventing the numerical "explosion" typical of standard explicit methods.

### 2. Constraint & Impulse Solver
The engine models contacts as non-linear constraints resolved via **Sequential Impulses**. 
- **Manifold Generation:** Accurate calculation of contact normals, penetration depths, and contact points.
- **Iterative Convergence:** A Gauss-Seidel approach iterates over all active manifolds to converge on a stable global state, effectively resolving resting contacts and stacked bodies.
- **Baumgarte Stabilization:** Used for positional correction to mitigate "sinking" without introducing artificial kinetic energy.

### 3. Aerodynamics & Magnus Effect
Includes real-time fluid dynamics approximations for moving projectiles:
- **Rayleigh Drag:** Quadratic velocity-based air resistance.
- **Magnus Lift Tensor:** Real-time calculation of lift induced by angular velocity, allowing for realistic "curve-ball" trajectories based on the spin ratio of the body.

### 4. Deterministic Trajectory Predictor
A high-speed "Ghost" system that performs a multi-step numerical look-ahead. It fast-forwards the engine's exact integration logic (including drag and lift) to project a mathematically perfect future state of the system before it occurs.

## ⚡ Key Features
- **Rigid Body Dynamics:** Full support for mass, inertia, and rotational friction.
- **Soft Body Lattice:** Mass-spring systems with cross-bracing for structural integrity.
- **Fixed-Timestep Accumulator:** Decoupled physics logic ($120$ Hz) to guarantee deterministic behavior across different hardware.
- **Optimized Memory Layout:** Flat-vector allocations to maximize CPU cache coherency during the integration loop.

## 🛠 Build & Requirements
- **Standard:** C++17 or higher.
- **Dependencies:** [Raylib](https://raylib.com).

```bash
# Compilation example
g++ main.cpp -o physics_sim -std=c++17 -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
```

## 📜 License
This project is licensed under the **MIT License** - see the LICENSE file for details.
