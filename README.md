# CP Physics Engine 2.5D 🚀

A highly optimized, custom-built 2D physics engine written completely from scratch in C++ and rendered in a 3D environment using **Raylib**. This project demonstrates advanced mathematical concepts in game physics, including rotational dynamics, iterative impulse resolution, and constraint-based soft bodies.

## ✨ Features

- **Custom Physics Solver:** Fixed-timestep integration ensuring rock-solid numerical stability (no spirals of death).
- **Rigid Body Dynamics:** Support for Circle and AABB (Axis-Aligned Bounding Box) colliders.
- **Angular/Rotational Physics:** Calculates moment of inertia, torque, and angular velocity. Objects tumble and rotate accurately upon off-center collisions.
- **Constraint System:** Distance joints and spring forces enable the creation of soft bodies (squishy cubes) and complex structures like suspension bridges.
- **Friction & Restitution:** Tunable bounciness and realistic rotational friction across surfaces.
- **Interactive First-Person Camera:** Free-look camera system to fly around the simulation.
- **Projectile Interaction:** Left-click to fire high-velocity "tungsten spheres" into the physics environment to test stability and watch structures collapse!

## 🎮 Controls

Your cursor is locked to the window for first-person interaction.

- **`W` `A` `S` `D`** - Move around the lab
- **`Mouse`** - Look around
- **`Left Click`** - Shoot a high-velocity physics sphere
- **`ESC`** - Exit the simulation

## 🛠️ Build Instructions

### Prerequisites
- **C++ Compiler:** `g++` (MinGW-w64 on Windows)
- **Library:** `Raylib`

### Compiling on Windows

Open your terminal in the project directory and run the following command:

```powershell
g++ -O3 main.cpp -o physics_sim.exe -lraylib -lgdi32 -lwinmm
```

Then run the executable:

```powershell
.\physics_sim.exe
```

## 🧠 Under the Hood

The engine uses an **Iterative Impulse-based Solver**. For every physics step:
1. Iterates over all bodies, integrating forces and gravity to calculate new velocities.
2. Checks for collisions (Circle vs Circle, AABB vs AABB, Circle vs AABB) and creates collision manifolds.
3. Resolves collisions over multiple iterations (default 15) to calculate linear and rotational impulses exactly at the point of contact.
4. Applies positional correction to mitigate floating-point errors and sinking.
