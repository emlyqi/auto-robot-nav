# Autonomous Robot Navigation Stack
End-to-end autonomous navigation stack in C++ / ROS 2 for a simulated differential-drive robot with 2D lidar. The robot receives a user-clicked goal and drives there autonomously — avoiding obstacles and remembering areas it's already explored.

Built for the [WATonomous ASD Admissions Assignment](./ASSIGNMENT.md).

## Demo
(sped up 5x)

https://github.com/user-attachments/assets/dafb7a27-f421-458e-be2f-24f7da117d24

## What it does
Four ROS 2 nodes, each handling one layer of the navigation stack:

| Node | Role | Key technique |
|---|---|---|
| **Costmap** | perception | Polar → grid conversion + obstacle inflation |
| **Map Memory** | world model | Persistent global map via 2D rigid-body transforms |
| **Planner** | global action | A\* path planning on occupancy grids |
| **Control** | local action | Pure pursuit with rotate-in-place behavior |

Each node is split into a ROS-facing layer (`*_node`) and a pure algorithm layer (`*_core`) for testability and reuse.

## Running
Prerequisites: Docker + Docker Compose.

```bash
git clone https://github.com/emlyqi/wato_asd.git
cd wato_asd
./watod up
```

Open Foxglove Studio, connect to `ws://localhost:9000`, and click a goal point on the 3D map. The robot plans a path and drives to it.
