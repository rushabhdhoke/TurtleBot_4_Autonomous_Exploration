# Algorithm Comparison Repository - Structure

This repository provides **modular SLAM exploration algorithms** for comparative analysis.

**Same Environment + Different Algorithms = Clear Comparison**

## 📂 Structure

```
demo_bot/
├── algorithms/          # 🔥 All exploration algorithms
│   ├── frontier_explorer.py       # ✅ DONE - Greedy frontier-based
│   ├── astar_explorer.py          # TODO - A* path planning
│   ├── bfs_explorer.py            # TODO - Breadth-First Search
│   ├── dfs_explorer.py            # TODO - Depth-First Search
│   ├── greedy_explorer.py         # TODO - Pure greedy (closest first)
│   ├── ekf_explorer.py            # TODO - Extended Kalman Filter SLAM
│   ├── ukf_explorer.py            # TODO - Unscented Kalman Filter
│   ├── particle_filter.py         # TODO - MCL-based exploration
│   └── vslam_explorer.py          # TODO - Visual SLAM with camera
│
├── launch/
│  ├── headless_sim.launch.py      # Same environment for all
│   └── start_exploring.launch.py  # Algorithm selector
│
└── config/
    ├── nav2_fast_frontier.yaml    # Algorithm-specific params
    ├── nav2_astar.yaml            # TODO
    └── ...
```

## 🎯 Usage

**1. Launch Environment** (Same for all algorithms):
```bash
ros2 launch demo_bot headless_sim.launch.py world:=maze
```

**2. Launch Algorithm** (Swap here):
```bash
# Frontier Explorer (Current)
ros2 launch demo_bot start_exploring.launch.py algorithm:=frontier

# A* Explorer (Future)
ros2 launch demo_bot start_exploring.launch.py algorithm:=astar

# BFS Explorer (Future)
ros2 launch demo_bot start_exploring.launch.py algorithm:=bfs
```

## 📊 Comparison Metrics

Each algorithm will output:
- **Exploration Time** (target: <5 mins)
- **Map Coverage** (target: >98%)
- **Path Length** (total distance traveled)
- **Computational Cost** (CPU/memory usage)

## 🏆 Algorithm Categories

### Basic (Graph Search)
- ✅ **Frontier Explorer** - Done! (Greedy frontier-based)
- 🔲 **A*** - Optimal path planning
- 🔲 **BFS** - Breadth-first exploration
- 🔲 **DFS** - Depth-first exploration  
- 🔲 **Pure Greedy** - Closest frontier only

### Medium (Probabilistic)
- 🔲 **EKF SLAM** - Extended Kalman Filter
- 🔲 **UKF SLAM** - Unscented Kalman Filter
- 🔲 **Particle Filter** - Monte Carlo Localization

### Advanced (Multi-Sensor)
- 🔲 **Visual SLAM** - Camera-based
- 🔲 **Lidar + Camera Fusion** - Best of both worlds
- 🔲 **Learning-Based** - Reinforcement Learning (optional)

## 🚀 Current Status

**Frontier Explorer** 
- Speed: 3.0 m/s
- Features: Directional commitment + cleanup mode
- Status: Ready for testing

Ready to implement next algorithm!
