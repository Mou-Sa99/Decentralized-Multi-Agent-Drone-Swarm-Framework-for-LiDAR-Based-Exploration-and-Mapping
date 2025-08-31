
# Swarm Control — Multi-Drone Mapping System

This project implements a **multi-agent swarm of drones in AirSim** for collaborative **3D structure mapping**.  
It combines:
- A **Scout Drone** → explores, detects obstacles, closes perimeters, and flags structures.  
- **Mapper Drones** → use flags to perform **layer-by-layer mapping** with LiDAR.  
- A **Shared Map** → common space where point clouds (`.ply`) and flags (`.json`) are stored.  

---

##System Requirements
- **OS:** Windows 10 / 11 (64-bit)  
- **GPU:** NVIDIA CUDA-capable (for Open3D acceleration, optional)  
- **RAM:** 16GB recommended  
- **Disk:** ~15 GB free space  
- **Simulator:** Microsoft AirSim (Unreal Engine 4.27)  

---

##  Versions Used
| Component         | Version |
|-------------------|---------|
| Python            | 3.10.x |
| AirSim Python API | 1.8.1 |
| Open3D            | 0.18.0 |
| CUDA              | 11.6 (if GPU acceleration used) |
| Visual Studio     | 2022 (MSVC v143) |
| CMake             | 3.27+ |
| NumPy             | 1.26.4 |
| OpenCV            | 4.9.0.80 |
| SciPy             | 1.13.1 |
| Matplotlib        | 3.9.0 |
| pandas            | 2.2.2 |
| psutil            | 5.9.8 |
| msgpack-rpc-python| 0.4.1 |

---

##Installation
```powershell
# Clone AirSim
git clone https://github.com/microsoft/AirSim.git
cd AirSim

# Setup Python environment
py -3.10 -m venv airsim-env
.\airsim-env\Scripts\activate
python -m pip install --upgrade pip

# Install dependencies
pip install airsim==1.8.1 open3d==0.18.0 numpy==1.26.4 opencv-python==4.9.0.80 ^
            scipy==1.13.1 matplotlib==3.9.0 pandas==2.2.2 psutil==5.9.8 ^
            msgpack-rpc-python==0.4.1
````

If you want GPU acceleration in Open3D:

* Install **CUDA 11.6**
* Install **Visual Studio 2022**
* Rebuild Open3D with CUDA enabled.

---

##Project Layout

```
AirSim/
└─ PythonClient/
   └─ multirotor/
      ├─ Swarm_Control/
      │   ├─ Scout_Drone/
      │   │   ├─ Common_Defs.py
      │   │   ├─ Obstacle_Detector.py
      │   │   ├─ Perimeter_Mapper.py
      │   │   ├─ Point_cloud_sorter.py
      │   │   ├─ Scout_Drone.py
      │   │   └─ Write_to_JSON.py
      │   │
      │   ├─ Mapper_Drone/
      │   │   ├─ Common_Defs.py
      │   │   ├─ Mapper_Drone.py
      │   │   ├─ Mapper_Drone_assembly.py
      │   │   └─ Surface_Tracer.py
      │
      ├─ shared_map/
      │   ├─ global_map.ply
      │   ├─ scout_drone.ply
      │   ├─ scout_drone1.ply
      │   ├─ structure_flags.json
      │   └─ visualizer.py
```

---

##Module Details

###Scout\_Drone

* **Common\_Defs.py** → LiDAR & math utilities.
* **Obstacle\_Detector.py** → obstacle detection from LiDAR scans.
* **Perimeter\_Mapper.py** → perimeter tracing & closure detection.
* **Point\_cloud\_sorter.py** → sorts/filters raw LiDAR point clouds.
* **Scout\_Drone.py** → main scout controller (entry point).
* **Write\_to\_JSON.py** → saves perimeter flags to `shared_map/structure_flags.json`.

###Mapper\_Drone

* **Common\_Defs.py** → utilities: LiDAR → NED, `.ply` I/O, surface normals, rotations.
* **Mapper\_Drone.py** → orchestrates two mapper drones in parallel (`multiprocessing`).
* **Mapper\_Drone\_assembly.py** → places mapper drones vertically (5 m apart).
* **Surface\_Tracer.py** → surface-tracing and `.ply` writing logic.

###shared\_map

* **global\_map.ply** → fused 3D map from mapper drones.
* **scout\_drone.ply / scout\_drone1.ply** → perimeter scans from scout.
* **structure\_flags.json** → perimeter flags written by scout.
* **visualizer.py** → Open3D-based tool to visualize `.ply` files.

---

##Usage

### 1. Run Scout Drone

Scout explores, closes perimeter, and flags structure:

```powershell
cd AirSim\PythonClient\multirotor\Swarm_Control\Scout_Drone
python Scout_Drone.py
```

Produces:

* `shared_map/scout_drone.ply`
* `shared_map/structure_flags.json`

---

### 2. Run Mapper Drones

Mapper drones assemble at flagged coordinates and perform mapping:

```powershell
cd AirSim\PythonClient\multirotor\Swarm_Control\Mapper_Drone
python Mapper_Drone.py
```

Produces:

* `shared_map/global_map.ply`

---

## Visualize Results

Open point cloud with Open3D:

```python
import open3d as o3d
pcd = o3d.io.read_point_cloud("shared_map/global_map.ply")
o3d.visualization.draw_geometries([pcd])
```

---

## Troubleshooting

* **`ModuleNotFoundError: airsim`** → run `pip install -e .` inside `AirSim/PythonClient`.
* **PLY not saving** → ensure `shared_map/` exists and path is correct.
* **Multiprocessing stuck** → keep `if __name__ == "__main__":` guard in `Mapper_Drone.py`.
* **Drone not rotating** → adjust yaw threshold in `rotate_drone_toward_normal`.
* **Altitude stepping too large** → edit `Mapper_Drone_assembly.py` (default 10 m per iteration).

---

## Workflow Summary

1. Launch **AirSim Blocks** (or another environment).
2. Run **Scout Drone** → perimeter `.ply` + flags `.json`.
3. Run **Mapper Drones** → 3D surface `.ply`.
4. Visualize results in **Open3D/CloudCompare**.

```
```
