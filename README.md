# PathWeaver MPC Simulator Documentation

**Created by: Idriss Boukmouche (GitHub: terminaldz)**

### I. Overview

The **PathWeaver MPC Simulator** is a browser-based, real-time autonomous vehicle motion planning and simulation platform. It leverages WebGL and Three.js to demonstrate advanced path planning algorithms, specifically focusing on a lattice-based approach combined with Model Predictive Control (MPC) for vehicle guidance. The simulator allows users to create and run various driving scenarios, observe the vehicle's decision-making process, and interact with different control modes.

The system is designed to perform complex calculations typically handled by high-performance computing environments (like C++/CUDA) directly within the web browser, making advanced autonomous driving concepts accessible and visual.

### II. How to Run the Project

To run the PathWeaver MPC Simulator, follow these steps:

1.  **Prerequisites:**
    *   **Node.js and npm:** Ensure you have Node.js and npm (Node Package Manager) installed on your system. You can download them from [nodejs.org](https://nodejs.org/).
    *   **Supported Browser:** Google Chrome is highly recommended for full functionality due to its WebGL 2 and OffscreenCanvas support in Web Workers.
        *   **Enable Hardware Acceleration:** In Chrome, go to `chrome://settings/system` and ensure "Use hardware acceleration when available" is enabled.
        *   **Enable Experimental Canvas Features:** In Chrome, go to `chrome://flags/#enable-experimental-canvas-features` and enable this flag. Relaunch Chrome after changing flags.

2.  **Download or Clone the Project:**
    *   If you have the project files as a ZIP, extract them to a local directory.
    *   Alternatively, if it's a Git repository, clone it:
        ```bash
        git clone https://github.com/TerminalDZ/PathWeaver-MPC-Simulator.git
        cd pathweaver-mpc-simulator 
        ```

3.  **Install Dependencies:**
    *   Open a terminal or command prompt in the root directory of the project (where `package.json` is located).
    *   Run the following command to install the necessary development dependencies (like Webpack):
        ```bash
        npm install
        ```

4.  **Build the Project:**
    *   After dependencies are installed, build the project using the Webpack script defined in `package.json`:
        ```bash
        npm run build
        ```
        This command will typically use Webpack to bundle the JavaScript files and place the output in a `dist` directory (e.g., `dist/Dash.js` and `dist/PathPlannerWorker.js`). The command `set NODE_OPTIONS=--openssl-legacy-provider` is included for compatibility with newer Node.js versions on some systems when older Webpack versions are used; it might not be strictly necessary on all setups.

5.  **Open in Browser:**
    *   Once the build is complete, you can open the `index.html` file directly in your Google Chrome browser.
    *   Navigate to the project directory in your file explorer and double-click `index.html`, or use the "Open File..." option in your browser.

6.  **(Optional) Development Mode - Watch for Changes:**
    *   If you plan to make changes to the code and want them to be automatically rebuilt, you can run the watch script:
        ```bash
        npm run watch
        ```
        This will keep Webpack running in the background, and it will re-bundle the files whenever it detects a change in the source code. You'll still need to refresh the `index.html` page in your browser to see the changes.

### III. Core Functionality and How It Works

The simulator operates in several key stages:

1.  **Scenario Definition (Editor):**
    *   Users can define a road layout using anchor points to create a Catmull-Rom spline, forming the centerline of a two-lane road (`js/autonomy/LanePath.js`, `js/simulator/Editor.js`).
    *   Static obstacles (e.g., parked cars, debris) can be placed on the map (`js/autonomy/StaticObstacle.js`, `js/simulator/Editor.js`).
    *   Dynamic obstacles (e.g., other vehicles, cyclists, pedestrians) can be defined with initial positions (in station-latitude coordinates relative to the road) and velocities (`js/autonomy/DynamicObstacle.js`, `js/simulator/DynamicObstacleEditor.js`).
    *   Initial vehicle speed, speed limits, and lane preferences are configurable via the UI, managed by `js/simulator/Editor.js` and `js/simulator/PathPlannerConfigEditor.js`.

2.  **Path Planning (`js/autonomy/path-planning/PathPlanner.js` & `js/GPGPU.js`):**
    *   This is the heart of the autonomous mode. When the vehicle is in "Autonomous" or "MPC" mode, the `PathPlanner` is invoked.
    *   **SL Coordinates & Lattice:** The planner primarily operates in a station-latitude (SL) coordinate system relative to the defined road.
        *   **Station (S):** Longitudinal distance along the road's centerline.
        *   **Latitude (L):** Lateral offset from the centerline.
    *   **State Lattice (`js/autonomy/path-planning/RoadLattice.js`):** A graph is constructed in this SL space. Nodes in this lattice represent potential future states of the vehicle (position, orientation, curvature) at discrete stations and latitudes.
    *   **Path Generation (Polynomial Spirals):**
        *   **Cubic & Quintic Spirals:** To connect the vehicle's current state to the lattice, and to connect lattice nodes themselves, polynomial spiral curves are used. These curves ensure continuous curvature, which is essential for smooth and physically achievable steering.
            *   `js/autonomy/path-planning/CubicPath.js`: Defines paths where curvature is a cubic polynomial of arc length.
            *   `js/autonomy/path-planning/QuinticPath.js`: Defines paths where curvature is a quintic polynomial, allowing matching of higher-order derivatives for smoother transitions.
        *   **Optimization:** The parameters of these spiral paths are optimized (e.g., using Newton's method or relaxation iterations as seen in `CubicPath.js` and the GPGPU kernels like `js/autonomy/path-planning/gpgpu-programs/optimizeCubicPaths.js`, `js/autonomy/path-planning/gpgpu-programs/optimizeQuinticPaths.js`) to meet the boundary conditions.
    *   **Obstacle Representation & Cost Grids (GPGPU):**
        *   **XY Obstacle Grid (`js/autonomy/path-planning/gpgpu-programs/xyObstacleGrid.js`):** Static obstacles are rasterized into a 2D grid in Cartesian (XY) coordinates.
        *   **SL Obstacle Grid (`js/autonomy/path-planning/gpgpu-programs/slObstacleGrid.js`):** This XY grid is transformed into an SL-space occupancy grid.
        *   **Dilation (`js/autonomy/path-planning/gpgpu-programs/slObstacleGridDilation.js`):** Obstacles in the SL grid are "dilated" to create collision and hazard zones.
        *   **Dynamic Obstacle Grid (`js/autonomy/path-planning/gpgpu-programs/slDynamicObstacleGrid.js`):** Dynamic obstacles' predicted SL positions over future time frames are rasterized into a 3D SLT (Station-Latitude-Time) grid.
        *   **XYSL Map (`js/autonomy/path-planning/gpgpu-programs/xyslMap.js`):** A mapping for efficient conversion between XY and SL coordinates.
    *   **Graph Search (`js/autonomy/path-planning/gpgpu-programs/graphSearch.js` GPGPU Kernel):**
        *   **Multi-dimensional Nodes:** The lattice nodes are augmented into a 5D state space: (Station, Latitude, Time Range, Velocity Range, Acceleration Profile).
        *   **Dynamic Programming:** The planner uses dynamic programming to find the least-cost path through this 5D state lattice, iterating station by station.
        *   **Cost Function:** The cost of traversing an edge includes factors like proximity to obstacles, lane deviation, speed limit adherence, control effort, and time.
        *   **Output:** The GPGPU `graphSearch` kernel outputs a cost table. The CPU then reconstructs the optimal trajectory.
    *   **WebGL for Parallelism (`js/GPGPU.js`):** This class manages WebGL contexts and shaders for parallel computations. Textures are used for data I/O with shaders.

3.  **Vehicle Control:**
    *   Once a path is planned, a controller makes the vehicle follow it.
    *   **`js/autonomy/control/ManualController.js`:** Direct user keyboard control.
    *   **`js/autonomy/control/AutonomousController.js` / `js/autonomy/control/FollowController.js` (PID/Stanley-like):**
        *   A traditional feedback/feedforward controller that localizes the car on the planned path and uses PID-like logic for longitudinal control (speed) and Stanley-like geometric logic for lateral control (steering).
    *   **`js/autonomy/control/MPCController.js` (Model Predictive Control):** This is the most advanced controller.
        *   **Model-Based Prediction:** It uses a kinematic bicycle model (`predictState` method within `MPCController.js`) to predict the car's future trajectory over a `predictionHorizon` (N steps of `mpc_dt` each).
        *   **Cost Function:** For each predicted trajectory, it calculates a cost (`calculateCost` method) based on deviations from a reference path (derived from the globally planned path by `PathPlanner`), control effort (steering, acceleration), and control rates (change in steering/acceleration). The `this.weights` object defines the importance of each factor.
        *   **Optimization:** At each simulation step (`control` method), it performs an optimization. The current implementation (`optimizeControl` method) uses a grid search over a discretized set of possible initial control actions (steering angle and acceleration value). For each candidate initial action:
            *   It simulates the car's trajectory over the `predictionHorizon`, assuming this initial action is taken and (in this simplified MPC) potentially held constant or a simple policy is followed for subsequent steps in the horizon.
            *   It evaluates the total discounted cost of this predicted trajectory.
        *   The initial control action pair that leads to the minimum total cost is selected.
        *   **Receding Horizon:** Only the *first* control action (steering and acceleration) from this "optimized" plan is actually applied to the car for the current simulation time step (`dt_sim`). The entire optimization process is repeated at the next time step using the car's new state.
        *   **Reference States (`getReferenceState`):** For each step `k` within its internal prediction horizon, the MPC controller queries the globally planned path (from `PathPlanner`) to get a target state (position, orientation, velocity) for time `currentCarTimeOnPath + (k+1)*mpc_dt`. This `currentCarTimeOnPath` is an estimation of how far, in terms of time, the car has already progressed along the global path.
        *   **Control Output:** The optimized actual steering angle and acceleration value are converted into normalized `gas` (0-1), `brake` (0-1), and `steer` (rate, -1 to +1) commands suitable for the `Car.js` model.
        *   **Key Advantage:** MPC's predictive nature allows it to anticipate future conditions (like curves in the reference path) and make smoother, more proactive control decisions compared to purely reactive controllers. It systematically balances multiple objectives defined by the cost function.

4.  **Physics Simulation (`js/physics/Physics.js`, `js/physics/Car.js`):**
    *   `Car.js` defines the vehicle's physical properties and update rules.
    *   `step(dt)` updates the car's state based on applied controls and physics.
    *   `update(controls, dt)` translates normalized control inputs into car's internal acceleration and wheel angular velocity.

5.  **Visualization (Three.js & Dashboard):**
    *   `js/Simulator.js` orchestrates the scene, rendering, and UI.
    *   `js/objects/` contains visual representations for car, obstacles.
    *   `js/simulator/Dashboard.js` displays telemetry.
    *   Camera controls (`js/simulator/OrbitControls.js`, `js/simulator/TopDownCameraControls.js`) provide views.

### IV. Technology Stack and Usage

*   **JavaScript (ES6 Modules):** Primary language for simulator logic, UI, control.
*   **Three.js (`vendor/three.js`, `vendor/THREE.MeshLine.js`):** 3D graphics library for scene creation, WebGL rendering, camera/light management. `THREE.MeshLine` for smooth path visualization.
*   **WebGL (via Three.js and `js/GPGPU.js`):** For rendering and General-Purpose GPU computing. `GPGPU.js` abstracts shader-based parallel computations, using textures for data.
*   **GLSL (OpenGL Shading Language):** Used in `js/autonomy/path-planning/gpgpu-programs/` for parallel computation kernels (obstacle grids, path optimization, graph search).
*   **HTML/CSS (Bulma CSS Framework):** `index.html` for page structure, `css/dash.css` for custom styles, `vendor/bulma.min.css` for UI components.
*   **Web Workers (`workers/PathPlannerWorker.js`):** Runs the `PathPlanner` (with GPGPU) in a separate thread to prevent UI freezing.
*   **Webpack (`webpack.config.js`):** Module bundler for packaging JavaScript into `dist/Dash.js` and `dist/PathPlannerWorker.js`.
*   **Node.js & npm (`package.json`):** For development environment, dependencies, and build scripts.

### V. Project Execution Flow (Simplified Autonomous/MPC Mode)

1.  **Initialization (`js/Simulator.js`):** Sets up Three.js, physics, UI, starts `PathPlannerWorker`, begins animation loop.
2.  **Scenario Definition/Load:** User creates or loads a scenario, defining `lanePath`, obstacles, speeds.
3.  **Simulation Loop (`js/Simulator.js` - `step` method):**
    *   If `plannerReady`: `startPlanner()` sends current state to `PathPlannerWorker`.
    *   **In `PathPlannerWorker.js`:** `PathPlanner.plan()` uses GPGPU for grids, path optimization, graph search; posts planned path back.
    *   **Back in `js/Simulator.js` - `receivePlannedPath()`:** Updates `plannerReady`, processes path, updates active controller (`AutonomousController` or `MPCController`) with the new path.
    *   **Controller Logic:**
        *   'manual': `ManualController.control()`.
        *   'autonomous': `AutonomousController.control()` (or `FollowController`).
        *   'mpc': `MPCController.control()`.
    *   **Vehicle Physics Update:** `this.car.update()`, `this.physics.step()`.
    *   **UI & Rendering:** Update dynamic objects, cameras, dashboard, render scene.
    *   Loop via `requestAnimationFrame`.

