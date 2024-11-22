# requirements
- isaac sim installed from omniverse app
- optional installation: 
    - omniverse cache for performance
    - for meta quest 3: install
        - "omni.physx.fabric" (omniverse/isaacsim) extension, currently required by isaacSimPublisher
            - the code should enable this ext automatically
        - unity w/ Meta XR All-in-One SDK
        - extra: meta xr simulator(unity) for running on PC
        - https://github.com/intuitive-robots/IRXR-Unity/tree/meta-quest3-dev to run on meta quest
        - and https://github.com/intuitive-robots/SimPublisher/tree/isaacsim to communicate with meta quest over tcp protocol
            - install simpublisher (as editable)
                ```
                cd OMNIVERSE_ISAACSIM_PATH
                python -m pip install -e "PATH_TO\SimPublisher"
                ```

# setup
1. unzip into a folder
2. create softlink: `ln -s "PATH_TO_UNZIPPED_FOLDER" "PATH_TO_ISAAC_SIM\exts\omni.isaac.examples\omni\isaac\examples\user_examples\franka_rope"`
3. add `from .franka_rope import *` to `PATH_TO_ISAAC_SIM\exts\omni.isaac.examples\omni\isaac\examples\user_examples\__init__.py`
4. profit! 
    ![](isaacsim.png)
    ![](vr.png)
    ![](metaquest3.mkv)

# code overview
- `class FrankaRope(BaseSample):` handles core logic and isolated events including simulation
    - BaseSample is stateful and provides life cycle management
- `class ControlFlow:` handles basic platform agnostic basic control logic
- `class IsaacUIUtils(ControlFlow):` and `class VRUIUtils(ControlFlow):` are for complex interdependent button events/states
- `class RigidBodyRope:` is based on the reference implementation `PATH_TO_ISAAC_SIM\extsphysics\omni.physx.demos\omni\physxdemos\scenes\RigidBodyRopesDemo.py`
    - it does not use the `pointinstancer` since 
        - `isaacsimpublisher` currently rely on the `fabric` backend
        - and current implementation of `fabric` (v106.1.9) backend had runtime issue with `pointinstancer`
        > Point instancer rigid bodies are now not updated through fabric extension. Runtime changes to the scene composition might not be correctly reflected in fabric and maybe lead into issues.

## vr buttons
- press X to re-init the stage including the shape of the rope
- press Y to start and stop/save the following and recording task
    - re-init the stage without stop the task first will **discard** current recording
- holding A to recalibrate the pose of the vr controller w/o updating to the gripper
- press B to pause/unpause the simulation
- press left/right index trigger to open/close the parallel gripper

# fix
xr simulator will only launch once, unless it was destroyed properly:
- follow https://communityforums.atmeta.com/t5/Unity-Development/Meta-XR-Simulator-starts-only-once/td-p/1141806 to add life cycle management to `Assets\SceneLoader\Scripts\RigidObjectsController.cs` in `IRXR-Unity`

---
weird **bugs** in isaac sim/omniverse
- clean cache and/or data using: bat or sh: `isaac-sim.bat --clear-data --clear-cache`
    - **WARNING: ISAACSIM WILL RESET TO DEFAULT, DATA/CACHE WILL LOST**
- for reproducibility: test the code after cleaning the cache + data