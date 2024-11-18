# simplified in 1 file, no more extension boilerplate

# NICE TO HAVE
# TODO deformable support
# TODO numpy to torch / warp: sim and phy on GPU
# TODO speed limit, or better ik support: currently: NaN when to fast
# TODO use rope factory to pre-generate ropes in background in another stage

# NEXT 
# TODO replay saved json
# TODO rewrite isaacsimpublisher: to support openusd backend, to support visuals, to support rotations

# To launch
# TODO not ifdef vr:
# isaac only needs to bind the input once on startup
#IsaacUIUtils.setUp()

# TODO ifdef vr:
#VRUIUtils.setUp()

# the actual code
import carb

from omni.isaac.examples.base_sample import BaseSample
import numpy as np

from omni.isaac.franka import KinematicsSolver 
from omni.isaac.franka.tasks import FollowTarget
from omni.isaac.core.objects import VisualCuboid

from pxr import PhysxSchema

# TODO ifdef vr: import them
from simpub.sim.isaacsim_publisher import IsaacSimPublisher
from simpub.xr_device.meta_quest3 import MetaQuest3
import carb.events
import omni.kit.app

from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.stage import get_stage_units
from datetime import datetime

# world is a SimulationContext subclass
# stage uses kit app, a helper class
# scene is object subclass
# robot in articulation in single_prim_wrapper

# BaseSample is preconfigurated with phyicsScene
# methods to implement:
#     def setup_scene(cls, scene: Scene) -> None:
#     async def setup_post_load(cls):
#     async def setup_pre_reset(cls):
#     async def setup_post_reset(cls):
#     async def setup_post_clear(cls):
class FrankaRope(BaseSample):
    def _init_vars(self):
        try:
            del self._old_observations
            del self._pre_physics_callback
            del self._post_physics_callback
            del self._ropeScenePath
            del self._ropePrimPath
        except:
            pass
        self._old_observations=None
        self._pre_physics_callback=None
        self._post_physics_callback=None
        self._ropeScenePath=None
        self._ropePrimPath=None

    def __init__(self) -> None:
        super().__init__()
        self._init_vars()
        self._world_settings = {
            "physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0,
            "physics_prim_path": "/PhysicsScene", "sim_params":None, "set_defaults":True, "backend":"numpy","device":None
        }

        return
        
    async def setup_pre_reset(self) -> None:
        world = self._world
        self._init_vars()
        if world.physics_callback_exists("sim_step"):
            world.remove_physics_callback("sim_step")
        # self._robot_articulation_solver.reset()
        return

    def world_cleanup(self):
        try:
            del self._robot_articulation_solver
        except:
            pass
        self._robot_articulation_solver = None
        return
    
    # should add tasks here
    # BUT: Err set up scene:
    # Cannot add the object target to the scene since its name is not unique
    def setup_scene(self) -> None:
        world = self._world
        world.clear()


        # PhysX error: The application needs to increase PxGpuDynamicsMemoryConfig::foundLostAggregatePairsCapacity to 1101
        PhysicsScene = world.stage.GetPrimAtPath("/PhysicsScene")
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(PhysicsScene)
        physxSceneAPI.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(10 * 1024)
   
        # tested on amd epyc 7543p w/ 8 threads + a4500 20gb vram: cpu solver is actually faster
        # api from usdrt\scenegraph\usd\physxSchema\physxSceneAPI.h
        physxSceneAPI.CreateCollisionSystemAttr().Set("PGM")
        physxSceneAPI.CreateSolverTypeAttr().Set("TGS")
        physxSceneAPI.CreateBroadphaseTypeAttr().Set("MBP")
        settings = carb.settings.get_settings()
        # async rendering
        # w/o if statement: re-run may freeze
        if not settings.get("/app/asyncRendering"):
            settings.set("/app/asyncRendering", True)
            settings.set("/app/asyncRendering=false",False)
            
        #world.scene.add_default_ground_plane()
        # TODO

    async def setup_post_load(self) -> None:
        world = self._world
        _target_prim_path = find_unique_string_name(
                initial_name="/World/FollowedTarget", is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
        _target_prim = VisualCuboid(
            prim_path=_target_prim_path,
            color=np.array([1, 0, 0]),
            size=1.0,
            scale=np.array([0.02, 0.02, 0.02]) / get_stage_units()
        )
        task = FollowTarget(name="follow_target_task",target_position=[0.2, 0.2, 0.015], target_prim_path=_target_prim_path) # core task api which also set_robot(), bad practice but in api # TODO
        task.set_up_scene(world.scene)
        world.add_task(task)

        # to mimic the loas world async behavior from base sample.py
        # since tasks are here not in setup scene
        self._current_tasks = self._world.get_current_tasks() 

        try:
            RigidBodyRope(world.stage).deleteRope(self._ropeScenePath)
        except:
            pass
        self._ropeScenePath,self._ropePrimPath=RigidBodyRope(world.stage).createRope()

        # get params method in the follow target.py task returns a dict containing:
        #     ["target_prim_path"]
        #     ["target_name"]
        #     ["target_position"]
        #     ["target_orientation"]
        #     ["robot_name"]
        self._task=world.get_task("follow_target_task")
        self._target=self._task._target
        # self._target.initialize()

        # generate random rope shape procedurally
        # TODO play it in another stage, and copy the steady state result back into the main stage
        await world.play_async()
        await asyncio.sleep(3)
        await world.pause_async()

        self._task_params = task_params=self._task.get_params() # fixed value!
        self._robot_name = robot_name=self._task_params["robot_name"]["value"]
        self._target_name = task_params["target_name"]["value"] 
        self._target_prim_path= task_params["target_prim_path"]["value"] 
        robot = self._robot = world.scene.get_object(robot_name)
        self._robot_articulation_solver = KinematicsSolver(robot) # TODO be configurable by yaml
        self._robot_articulation_controller = robot.get_articulation_controller()


    async def _on_simulation_event_async(self, val,callback_fn=None):
        world = self._world
        if val:
            await world.play_async()
        else:
            await world.pause_async()
        if (callback_fn is not None):
            callback_fn()
    

    async def _on_follow_target_event_async(self, val,callback_fn=None):
        world = self._world
        if val:
            await world.play_async()
            world.add_physics_callback("sim_step", self._on_physics_callback)
        else:
            self._init_vars()
            if world.physics_callback_exists("sim_step"):
                world.remove_physics_callback("sim_step")
        if (callback_fn is not None):
            callback_fn()


    # in parallel gripper: 0 == close
    async def _on_gripper_action_event_async(self, val,callback_fn=None): 
        robot = self._robot
        if val:
            robot._gripper.open() # TODO precise control?
        else:
            robot._gripper.close()
        if (callback_fn is not None):
            callback_fn()


    # "all physics callbacks shall take `step_size` as an argument"
    def _on_physics_callback(self, step_size):
        if (self._pre_physics_callback is not None):
            self._pre_physics_callback(step_size)

        self._on_follow_target_simulation_step(step_size)

        if (self._post_physics_callback is not None):
            self._post_physics_callback(step_size)

    def _on_follow_target_simulation_step(self, step_size) -> None:
        observations =  self._world.get_observations()

        _tmp=observations[self._target_name]["position"]
        print(f">>>new tgt {_tmp}<<<")

        if self._old_observations is not None:   
            _tmp=self._old_observations[self._target_name]["position"]
            print(f">>> old tgt {_tmp}<<<")

        if (self._old_observations is not None):
            _pos_delta=observations[self._target_name]["position"]-self._old_observations[self._target_name]["position"]
            _ori_delta=observations[self._target_name]["orientation"]-self._old_observations[self._target_name]["orientation"]
            # target stays still, do nothing
            if (np.linalg.norm(_pos_delta)+np.linalg.norm(_ori_delta) <= np.finfo(np.dtype(_pos_delta[0])).eps):
                return
        self._old_observations=observations

        actions, succ = self._robot_articulation_solver.compute_inverse_kinematics(
            target_position=observations[self._target_name]["position"],
            target_orientation=observations[self._target_name]["orientation"],
        )
        print(f"actions {actions} is {succ}")
        # HELL NO (succ == True) != successed # TODO speed limit
        # return early in case of NaN will still cause the BroadPhaseUpdateData error -> hmm should clip motion # TODO
        # None in action is normal, nan is not
        # >>> old tgt [-0.10554442  0.29619464  0.015     ] <<<
        # >>> new tgt [-0.12670761  0.3028575   0.015     ] <<<
        # actions {'joint_positions': [nan, nan, nan, nan, nan, nan, nan], 'joint_velocities': None, 'joint_efforts': None} is True
        if succ:
            self._robot_articulation_controller.apply_action(actions)
            _tmp=observations[self._target_name]["position"]
            print(f">>>pos after ACTION {_tmp}<<<")
            print("!!!!!!!!!!!!!")
        else:
            carb.log_warn("IK did not converge to a solution. No action is being taken.")
        return


    def _on_logging_event_async(self, val,callback_fn=None):
        world = self._world
        self.data_logger = data_logger = world.get_data_logger()
        if not data_logger.is_started():
            robot_name = self._robot_name
            target_name = self._target_name
            rope_name=self._ropePrimPath
            def frame_logging_func(tasks, scene):
                return {
                    "joint_positions": scene.get_object(robot_name).get_joint_positions().tolist(),
                    "applied_joint_positions": scene.get_object(robot_name)
                    .get_applied_action()
                    .joint_positions.tolist(),
                    "target_world_position": scene.get_object(target_name).get_world_pose()[0].tolist(),
                    "target_world_rotation": scene.get_object(target_name).get_world_pose()[1].tolist(),
                    "target_local_position": scene.get_object(target_name).get_local_pose()[0].tolist(),
                    "target_local_rotation": scene.get_object(target_name).get_local_pose()[1].tolist(),
                    "rope_world_position": scene.get_object(rope_name).get_world_pose()[0].tolist(),
                    "rope_world_rotation":scene.get_object(rope_name).get_world_pose()[1].tolist(),
                    "rope_local_position": scene.get_object(rope_name).get_local_pose()[0].tolist(),
                    "rope_local_rotation":scene.get_object(rope_name).get_local_pose()[1].tolist(),
                }

            data_logger.add_data_frame_logging_func(frame_logging_func)
        if val:
            data_logger.start()
        else:
            data_logger.pause()
        if (callback_fn is not None):
            callback_fn()

    def _on_save_data_event_async(self, log_path=None,callback_fn=None):
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        if log_path is None:
            log_path=os.path.join(os.getcwd(), f"output_data_{current_time}.json")
        world = self._world
        data_logger = self.data_logger=world.get_data_logger()
        data_logger.save(log_path=log_path)
        data_logger.reset()
        if (callback_fn is not None):
            callback_fn()


from omni.isaac.core.prims import XFormPrim
import carb
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics, UsdShade, PhysxSchema
from omni.physx.scripts import physicsUtils, utils
import omni.isaac.core.utils.stage as stage_utils
import omni.isaac.core.utils.prims as prims_utils

import random

class RigidBodyRope:
    def __init__(
        self,
        _stage,
        _uuid=None,
        name="RopeScene",
        _linkHalfLength=0.03,
        _linkRadius=None,
        _ropeLength=2.0,
        _rope_damping=10,
        _rope_stiffness=1.0,
        _coneAngleLimit=110,
        _ropeColor=None,
        _density=0.00005,
    ):
        self._stage = _stage
        self._ropeScenePath=f"/World/{name}" #stage_utils.get_next_free_path(f"/World/{name}")
        self._linkHalfLength = _linkHalfLength
        self._linkRadius = _linkRadius or 0.5 * self._linkHalfLength
        self._ropeLength = _ropeLength
        self._rope_damping = _rope_damping
        self._rope_stiffness = _rope_stiffness
        self._coneAngleLimit = _coneAngleLimit
        self._ropeColor =  _ropeColor or Gf.Vec3f(165.0, 21.0, 21.0)/255.0
        self._density = _density


        # physics options:
        self._contactOffset = 2.0
        self._physicsMaterialPath = Sdf.Path(self._ropeScenePath).AppendChild("PhysicsMaterial")
        UsdShade.Material.Define(self._stage, self._physicsMaterialPath)
        material = UsdPhysics.MaterialAPI.Apply(self._stage.GetPrimAtPath(self._physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(0.5)
        material.CreateDynamicFrictionAttr().Set(0.5)
        material.CreateRestitutionAttr().Set(0)
    
    def deleteRope(self,path):
        prims_utils.delete_prim(path)

    def createRope(self):
        linkLength = 2.0 * self._linkHalfLength - self._linkRadius
        numLinks = int(self._ropeLength / linkLength)
        xStart = -numLinks * linkLength * 0.5

        self._ropePrimPath=scopePath=stage_utils.get_next_free_path("rope", self._ropeScenePath)
        UsdGeom.Scope.Define(self._stage, scopePath)

        y = 0.20  # height of the rope
        z = 0.5
        
        # Create individual capsules # current impl of phy fabric does not support pointinstancer
        capsules = []
        for linkInd in range(numLinks):
            x = xStart + linkInd * linkLength
            capsulePath = Sdf.Path(scopePath).AppendChild(f"capsule_{linkInd}")
            capsule = self._createCapsule(capsulePath)
            # Set transform for each capsule
            xformable = UsdGeom.Xformable(capsule)
            if linkInd == numLinks//2:
                xformable.AddTranslateOp().Set(Gf.Vec3d(x+random.uniform(-0.2, 0.2), y, z+random.uniform(-0.2, 0.2)))
                xformable.AddRotateXYZOp().Set(Gf.Vec3d(random.uniform(-180.0, 180.0),random.uniform(-180.0, 180.0),random.uniform(-180.0, 180.0)))
            else:
                xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
            capsules.append(capsulePath)

        # Create joints between capsules
        jointX = self._linkHalfLength - 0.5 * self._linkRadius
        for linkInd in range(numLinks - 1):
            jointPath = Sdf.Path(scopePath).AppendChild(f"joint_{linkInd}")
            joint = self._createJoint(jointPath)
            
            # Set joint properties
            joint.CreateBody0Rel().SetTargets([capsules[linkInd]])
            joint.CreateBody1Rel().SetTargets([capsules[linkInd + 1]])
            
            # Set local positions for joint ends
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(jointX, 0, 0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-jointX, 0, 0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        return self._ropeScenePath,self._ropePrimPath

    def _createCapsule(self, path: Sdf.Path):
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, path)
        capsuleGeom.CreateHeightAttr(self._linkHalfLength)
        capsuleGeom.CreateRadiusAttr(self._linkRadius)
        capsuleGeom.CreateAxisAttr("X")
        capsuleGeom.CreateDisplayColorAttr().Set([self._ropeColor])

        # Add physics properties
        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        massAPI.CreateDensityAttr().Set(self._density)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsuleGeom.GetPrim())
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, capsuleGeom.GetPrim(), self._physicsMaterialPath)
        
        return capsuleGeom

    def _createJoint(self, jointPath):
        joint = UsdPhysics.Joint.Define(self._stage, jointPath)
        d6Prim = joint.GetPrim()

        # Lock translational DOFs
        for axis in ["transX", "transY", "transZ"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)

        # Configure rotational DOFs
        for axis in ["rotX", "rotY", "rotZ"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
            limitAPI.CreateLowAttr(-self._coneAngleLimit)
            limitAPI.CreateHighAttr(self._coneAngleLimit)

            # Add joint drives
            driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, axis)
            driveAPI.CreateTypeAttr("force")
            driveAPI.CreateDampingAttr(self._rope_damping)
            driveAPI.CreateStiffnessAttr(self._rope_stiffness)

        return joint
    

class RigidBodyRope_withPointintancer:
    def __init__(
        self,
        _stage,
        _uuid=None,
        name="RopeScene",
        _linkHalfLength=0.03,
        _linkRadius=None,
        _ropeLength=2.0,
        _rope_damping=0.1,
        _rope_stiffness=0.01,
        _coneAngleLimit=110,
        _ropeColor=None,
        _density=0.00005,
    ):
        self._stage = _stage
        self._ropeScenePath=f"/World/{name}" #stage_utils.get_next_free_path(f"/World/{name}")
        self._linkHalfLength = _linkHalfLength
        self._linkRadius = _linkRadius or 0.5 * self._linkHalfLength
        self._ropeLength = _ropeLength
        self._rope_damping = _rope_damping
        self._rope_stiffness = _rope_stiffness
        self._coneAngleLimit = _coneAngleLimit
        self._ropeColor =  _ropeColor or Gf.Vec3f(165.0, 21.0, 21.0)/255.0
        self._density = _density


        # physics options:
        self._contactOffset = 2.0
        self._physicsMaterialPath = Sdf.Path(self._ropeScenePath).AppendChild("PhysicsMaterial")
        UsdShade.Material.Define(self._stage, self._physicsMaterialPath)
        material = UsdPhysics.MaterialAPI.Apply(self._stage.GetPrimAtPath(self._physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(0.5)
        material.CreateDynamicFrictionAttr().Set(0.5)
        material.CreateRestitutionAttr().Set(0)
    
    def deleteRope(self,path):
        stage_utils.clear_stage(path)

    def createRope(self):
        linkLength = 2.0 * self._linkHalfLength - self._linkRadius
        numLinks = int(self._ropeLength / linkLength)
        xStart = -numLinks * linkLength * 0.5

        scopePath = stage_utils.get_next_free_path("rope", self._ropeScenePath)
        UsdGeom.Scope.Define(self._stage, scopePath)

        # capsule instancer
        instancerPath = Sdf.Path(scopePath).AppendChild("rigidBodyInstancer")
        rboInstancer = UsdGeom.PointInstancer.Define(self._stage, instancerPath)

        capsulePath = instancerPath.AppendChild("capsule")
        self._createCapsule(capsulePath)

        meshIndices = []
        positions = []
        orientations = []

        y = 0.20 # todo
        z = 0.0

        for linkInd in range(numLinks):
            meshIndices.append(0)
            x = xStart + linkInd * linkLength
            positions.append(Gf.Vec3f(x, y, z))
            orientations.append(Gf.Quath(1.0))

        meshList = rboInstancer.GetPrototypesRel()
        # add mesh reference to point instancer
        meshList.AddTarget(capsulePath)

        rboInstancer.GetProtoIndicesAttr().Set(meshIndices)
        rboInstancer.GetPositionsAttr().Set(positions)
        rboInstancer.GetOrientationsAttr().Set(orientations)

        # joint instancer
        jointInstancerPath = Sdf.Path(scopePath).AppendChild("jointInstancer")
        jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath)

        jointPath = jointInstancerPath.AppendChild("joint")
        self._createJoint(jointPath)

        meshIndices = []
        body0s = []
        body0indices = []
        localPos0 = []
        localRot0 = []
        body1s = []
        body1indices = []
        localPos1 = []
        localRot1 = []      
        body0s.append(instancerPath)
        body1s.append(instancerPath)

        jointX = self._linkHalfLength - 0.5 * self._linkRadius
        for linkInd in range(numLinks - 1):
            meshIndices.append(0)

            body0indices.append(linkInd)
            body1indices.append(linkInd + 1)

            localPos0.append(Gf.Vec3f(jointX, 0, 0)) 
            localPos1.append(Gf.Vec3f(-jointX, 0, 0)) 
            localRot0.append(Gf.Quath(1.0))
            localRot1.append(Gf.Quath(1.0))

        meshList = jointInstancer.GetPhysicsPrototypesRel()
        meshList.AddTarget(jointPath)

        jointInstancer.GetPhysicsProtoIndicesAttr().Set(meshIndices)

        jointInstancer.GetPhysicsBody0sRel().SetTargets(body0s)
        jointInstancer.GetPhysicsBody0IndicesAttr().Set(body0indices)
        jointInstancer.GetPhysicsLocalPos0sAttr().Set(localPos0)
        jointInstancer.GetPhysicsLocalRot0sAttr().Set(localRot0)

        jointInstancer.GetPhysicsBody1sRel().SetTargets(body1s)
        jointInstancer.GetPhysicsBody1IndicesAttr().Set(body1indices)
        jointInstancer.GetPhysicsLocalPos1sAttr().Set(localPos1)
        jointInstancer.GetPhysicsLocalRot1sAttr().Set(localRot1)

        return scopePath
    
    def _createCapsule(self, path: Sdf.Path):
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, path)
        capsuleGeom.CreateHeightAttr(self._linkHalfLength)
        capsuleGeom.CreateRadiusAttr(self._linkRadius)
        capsuleGeom.CreateAxisAttr("X")
        capsuleGeom.CreateDisplayColorAttr().Set([self._ropeColor])

        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        massAPI.CreateDensityAttr().Set(self._density)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsuleGeom.GetPrim())
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, capsuleGeom.GetPrim(), self._physicsMaterialPath)

    def _createJoint(self, jointPath):        
        joint = UsdPhysics.Joint.Define(self._stage, jointPath)

        # locked DOF (lock - low is greater than high)
        d6Prim = joint.GetPrim()
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transX")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transY")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transZ")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        # limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "rotX")
        # limitAPI.CreateLowAttr(1.0)
        # limitAPI.CreateHighAttr(-1.0)

        # Moving DOF:
        dofs = ["rotX", "rotY", "rotZ"]
        for d in dofs:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, d)
            limitAPI.CreateLowAttr(-self._coneAngleLimit)
            limitAPI.CreateHighAttr(self._coneAngleLimit)

            # joint drives for rope dynamics:
            driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, d)
            driveAPI.CreateTypeAttr("force")
            driveAPI.CreateDampingAttr(self._rope_damping)
            driveAPI.CreateStiffnessAttr(self._rope_stiffness)

# TODO
class RigidBodyRope_Factory():
    pass


# TODO deformable have to work on GPU, which means no numpy code!
# from omni.kit.commands import execute
# class DeformableRope:
#     def __init__(self,_stage):
#         self._stage=_stage
#         pass

#     def createRope(self):
#         stage = self._stage

#         # Define the root Xform
#         rootXform = UsdGeom.Xform.Define(stage, '/World')

#         cylinderXform = UsdGeom.Xform.Define(stage, '/World/DeformableRope')
#         cylinderXform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
#         cylinderXform.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, 0))
#         cylinderXform.AddScaleOp().Set(Gf.Vec3f(0.03, 5, 0.03))


#         result, prim_path = execute(
#             "CreateMeshPrim",
#             prim_type="Cylinder",
#             object_origin=Gf.Vec3f(0.0, 0.0, 0.0),
#         )
#         execute("MovePrim", path_from=prim_path, path_to='/World/DeformableRope/DeformableRope')

#         cylinderPrim = stage.GetPrimAtPath("/World/DeformableRope/DeformableRope")

#         PhysxSchema.PhysxDeformableBodyAPI.Apply(cylinderPrim)
#         cylinderPrim.CreateAttribute('physxDeformable:enableCCD', Sdf.ValueTypeNames.Bool).Set(True)
#         cylinderPrim.CreateAttribute('physxDeformable:numberOfTetsPerHex', Sdf.ValueTypeNames.UInt).Set(5)
#         cylinderPrim.CreateAttribute('physxDeformable:simulationHexahedralResolution', Sdf.ValueTypeNames.UInt).Set(50)


#         # Define Material for deformable physics using PhysxSchema
#         # cotton
#         # https://repository.gatech.edu/server/api/core/bitstreams/8214a46f-0424-436e-a261-5c46f91c8d4e/content
#         # http://www-materials.eng.cam.ac.uk/mpsite/short/OCR/ropes/default.html
#         material = stage.DefinePrim('/deformablePhysicsMaterial', 'Material')
#         PhysxSchema.PhysxDeformableBodyMaterialAPI.Apply(material)
#         material.CreateAttribute('physxDeformableBodyMaterial:density', Sdf.ValueTypeNames.Float).Set(1.54)
#         material.CreateAttribute('physxDeformableBodyMaterial:dynamicFriction', Sdf.ValueTypeNames.Float).Set(0.257)
#         material.CreateAttribute('physxDeformableBodyMaterial:elasticityDamping', Sdf.ValueTypeNames.Float).Set(0.3)
#         material.CreateAttribute('physxDeformableBodyMaterial:poissonsRatio', Sdf.ValueTypeNames.Float).Set(0.0)
#         material.CreateAttribute('physxDeformableBodyMaterial:youngsModulus', Sdf.ValueTypeNames.Float).Set(8000000000)
#         pass

# the async ext utils for stage hot reload etc
import asyncio
from omni.isaac.core.utils.stage import (
    create_new_stage_async,
    is_stage_loading,
    update_stage_async,
    clear_stage,
    close_stage,
    create_new_stage,
)

# ~~anti feature/no impl: ui~~ impl below for both button binding in isaac and vr

import os
import omni.ui
from omni.isaac.ui.ui_utils import btn_builder, state_btn_builder, str_builder

import carb.input
import omni.appwindow

from typing import Optional, Dict, List, Any
from dataclasses import dataclass

class Button:
    default_state: str
    alternative_state: Optional[str] = None
    current_state: str

    def __init__(self,default_state,alternative_state,current_state):
        self.default_state=default_state
        self.alternative_state=alternative_state
        self.current_state=current_state

    def get_state_and_flip(self):
        started=self.current_state == self.default_state
        self.get_or_set_state(started)
        return started
    
    def get_or_set_state(self, toStart:bool=None):
        if toStart is None:
            return self.current_state == self.default_state
        if not toStart:
            self.default()
        else:
            self.alternative()
    

    def default(self):
        self.current_state=self.default_state

    def alternative(self):
        self.current_state=self.alternative_state

class ControlFlow:
    _sample=None        
    buttons: Optional[Dict[str, Button]] = None
    BUTTON_CONFIG: Dict[str, List[str]] = {
        "(Re)load": ["(re)load"],
        "Stop(Reset)": ["stop(reset)"],
        "Clear": ["clear"],
        "Follow Target": ["to start", "to stop"],
        "Simulation": ["to play", "to pause"],
        "Gripper Action": ["to open", "to close"],
        "Start Logging": ["to begin", "to stop"],
        "Save Data": ["save"],
    }

    @classmethod
    def _populate_button(cls, name: str, state: str, alt_state: Optional[str] = None) -> None:
        cls.buttons[name] = Button(state, alt_state, state)

    @classmethod
    def reset_buttons(cls,callback_fn=None):
        for name,button in cls.buttons.items():
            cls.buttons[name].default()
            print(name)
            print(cls.buttons[name].default_state)
        if (callback_fn is not None):
            callback_fn()

    @classmethod
    def init_buttons(cls,callback_fn=None) -> None:
        if cls.buttons is not None:
            cls.reset_buttons()
            return
        cls.buttons={}
        try:
            for name, states in cls.BUTTON_CONFIG.items():
                alt_state = states[1] if len(states) > 1 else None
                cls._populate_button(name, states[0], alt_state)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize buttons: {str(e)}")
        if (callback_fn is not None):
            callback_fn()

    @classmethod
    async def setUp_async(cls, callback_fn=None):
        if (cls._sample is not None):
            try:
                await cls.tearDown_async()
            except:
                pass
        # await create_new_stage_async()
        # await update_stage_async()
        cls._sample = FrankaRope() # TODO replace with yaml
        await cls._sample.load_world_async()
        await update_stage_async()
        if (callback_fn is not None):
            callback_fn()
    

    @classmethod
    async def tearDown_async(cls,callback_fn=None) -> None:
        if (cls._sample is not None):
            close_stage()
            await cls._sample.clear_async() # clear async does not close old stage
        await update_stage_async()
        try:
            del cls._sample
        except:
            pass
        cls._sample = None
        if (callback_fn is not None):
            callback_fn()
            
    @classmethod
    async def reset_async(cls,callback_fn=None) -> None:
        if (cls._sample is not None):
            await cls._sample.reset_async()
        if callback_fn is not None:
            callback_fn()
        await update_stage_async()

    @classmethod
    def on_reload(cls,callback_fn=None):
        async def _on_reload_async(callback_fn=None):
            cls.init_buttons()
            await cls.setUp_async(callback_fn)
        asyncio.ensure_future(_on_reload_async(callback_fn))

    @classmethod
    def on_reset(cls,callback_fn=None):
        async def _on_reset_async(callback_fn=None):
            cls.init_buttons()
            await cls.reset_async(callback_fn)
        asyncio.ensure_future(_on_reset_async(callback_fn))
    
    @classmethod
    def on_clear(cls,callback_fn=None):
        async def _on_clear_async(callback_fn=None):
            cls.init_buttons()
            await cls.tearDown_async(callback_fn)
        asyncio.ensure_future(_on_clear_async(callback_fn))

    @classmethod
    def on_simulation_button_event(cls, playing=None,callback_fn=None):
        async def _on_simulation_button_event_async(playing=None,callback_fn=None):
            val = cls.buttons["Simulation"].get_state_and_flip()
            if playing is None:
                playing=val
            else:
                cls.buttons["Simulation"].get_or_set_state(playing)
            # two seprate states: playing and val: workaround for isaac sim's stateful a/b-text button
            assert (playing == val)
            if not playing:
                pass
                # better: not modifty the task
                # cls.buttons["Follow Target"].default()
                # await cls._sample._on_follow_target_event_async(False)
            else:
                pass
            await cls._sample._on_simulation_event_async(playing,callback_fn)
        asyncio.ensure_future(_on_simulation_button_event_async(playing,callback_fn))
    
    @classmethod # TODO maybe refactor using publisher subscriber/callback
    def on_follow_target_button_event(cls, following=None,callback_fn=None):
        async def _on_follow_target_button_event_async(following=None,callback_fn=None):
            val=cls.buttons["Follow Target"].get_state_and_flip()
            if following is None:
                following=val
            else:
                cls.buttons["Follow Target"].get_or_set_state(following)
            print(following)
            print(val)
            assert (following == val)
            if following:
                cls.buttons["Simulation"].alternative()
            else:
                cls.buttons["Simulation"].default()
            await cls._sample._on_follow_target_event_async(following,callback_fn)
        asyncio.ensure_future(_on_follow_target_button_event_async(following,callback_fn))

    @classmethod
    def on_gripper_action_button_event(cls, open=None,callback_fn=None):
        async def _on_gripper_action_button_event_async(open=None,callback_fn=None):
            val=cls.buttons["Gripper Action"].get_state_and_flip()
            if open is None:
                open=val
            else:
                cls.buttons["Gripper Action"].get_or_set_state(open)
            print(open)
            print(val)
            assert (open == val)
            await cls._sample._on_gripper_action_event_async(open,callback_fn)
        asyncio.ensure_future(_on_gripper_action_button_event_async(open,callback_fn))

    @classmethod
    def on_logging_button_event(cls, logging=None,callback_fn=None):
        val=cls.buttons["Start Logging"].get_state_and_flip()
        if logging is None:
            logging=val
        else:
            cls.buttons["Start Logging"].get_or_set_state(logging)
        print(logging)
        print(val)
        assert (logging == val)
        cls._sample._on_logging_event_async(logging,callback_fn)

    @classmethod
    def on_save_data_button_event(cls,callback_fn=None):
        cls._sample._on_save_data_event_async(callback_fn)


# singleton
# all UI related
from functools import partial
class IsaacUIUtils(ControlFlow):
    isaac_buttons=None
    ui_window=None

    @classmethod
    def bind_inputs(cls):
        if os.getenv("__input_bound") is not None:
            return
        app_window = omni.appwindow.get_default_app_window()
        input_interface = carb.input.acquire_input_interface()
        keyboard = app_window.get_keyboard()
        cls.sub_id = input_interface.subscribe_to_keyboard_events(keyboard, cls.on_input)
        os.environ["__input_bound"]=str(cls.sub_id)

    @classmethod
    def unbind_inputs(cls):
        if os.getenv("__input_bound") is None:
            return
        sub_id=cls.sub_id or int(os.getenv("__input_bound"))
        app_window = omni.appwindow.get_default_app_window()
        input_interface = carb.input.acquire_input_interface()
        keyboard = app_window.get_keyboard()
        input_interface.unsubscribe_to_keyboard_events(keyboard,sub_id)

    @classmethod
    def on_input(cls,event):
        if event.input == carb.input.KeyboardInput.SPACE:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                if not cls._sample._world.is_playing():
                    cls.on_simulation_button_event(True)
                else:
                    cls.on_simulation_button_event(False)
        if event.input == carb.input.KeyboardInput.J:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                cls.on_gripper_action_button_event(True)
        if event.input == carb.input.KeyboardInput.K:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                cls.on_gripper_action_button_event(False)


    @classmethod
    def setUp(cls, window_name: str = "Franka Rope") -> None:
        print("Creating window for environment.")
        ui_window = omni.ui.Workspace.get_window(window_name) or cls.ui_window
        if (ui_window is not None):
            # cls.tearDown()
            return # dont create ui window on hot reloading, also dont destroy exsiting window either
        async def _setUp_async(cls):
            await super().setUp_async()
            cls.bind_inputs()
            cls.ui_window = omni.ui.Window(window_name, width=300, height=300)
            cls.ui_window.flags = (omni.ui.WINDOW_FLAGS_NO_CLOSE)
            cls.add_buttons()
        asyncio.ensure_future(_setUp_async(cls))
        return
    
    @classmethod
    def tearDown(cls,window_name: str = "Franka Rope") -> None:
        asyncio.ensure_future(super().tearDown_async())
        # destroy the window
        try: # also works with no input bound
            cls.unbind_inputs()
        except:
            pass
        ui_window = omni.ui.Workspace.get_window(window_name) or cls.ui_window # just like x = a || b 
        if ui_window is not None:
            ui_window.visible = False
            ui_window.destroy()
            ui_window = None
        return

    @classmethod
    def _update_ui_button_text(cls):
        print(">>>in update ui button<<<")
        for name,dict in cls.isaac_buttons.items():
            if "b_text" in dict:
                print(name)
                print(cls.buttons[name].current_state.upper())
                cls.isaac_buttons[name]["ui_button"].text=cls.buttons[name].current_state.upper() # omniverse buttons are in CAP

    @classmethod
    def build_buttons(cls):
        _callback_fn=cls._update_ui_button_text
        _on_reload=partial(super().on_reload,callback_fn=_callback_fn)
        _on_reset=partial(super().on_reset,callback_fn=_callback_fn)
        _on_clear=partial(super().on_clear,callback_fn=_callback_fn)
        _on_follow_target_button_event=partial(super().on_follow_target_button_event,callback_fn=_callback_fn)
        _on_simulation_button_event=partial(super().on_simulation_button_event,callback_fn=_callback_fn)
        _on_gripper_action_button_event=partial(super().on_gripper_action_button_event,callback_fn=_callback_fn)
        _on_logging_button_event=partial(super().on_logging_button_event,callback_fn=_callback_fn)

        cls.FN_CONFIG={
            "(Re)load": _on_reload,
            "Stop(Reset)": _on_reset,
            "Clear": _on_clear,
            "Follow Target": _on_follow_target_button_event,
            "Simulation": _on_simulation_button_event,
            "Gripper Action": _on_gripper_action_button_event,
            "Start Logging": _on_logging_button_event,
            "Save Data": super().on_save_data_button_event,
        }
        
        dicts={}
        for name, button in cls.buttons.items():
            dicts[name]={
                "label":name,
                "type":"button",
            }
            if button.alternative_state is None:
                dicts[name]["text"]=name
            else:
                dicts[name]["a_text"]=button.default_state
                dicts[name]["b_text"]=button.alternative_state
        for name, fn in cls.FN_CONFIG.items():
            dicts[name]["on_clicked_fn"]=fn
        
        cls.isaac_buttons=dicts

    @classmethod
    def add_buttons(cls):
        if cls.buttons is None: # super().param != cls.param
            super().init_buttons()
        # if cls.isaac_buttons is not None:
        #     return
        cls.build_buttons()
        with cls.ui_window.frame:
            with omni.ui.VStack():
                for name,dict in cls.isaac_buttons.items():
                    if "b_text" in dict:
                        _btn=state_btn_builder(**dict)
                        _btn.enabled=True
                        cls.isaac_buttons[name]["ui_button"]=_btn
                    else:
                        btn_builder(**dict)
                # dict = {
                #     "label": "Output Directory",
                #     "type": "stringfield",
                #     "default_val": os.path.join(os.getcwd(), "OUTPUT PATH.json"),
                #     "tooltip": "Output Directory",
                #     "on_clicked_fn": None,
                #     "use_folder_picker": False,
                #     "read_only": False,
                # }
                # str_builder(**dict)


# semi hidden api
# https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.utils/docs/index.html#omni.isaac.utils._isaac_utils.math.mul
from omni.isaac.utils._isaac_utils import math as mu
from concurrent.futures import ThreadPoolExecutor
import threading
import time
lock=threading.Lock()

class VRUIUtils(ControlFlow):

    @classmethod
    def _init_vars(cls):
        try:
            del cls.publisher
            del cls.vr_controller
            del cls.old_input_pos
            del cls.old_input_rot
            del cls.old_input_data
            del cls.input_data
            del cls.already_pressed
            del cls.names
            del cls._zeroing
        except:
            pass
        cls.publisher=None
        cls.vr_controller=None
        cls.old_input_pos=None
        cls.old_input_rot=None
        cls.old_input_data=None
        cls.input_data=None
        cls.already_pressed={}
        cls.names=[]
        cls._zeroing=False

    @classmethod
    def init_publisher(cls,vr_controller=None):
        def _init_publisher(cls,vr_controller=None):
            world=cls._sample._world
            if (cls.publisher is None):
                print("INIT SIMPUBLISHER <<< ")
                cls.publisher = IsaacSimPublisher(host="127.0.0.1", stage=world.stage) # for InteractiveScene
            # THE MetaQuest3 NAME MUST BE THE SAME AS IN THE CSHARP CODE
            if (cls.vr_controller is None):
                print("INIT META QUEST 3 <<< ")
                # only works w/ fabric backend
                cls.vr_controller=vr_controller or MetaQuest3("UnityClient")
            super().on_simulation_button_event(False)
        super().on_simulation_button_event(True,callback_fn=partial(_init_publisher,cls,vr_controller))

    @classmethod
    async def init_publisher_async(cls,vr_controller=None):
        cls.init_publisher(vr_controller)
    
    @classmethod
    def destroy_publisher(cls):
        if cls.publisher is not None:
            cls.publisher.scene_update_streamer.shutdown()
            cls.publisher.scene_service.shutdown()
            cls.publisher.asset_service.shutdown()
            try:
                del cls.publisher
            except:
                pass
            cls.publisher=None
        # if cls.vr_controller is not None:

    @classmethod
    async def destroy_publisher_async(cls):
        cls.destroy_publisher()

    @classmethod
    def setUp(cls,vr_controller=None):
        cls._init_vars()
        for name,_ in cls.BUTTON_CONFIG.items():
            cls.names.append(name)
        cls.names.append("Reposition")

        async def _setUp_async(cls,vr_controller=None):
            super().init_buttons()

            cls.reset_press_states()
            if (cls.vr_controller is not None):
                cls.tearDown()
                
            await super().setUp_async()

            # issue: isaacsim/omniverse on init launch does not populate rt_prim without out pressing simulation start first (register the timeline event)
            # issue: w/o timeline play, the vr controller wont be registered to send back data
            cls.init_publisher()

            loop = asyncio.get_event_loop()
            await asyncio.to_thread(partial(cls.ui_demon,loop))
        asyncio.ensure_future(_setUp_async(cls,vr_controller))

    @classmethod
    def ui_demon(cls,loop):
        asyncio.set_event_loop(loop)
        while True:
            start = time.time()
            try:
                cls.bind_inputs()
            except:
             pass
            end = time.time()
            if (end-start<0.009):
                time.sleep(0.009-(end-start))

    @classmethod
    def pre_physics_callback(cls,step_size):
        print(">>>calling pre phy callback<<<")
        if cls._requested_zeroing_pose():
            cls.register_zeroing_pose()
        else:
            cls.default_task()

    @classmethod
    def _requested_zeroing_pose(cls):
        with lock:
            print(">>> zeroing <<<")
            if cls._zeroing:
                cls._zeroing=False
                return True
            else:
                return False

    @classmethod
    def _request_zeroing_pose(cls):
        cls._zeroing=False

    @classmethod
    def register_zeroing_pose(cls):
        input_data=cls.vr_controller.get_input_data()        
        cls.zeroing_pose(input_data)

    @classmethod
    def default_task(cls):
        print(">>> default task <<<")
        input_data=cls.vr_controller.get_input_data()
        cls.transform_target(input_data)
        cls.reset_press_states()
        return

    @classmethod
    def reset_press_states(cls):
        for name in cls.names:
            cls.already_pressed[name]=False

    @classmethod
    def bind_inputs(cls):

        # control logic: 
        #   each button can be pressed max once between each simulation step
        # control logic priority:
        # 1. return if the input is invalid
        # 2. return on reset
        # 3. return on simulation Simulation
        # 4. return on follow target task start/stop
        # 5. continue only if **none** of the above conditions meet: update pose and gripper action
        if cls.vr_controller is None:
            return
        cls.old_input_data=cls.input_data or cls.vr_controller.get_input_data()
        input_data = cls.input_data= cls.vr_controller.get_input_data()
        if input_data is None:
            return
        if cls._sample is None:
            return
        
        if cls._reset_pressed(input_data) and not cls.already_pressed["Stop(Reset)"]:
            cls.reset_press_states()
            cls.already_pressed["Stop(Reset)"]=True
            print(f" Stop(Reset) ")
            # ~~TODO discard logging~~ sample reset also reset logger
            cls._request_zeroing_pose()
            _onFinish=lambda:print("vibrate") # vibrate
            def _onFinish(cls):
                async def _onFinish_async(cls):
                    super().on_simulation_button_event(True,callback_fn=partial(super().on_simulation_button_event,False))
                    print("vibrate")
                asyncio.ensure_future(_onFinish_async(cls))
            super().on_reset(_onFinish(cls))
            return
        
        if cls._simulation_pressed(input_data) and not cls.already_pressed["Simulation"]:
            cls.reset_press_states()
            cls.already_pressed["Simulation"]=True
            val=cls.buttons["Simulation"].get_or_set_state()
            print(f" simulation is {val}")
            if val:
                cls._request_zeroing_pose()                
            super().on_simulation_button_event(val,cls.reset_press_states)
            return
        
        if cls._follow_target_pressed(input_data) and not cls.already_pressed["Follow Target"]:
            cls.already_pressed["Follow Target"]=True
            val=cls.buttons["Follow Target"].get_or_set_state()
            print(f" follow target is {val}")
            if val:
                super().on_logging_button_event(True)
                cls._request_zeroing_pose()
            else:
                super().on_save_data_button_event()
                pass
            def _pre_physics_callback(val):
                sample = cls._sample
                if not val:
                    try: 
                        del cls._sample._pre_physics_callback
                        cls._sample._pre_physics_callback=None
                    except:
                        pass
                if cls._sample._pre_physics_callback is None:
                    sample._pre_physics_callback=cls.pre_physics_callback
            super().on_follow_target_button_event(val,callback_fn=partial(_pre_physics_callback,val))
            return

        if cls._gripper_action_pressed(input_data) and not cls.already_pressed["Gripper Action"]:
            print(">>>gripper action pressed<<<")
            if cls.already_pressed["Gripper Action"] is not True:
                print("vibrate: performing gripper action ")
            cls.already_pressed["Gripper Action"]=True;
            val=cls.buttons["Gripper Action"].get_or_set_state()
            cls.on_gripper_action_button_event(val)


    @classmethod
    def _reset_pressed(cls, input_data):
        # input_data format {'left': {'pos': [1.0, 0.0, 0.0], 'rot': [0.0, 0.0, -1.0, 0.0], 'index_trigger': False, 'hand_trigger': False}, 'right': {'pos': [1.0, 0.0, 0.0], 'rot': [0.0, 0.0, -1.0, 0.0], 'index_trigger': False, 'hand_trigger': False}, 'A': False, 'B': False, 'X': False, 'Y': False}
        return not cls.old_input_data["X"] and input_data["X"]

    @classmethod
    def _follow_target_pressed(cls, input_data):
        return not cls.old_input_data["Y"] and input_data["Y"]
    
    @classmethod
    def _reposition_pressed(cls, input_data):
        return input_data["A"]
    
    @classmethod
    def _simulation_pressed(cls, input_data):
        return not cls.old_input_data["B"] and input_data["B"]
    
    @classmethod
    def _gripper_action_pressed(cls, input_data):
        return not cls.old_input_data["right"]["index_trigger"] and input_data["right"]["index_trigger"]

    @classmethod
    def zeroing_pose(cls, input_data):
        cls.old_input_pos,cls.old_input_rot = cls.get_pose(input_data)


    @classmethod
    def transform_target(cls,input_data):
        async def _transform_target_async(input_data):
            if input_data is None:
                return

            # no pose update if holding the reposition button to relocate the followed target
            if  cls._reposition_pressed(input_data):
                # old_input_pos=None # delete the memory
                # old_input_rot=None
                cls.zeroing_pose(input_data)
                return
            
            old_input_pos,old_input_rot=cls.old_input_pos,cls.old_input_rot
            if old_input_pos is None or old_input_rot is None:
                old_input_pos,old_input_rot=cls.zeroing_pose(input_data)
            try:
                input_pos,input_rot= cls.old_input_pos,cls.old_input_rot = cls.get_pose(input_data)
            except: # metaquest3.py returns none in the data :( the err: TypeError: cannot unpack non-iterable NoneType object
                return

            # numpy is weird
            # delta_pos=input_pos-old_input_pos
            delta_pos=np.subtract(input_pos,old_input_pos)
            delta_rot=mu.mul(input_rot,mu.inverse(old_input_rot)) # ~~the order is unclear in doc could be another way around~~
            task_params=cls._sample._task_params
            observations=cls._sample._world.get_observations()
            old_target_pos,old_target_rot=observations[cls._sample._target_name]["position"],observations[cls._sample._target_name]["orientation"]
            # target_pos=old_target_pos+delta_pos
            target_pos=np.add(old_target_pos,delta_pos)
            target_rot=mu.mul(delta_rot,old_target_rot)
            # set params expects a scalar-first (w, x, y, z) quaternion
            # the singe_prim_wrapper no longer expect target pos/ori but delta!!! follow_target.py has to adapt
            # ~~lul, isaac core modules are not compatible~~
            # AsyncUtils._sample._task.set_params(target_position=target_pos,target_orientation=target_rot)
            # AsyncUtils._sample._task.set_params(delta_pos,delta_rot)

            print(f">>>before set local { cls._sample._target.get_local_pose()}<<<")

            # z axis never below 0, better to use stage.get_stage_up_axis()-> str to determine up 
            # but z is the default up axis, set by set_stage_up_axis() in simulation context.py
            # dont follow it once it went below the ground
            eps= 0.015
            if target_pos[2]<=eps:
                target_pos[2]=eps
            cls._sample._target.set_local_pose(translation=target_pos, orientation=target_rot)
            await update_stage_async()
            print(f">>>tgt pos:{target_pos}    tgt rot:{target_rot}<<<")
        asyncio.ensure_future(_transform_target_async(input_data))

    @classmethod
    def get_pose(cls,input_data):
        # rot is quaternion
        # the rot from metaquest3 class is {-rightRot.z, rightRot.x, -rightRot.y, rightRot.w}
        q_xyzw=input_data["right"]["rot"]
        q_wxyz=np.array([q_xyzw[3],q_xyzw[0],q_xyzw[1],q_xyzw[2]])
        pos=input_data["right"]["pos"]
        print(f">>>pos:{pos}    rot:{q_wxyz}<<<")
        return input_data["right"]["pos"],q_wxyz
    

    @classmethod    
    def tearDown(cls):
        cls.unbind_inputs()
        try:
            cls.destroy_publisher()
        except:
            pass

    @classmethod
    def unbind_inputs(cls):
        try:
            _sample = cls._sample
            del _sample._pre_physics_callback
            _sample._pre_physics_callback=None
        except:
            pass