# TODO refactor later maybe?

from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics, UsdShade, PhysxSchema
from omni.physx.scripts import physicsUtils
import omni.isaac.core.utils.stage as stage_utils

class RigidBodyRopeNoPointinstancer:
    def __init__(
        self,
        _stage,
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
        self._defaultPrimPath=f"/World/{name}" #stage_utils.get_next_free_path(f"/World/{name}")
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
        self._physicsMaterialPath = Sdf.Path(self._defaultPrimPath).AppendChild("PhysicsMaterial")
        UsdShade.Material.Define(self._stage, self._physicsMaterialPath)
        material = UsdPhysics.MaterialAPI.Apply(self._stage.GetPrimAtPath(self._physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(0.5)
        material.CreateDynamicFrictionAttr().Set(0.5)
        material.CreateRestitutionAttr().Set(0)
    
    def deleteRope(self,path):
        stage_utils.clear_stage(path)

    # TODO randomize the rope via procedural phy sim/force field
    def createRope(self):
        linkLength = 2.0 * self._linkHalfLength - self._linkRadius
        numLinks = int(self._ropeLength / linkLength)
        xStart = -numLinks * linkLength * 0.5

        scopePath = stage_utils.get_next_free_path("rope", self._defaultPrimPath)
        UsdGeom.Scope.Define(self._stage, scopePath)

        y = 0.20  # height of the rope
        z = 0.0
        
        # Create individual capsules # current impl of phy fabric does not support pointinstancer
        capsules = []
        for linkInd in range(numLinks):
            x = xStart + linkInd * linkLength
            capsulePath = Sdf.Path(scopePath).AppendChild(f"capsule_{linkInd}")
            capsule = self._createCapsule(capsulePath)
            # Set transform for each capsule
            xformable = UsdGeom.Xformable(capsule)
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

        return self._defaultPrimPath

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




class RigidBodyRopeWithPointinstancer:
    def __init__(
        self,
        _stage,
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
        self._defaultPrimPath=f"/World/{name}" #stage_utils.get_next_free_path(f"/World/{name}")
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
        self._physicsMaterialPath = Sdf.Path(self._defaultPrimPath).AppendChild("PhysicsMaterial")
        UsdShade.Material.Define(self._stage, self._physicsMaterialPath)
        material = UsdPhysics.MaterialAPI.Apply(self._stage.GetPrimAtPath(self._physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(0.5)
        material.CreateDynamicFrictionAttr().Set(0.5)
        material.CreateRestitutionAttr().Set(0)
    
    def deleteRope(self,path):
        stage_utils.clear_stage(path)

    # TODO randomize the rope via procedural phy sim/force field
    def createRope(self):
        linkLength = 2.0 * self._linkHalfLength - self._linkRadius
        numLinks = int(self._ropeLength / linkLength)
        xStart = -numLinks * linkLength * 0.5

        scopePath = stage_utils.get_next_free_path("rope", self._defaultPrimPath)
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

        return self._defaultPrimPath
    
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


# TODO
class DeformableRope():
    pass