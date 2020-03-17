import pymel.core as pm
import math

selection = pm.selected()

handle = selection[0]
ctrl = selection[1]

if pm.objectType(handle) == 'ikHandle':
    affectedJoints = pm.ikHandle('RightHandIK', q=True, jl=True)

    P0 = pm.dt.Vector(pm.xform(affectedJoints[0], translation=True, q=True, worldSpace=True))
    P1 = pm.dt.Vector(pm.xform(affectedJoints[1], translation=True, q=True, worldSpace=True))
    P2 = pm.dt.Vector(pm.xform(handle, translation=True, q=True, worldSpace=True))
    altP1 = pm.dt.Vector(pm.xform(ctrl, rotatePivot=True, q=True, worldSpace=True))

    v1 = P2 - P0
    v2 = P1 - P0
    altV2 = altP1 - P0
    map(lambda x: x.normalize(), (v1, v2, altV2))

    N1 = v1.cross(v2)
    N2 = v1.cross(altV2)
    map(lambda x: x.normalize(), (N1, N2))
    a = math.acos(N1.dot(N2)) * 180 / math.pi

    