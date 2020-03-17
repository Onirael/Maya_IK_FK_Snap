import pymel.core as pm

selection = pm.selected()


for ctrl in selection:
    # Test whether the element is a controller
    try:
        pm.getAttr(str(ctrl) + ".childJoint")
    except:
        continue
    else:
        pm.xform(ctrl, translation=[0,0,0], rotation=[0,0,0], objectSpace=True)
        constraints = pm.listRelatives(ctrl, children=True, type=('orientConstraint', 'parentConstraint'))
        for constraint in constraints:
            parents = pm.listConnections(constraint, source=True, type='transform')
            ctrlParent = None
            for parent in parents:
                if str(parent) != ctrl and str(parent) != str(constraint):
                    ctrlParent = parent
                    break
            pm.parentConstraint(str(ctrlParent), str(constraint), e=True, maintainOffset=True)