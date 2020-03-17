import pymel.core as pm
import math

mainWindow = None # Stores main window instance

class MainWindow():
    def __init__(self):
        self.ctrlJoints = {}
        self.scrollList = None

# -- Functions -------------#

# Updates constraints
def UpdateConstraints(controller):
    constraints = pm.listRelatives(controller, children=True, type=('orientConstraint', 'parentConstraint'))
    for constraint in constraints:
        parents = pm.listConnections(constraint, source=True, type='transform')
        ctrlParent = None
        for parent in parents:
            if str(parent) != controller and str(parent) != str(constraint):
                ctrlParent = parent
                break
        print(str(constraint))
        pm.parentConstraint(str(ctrlParent), str(constraint), e=True, maintainOffset=True)

# Returns the children constrained to a controller as a set
def GetConstraintChildren(controller, childrenType):
    outChildren = set()
    constraints = set(pm.listConnections(controller, destination=True, type=('parentConstraint', 'orientConstraint', 'pointConstraint')))
    for constraint in constraints:
        children = set(pm.listConnections(constraint, destination=True, type=childrenType))
        outChildren = outChildren.union(children)
    return outChildren


# Creates and sets the joint offset and child joint attributes on the controllers
def SetControlAttributes(*args):
    selection = pm.selected()
    jntRot = pm.dt.TransformationMatrix(pm.xform(selection[1], worldSpace=True, matrix=True, q=True)).euler * 180 / math.pi
    ctrlRot = pm.dt.TransformationMatrix(pm.xform(selection[0], worldSpace=True, matrix=True, q=True)).euler * 180 / math.pi

    try:
        pm.getAttr(str(selection[0]) + ".childJoint")
    except:
        pm.addAttr(selection[0], longName="childJoint", dataType='string', niceName="Child Joint")     
        pm.addAttr(selection[0], longName="jointRotOffset", attributeType='double3', niceName="Joint Rotate Offset", hidden=False)        
        pm.addAttr(selection[0], longName="jointRotOffsetX", attributeType='double', niceName="Joint Rotate Offset X", parent="jointRotOffset")        
        pm.addAttr(selection[0], longName="jointRotOffsetY", attributeType='double', niceName="Joint Rotate Offset Y", parent="jointRotOffset")        
        pm.addAttr(selection[0], longName="jointRotOffsetZ", attributeType='double', niceName="Joint Rotate Offset Z", parent="jointRotOffset")        
        pm.addAttr(selection[0], longName="controllerRotOffset", attributeType='double3', niceName="Controller Rotate Offset", hidden=False)        
        pm.addAttr(selection[0], longName="controllerRotOffsetX", attributeType='double', niceName="Controller Rotate Offset X", parent="controllerRotOffset")        
        pm.addAttr(selection[0], longName="controllerRotOffsetY", attributeType='double', niceName="Controller Rotate Offset Y", parent="controllerRotOffset")        
        pm.addAttr(selection[0], longName="controllerRotOffsetZ", attributeType='double', niceName="Controller Rotate Offset Z", parent="controllerRotOffset")

        
    pm.setAttr(str(selection[0]) + ".childJoint", str(selection[1]))
    pm.setAttr(str(selection[0]) + ".jointRotOffset", jntRot)
    pm.setAttr(str(selection[0]) + ".controllerRotOffset", ctrlRot)

def SetIKAttributes(*args):
    selection = pm.selected()
    if pm.nodeType(selection[1]) != 'ikHandle':
        return

    handle = selection[1]    
    try:
        pm.getAttr(str(selection[0]) + ".childIK")
    except:
        pm.addAttr(selection[0], longName="childIK", dataType='string', niceName="Child IK Handle")
    try:
        pm.getAttr(str(handle) + ".snapController")
    except:
        pm.addAttr(handle, longName="snapController", dataType='string', niceName="Snap Controller")
        pm.addAttr(handle, longName="poleVectorController", dataType='string', niceName="Pole Vector Controller")
        pm.addAttr(handle, longName="poleVectorLength", attributeType='double', niceName="Pole Vector Length")

    poleConstraint = pm.listConnections(handle, source=True, type=('poleVectorConstraint'))[0]
    poleParents = pm.listConnections(poleConstraint, source=True, type='transform')
    poleCtrl = None
    for poleCtrl in poleParents:
        if poleCtrl != handle and pm.nodeType(poleCtrl) != "joint":
            break
    midJnt = pm.ikHandle(handle, q=True, jl=True)[1]
    P0 = pm.dt.Vector(pm.xform(midJnt, rotatePivot=True, q=True, worldSpace=True))
    P1 = pm.dt.Vector(pm.xform(poleCtrl, rotatePivot=True, q=True, worldSpace=True))

    pm.setAttr(str(selection[0]) + ".childIK", str(handle))
    pm.setAttr(str(handle) + ".poleVectorController", str(poleCtrl))
    pm.setAttr(str(handle) + ".poleVectorLength", P0.distanceTo(P1))



# Adds the selected joints to the control joints list and updates the scroll list display
def GetControlJoints(mainWindow):
    selection = pm.selected()
    ctrlJoints = {}
    
    for element in selection:
        try:
            childJoint = pm.getAttr(str(element) + ".childJoint")
        except:
            continue
        else:
            ctrlJoints[element] = pm.PyNode(childJoint)

    mainWindow.ctrlJoints = ctrlJoints
    mainWindow.scrollList.removeAll()
    mainWindow.scrollList.append(list(ctrlJoints.keys()))

# Snaps the selected controllers onto their child joint
def SnapFK(ctrlJoints):
    for ctrl, childJoint in ctrlJoints.items():
        # Rotation
        jntRot = pm.dt.TransformationMatrix(pm.xform(childJoint, worldSpace=True, matrix=True, q=True))
        jntRotOffset = pm.dt.TransformationMatrix(pm.dt.EulerRotation(pm.getAttr(str(ctrl) + ".jointRotOffset")))
        ctrlRotOffset = pm.dt.TransformationMatrix(pm.dt.EulerRotation(pm.getAttr(str(ctrl) + ".controllerRotOffset")))
        deltaMat = pm.dt.TransformationMatrix(jntRotOffset.asMatrixInverse() * jntRot)
        newRot = (ctrlRotOffset * deltaMat).euler * 180 / math.pi

        pm.xform(ctrl, rotation=newRot, worldSpace=True)

# Snaps IK handle angles of selected controllers to the FK joints
def SnapIK(ctrlJoints):
    for ctrl in ctrlJoints:
        # Snap IK controller to joint
        newPos = pm.dt.Vector(pm.xform(ctrlJoints[ctrl], rotatePivot=True, q=True, worldSpace=True))
        initialPos = pm.dt.Vector(pm.xform(ctrl, rotatePivot=True, q=True, worldSpace=True)) - \
                 pm.dt.Vector(pm.xform(ctrl, translation=True, q=True, worldSpace=True))
        pm.xform(ctrl, translation=newPos-initialPos, worldSpace=True)

        try:
            pm.getAttr(str(ctrl) + ".childIK")
        except:
            continue
        else:
            handle = pm.getAttr(str(ctrl) + ".childIK")
            snapCtrl = pm.getAttr(str(handle) + ".snapController")
            if not snapCtrl:
                print("No snap controller has been set for IK handle {}".format(str(handle)))
                continue

            affectedJoints = pm.ikHandle(handle, q=True, jl=True)
            poleCtrl = pm.getAttr(str(handle) + ".poleVectorController")

            P0 = pm.dt.Vector(pm.xform(affectedJoints[0], rotatePivot=True, q=True, worldSpace=True))
            P1 = pm.dt.Vector(pm.xform(affectedJoints[1], rotatePivot=True, q=True, worldSpace=True))
            P2 = pm.dt.Vector(pm.xform(handle, rotatePivot=True, q=True, worldSpace=True))
            P4 = pm.dt.Vector(pm.xform(poleCtrl, rotatePivot=True, q=True, worldSpace=True)) - \
                 pm.dt.Vector(pm.xform(poleCtrl, translation=True, q=True, worldSpace=True))
            altP1 = pm.dt.Vector(pm.xform(snapCtrl, rotatePivot=True, q=True, worldSpace=True))

            v0 = P2 - P1
            v1 = P2 - P0
            v2 = P1 - P0
            altV2 = altP1 - P0
            map(lambda x: x.normalize(), (v0, v1, v2, altV2))

            # Find plane normal vectors
            N1 = v1.cross(v2)
            N2 = v1.cross(altV2)            
            map(lambda x: x.normalize(), (N1, N2))
            
            # Find vector from the handle to the zero-translate pole vector controller
            v3 = (N1.cross(v0)  - v0).normal()

            a = math.acos(N1.dot(N2))
            R1 = pm.dt.TransformationMatrix()
            R1.setToRotationAxis(N1.cross(N2), -a)
            P3 = R1 * v3 * (P4 - P2).length()

            pm.xform(poleCtrl, translation=P2 + P3 - P4, worldSpace=True)

# Resets the selected controllers
def ResetControllers(ctrlJoints):
    for ctrl in ctrlJoints:
        pm.xform(ctrl, translation=[0,0,0], rotation=[0,0,0], objectSpace=True)
        UpdateConstraints(ctrl)
        
        # Reset IK twist
        try:
            pm.getAttr(str(ctrl) + ".childIK")
        except:
            continue
        else:
            pm.setAttr(str(ctrl) + ".IKTwist", 0)

# Updates the active constraints to avoid auto snapping on parent change
def MaintainOffset(ctrlJoints):
    for ctrl in ctrlJoints:
        UpdateConstraints(ctrl)

# Creates and initializes the main window
def SnapFKWindow(*arg):
    global mainWindow
    if not mainWindow:
        mainWindow = MainWindow();
    pm.window(title='Snap IK/FK Controllers', width=300)
    pm.columnLayout(adjustableColumn=True)
    pm.separator()
    pm.button(label='Set Controller Attributes', command='SetControlAttributes()')
    pm.separator()
    pm.button(label='Set IK Handle Attributes', command='SetIKAttributes()')
    pm.separator()
    pm.button(label='Select Controllers', command='ctrlJoints = GetControlJoints(mainWindow)')
    pm.separator()
    mainWindow.scrollList = pm.textScrollList('Active controllers')
    mainWindow.scrollList.append(list(mainWindow.ctrlJoints.keys()))
    pm.button(label='Snap Controllers to Joints', command='SnapFK(mainWindow.ctrlJoints)')
    pm.separator()
    pm.button(label='Snap IK Handles to Joints', command='SnapIK(mainWindow.ctrlJoints)')
    pm.separator()
    pm.button(label='Update Constraints', command='MaintainOffset(mainWindow.ctrlJoints)')
    pm.separator()
    pm.button(label='Reset Controllers', command='ResetControllers(mainWindow.ctrlJoints)')
    
    pm.showWindow()

# -- Create menu ----------#
main_window = pm.language.melGlobals['gMainWindow']

menu_obj = 'CustomTools'
menu_label = 'Custom Tools'

bMenuExists = pm.menu(menu_obj, exists=True)
if not bMenuExists:
    custom_tools_menu = pm.menu(menu_obj, label=menu_label, parent=main_window, tearOff=True)

# -- Create menu item -----#
if pm.menuItem('SnapFK', exists=True):
    pm.deleteUI('SnapFK')
pm.menuItem('SnapFK', label='Snap IK/FK...', command=SnapFKWindow, parent=menu_obj)
pm.setParent('..', menu=True)