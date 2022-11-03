# import Sofa.Core

# def createScene(rootNode):
        # rootNode.addObject("OglGrid", nbSubdiv=10, size=1000)

        # rootNode.findData('gravity').value=[0.0,-981.0,0.0];
        # rootNode.findData('dt').value=0.01

        # confignode = rootNode.addChild("Config")
        # confignode.addObject('RequiredPlugin', name="SofaMiscCollision", printLog=False)
        # confignode.addObject('RequiredPlugin', name="SofaPython3", printLog=False)
        # confignode.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D", printLog=False)
        # confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm", printLog=False)
        # confignode.addObject('RequiredPlugin', name="Sofa.Component.AnimationLoop", printLog=False)
        
        # confignode.addObject('RequiredPlugin', name="Sofa.Component.Mass", printLog=False)
        # confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
        


        # #Collision function

        # rootNode.addObject('DefaultPipeline')
        # rootNode.addObject('FreeMotionAnimationLoop')
        # rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
        # rootNode.addObject('BruteForceDetection')
        # rootNode.addObject('RuleBasedContactManager', responseParams="mu="+str(0.0), name='Response', response='FrictionContactConstraint')
        # rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5, angleCone=0.01)

        # ### Mechanical model

        # totalMass = 1.0
        # volume = 1.0
        # inertiaMatrix=[1., 0., 0., 0., 1., 0., 0., 0., 1.]

        # #Creating the floor
        # floor = rootNode.addChild("floor")

        # floor.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0.0,-300.0,0.0], rotation2=[0., 0., 0.], showObjectScale=5.0)

        # floor.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
        # floorCollis = floor.addChild('collision')
        # floorCollis.addObject('MeshObjLoader', name="loader", filename="/home/yi/peg_in_hole/SofaGym/mesh/floor.obj", triangulate="true")
        # floorCollis.addObject('MeshTopology', src="@loader")
        # floorCollis.addObject('MechanicalObject')
        # floorCollis.addObject('TriangleCollisionModel', moving=False, simulated=False)
        # floorCollis.addObject('LineCollisionModel', moving=False, simulated=False)
        # floorCollis.addObject('PointCollisionModel', moving=False, simulated=False)

        # floorCollis.addObject('RigidMapping')

        # #### visualization
        # floorVisu = floor.addChild("VisualModel")
        # floorVisu.loader = floorVisu.addObject('MeshObjLoader', name="loader", filename="/home/yi/peg_in_hole/SofaGym/mesh/floor.obj")
        # floorVisu.addObject('OglModel', name="model", src="@loader", scale3d=[5.0]*3, color=[1., 1., 0.], updateNormals=False)
        # floorVisu.addObject('RigidMapping')

        # #Creating the sphere
        # sphere = rootNode.addChild("sphere")
        # sphere.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0., 0., 0.], rotation2=[0., 0., 0.], showObjectScale=50)
        # sphere.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
        # sphere.addObject('UncoupledConstraintCorrection')

        # sphere.addObject('EulerImplicitSolver', name='odesolver')
        # sphere.addObject('CGLinearSolver', name='Solver')

        # collision = sphere.addChild('collision')
        # collision.addObject('MeshObjLoader', name="loader", filename="/home/yi/peg_in_hole/SofaGym/mesh/ball.obj", triangulate="true")

        # collision.addObject('MeshTopology', src="@loader")
        # collision.addObject('MechanicalObject')
        # collision.addObject('TriangleCollisionModel')
        # collision.addObject('LineCollisionModel')
        # collision.addObject('PointCollisionModel')

        # collision.addObject('RigidMapping')

        # #### visualization
        # sphereVisu = sphere.addChild("VisualModel")
        # sphereVisu.loader = sphereVisu.addObject('MeshObjLoader', name="loader", filename="/home/yi/peg_in_hole/SofaGym/mesh/ball.obj")
        # sphereVisu.addObject('OglModel', name="model", src="@loader", scale3d=[50]*3, color=[0., 1., 0.], updateNormals=False)
        # sphereVisu.addObject('RigidMapping')

        # return rootNode
# root = Sofa.Core.Node("root")
# createScene(root)
"""Implementation of a RestShapeForceField in python"""
# coding: utf8
import Sofa
import numpy as np


class RestShapeForceField(Sofa.Core.ForceFieldVec3d):
    """Implementation of a RestShapeForceField in python"""
    def __init__(self, ks=1.0, kd=1.0, *args, **kwargs):
        Sofa.Core.ForceFieldVec3d.__init__(self, *args, **kwargs)
        self.addData("ks", type="float", value=ks, help="The stiffness spring", group="Spring's Properties")                  
        self.addData("kd", type="float", value=kd, help="The damping spring", group="Spring's Properties")                  
        
    def init(self):
        mstate = self.getContext().mechanical
        self.initpos = mstate.position.array().copy()
        self.k = np.zeros((1,1))
        self.f = []
        self.d = 0.5

    def addForce(self, m, out_force, pos, vel):
        with out_force.writeableArray() as wa:
            wa[:] += ( (self.initpos-pos.value) * self.ks.value  )
                 
    def addDForce(self, df, dx, params):
        pass
        
    #def addKToMatrix(self, a, b):
    #    print(" Python::addKToMatrix: ", a, " ", b)


def createScene(node):
        node.addObject("RequiredPlugin", name="Sofa.GL.Component")
        node.addObject("RequiredPlugin", name="Sofa.Component.LinearSolver.Direct")
        node.addObject("RequiredPlugin", name="Sofa.Component.ODESolver.Backward")
        node.addObject("OglLineAxis")
        node.addObject("DefaultAnimationLoop", name="loop")
        node.addObject("EulerImplicitSolver")
        node.addObject('RequiredPlugin', name="Sofa.Component.Mass", printLog=False)
        node.addObject("CGLinearSolver", tolerance=1e-12, threshold=1e-12, iterations=25)

        o = node.addChild("Object")
        c = o.addObject("MechanicalObject", name="mechanical", position=[0.0,0.0,0.0, 1.0,0.0,0.0])
        c.showObject = True
        c.showColor = [1.0,0.0,0.0,1.0]
        c.drawMode = 1

        o.addObject("UniformMass", name="mass", totalMass=[0.1])
        o.addObject( RestShapeForceField(name="CPPObject", ks=2.0, kd=0.1))

        return node


def main():
    import SofaRuntime
    import Sofa.Gui
    SofaRuntime.importPlugin("SofaOpenglVisual")
    SofaRuntime.importPlugin("Sofa.Component.StateContainer")
    SofaRuntime.importPlugin("SofaImplicitOdeSolver")

    root=Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)

    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

    print("End of simulation.")


if __name__ == '__main__':
    main()