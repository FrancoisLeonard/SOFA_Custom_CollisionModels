import Sofa

class MainScene(Sofa.PythonScriptController):

    def createGraph(self, node):

        dt = 0.02 # In second

        self.rootNode = node.getRoot()
        self.rootNode.dt = dt
        self.rootNode.gravity = [0, -9.8, 0]

        self.rootNode.createObject('APIVersion', name = 17.12)
        self.rootNode.createObject('RequiredPlugin', name = 'SofaScytherInterface')
        self.rootNode.createObject('RequiredPlugin', name = 'SofaPython')
        self.rootNode.createObject('RequiredPlugin', name = 'SofaOpenglVisual')

        self.rootNode.createObject('VisualStyle', displayFlags="showBehaviorModels hideVisualModels showInteractionForceFields showCollisionModels")

        self.rootNode.createObject('FreeMotionAnimationLoop')

        self.rootNode.createObject('DefaultPipeline')
        self.rootNode.createObject('ScytherDetection', name = 'Detection')
        self.rootNode.createObject('ScytherIntersection', name='Intersection', alarmDistance='0.5', contactDistance='1')
        #self.rootNode.createObject('DefaultContactManager', name='Response', response='default', printLog="1")

        self.rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')
        #self.rootNode.createObject('GenericConstraintSolver', tolerance = '0.001', maxIterations='1000', mu='0')
        self.rootNode.createObject('LCPConstraintSolver', tolerance = '0.001', maxIt = '1000', mu='0.001')



        ##############
        ## OBJECT 1 ##
        ##############
        meshFile1 = '/home/franc/sofaScyther/models/cube.obj'
#        meshFile = '/home/vincent/Documents/code/sofa/plugins/SofaScyther/torus2_scale3_from_Unreal.obj'
#        meshFile = '/home/vincent/Documents/code/sofa/plugins/SofaScyther/efgSurfaceDump.obj'
#        meshFile = 'mesh/dragon.obj'

        object1Translation = [0, 5, 0]
        object1Rotation = [0, 0, 0]

        objectNode1 = self.rootNode.createChild('ObjectNode1')

        # Solvers
        objectNode1.createObject('EulerImplicitSolver', rayleighStiffness = 0.1, rayleighMass = 0.1, printLog = False, verbose = False)
        objectNode1.createObject('CGLinearSolver', iterations = 25, tolerance=1e-9, threshold=1e-9, printLog = True, verbose = False)

        objectNode1.createObject('DisplacedMeshTopology', translation = object1Translation,
                                                          rotation    = object1Rotation,
                                                          filename    = meshFile1)
        objectNode1.createObject('SofaScytherObject', name = 'EFG_manager1', printLog=True,
                                elemSize = 1.05,
                                numSamplingDivisions = 2,
                                numNeighbors = 8,
                                useCorotation = True,
                                youngModulus = 40000,
                                poissonRatio = 0.45,
                                triangleGenSizeRatio = 0.1,
                                drawIntegrationPoints = False,
                                drawNeighborhoods = False,
                                drawDofs = True,
                                drawLinks = False,
                                drawSurface = True,
                                drawSurfaceMapping = False,
                                drawError = False,
                                drawBoundingBox = True,
                                fixedBox = [-3, 3.5, -3, 3, 4.5, 3],
                                )
        objectNode1.createObject('MechanicalObject', name = 'MechAfter1', showObject = False, showObjectScale = 1,
                                position = "@EFG_manager1.positions")
        objectNode1.createObject('ScytherForceField', name = 'EFG_force', printLog=True)

#        objectNode1.createObject('UniformMass', totalMass = 1000)
        objectNode1.createObject('ParticleDiagonalMass')
        objectNode1.createObject('UncoupledConstraintCorrection');


        # Collisions
#        contactStiffness = 250;

        objectNode1ColSurf = objectNode1.createChild('Collision_Surf_1')
        contactMesh_1 = objectNode1ColSurf.createObject('MeshTopology', name = 'CollisionSurface_1', filename = meshFile1)
        objectNode1ColSurf.createObject('MechanicalObject', name='Col_Mech_1', src = contactMesh_1,
            dx = object1Translation[0], dy = object1Translation[1], dz = object1Translation[2],
            rx = object1Rotation[0],    ry = object1Rotation[1],    rz = object1Rotation[2])
        objectNode1ColSurf.createObject('ScytherTriangleCollisionModel', name='Tri1')
#        objectNode1ColSurf.createObject('LineCollisionModel', name='Line1')
        objectNode1ColSurf.createObject('ScytherPointCollisionModel', name='Point1')
        objectNode1ColSurf.createObject('SofaCollisionModelMapping', name='Col_Model_Mapping_1')



        ##############
        ## OBJECT 2 ##
        ##############
        meshFile2 = '/home/franc/sofaScyther/models/cube.obj'

        object2Translation = [0, 10, 0]
        object2Rotation = [0, 0, 0]

        objectNode2 = self.rootNode.createChild('ObjectNode2')

        # Object 2
        objectNode2.createObject('EulerImplicitSolver', rayleighStiffness = 0.1, rayleighMass = 0.1, printLog = False, verbose = False)
        objectNode2.createObject('CGLinearSolver', iterations = 25, tolerance=1e-9, threshold=1e-9, printLog = True, verbose = False)

        objectNode2.createObject('DisplacedMeshTopology', translation = object2Translation,
                                                          rotation    = object2Rotation,
                                                          filename = meshFile2)
        objectNode2.createObject('SofaScytherObject', name = 'EFG_manager2', printLog=True,
                                elemSize = 1.05,
                                numSamplingDivisions = 2,
                                numNeighbors = 8,
                                useCorotation = True,
                                youngModulus = 40000,
                                poissonRatio = 0.45,
                                triangleGenSizeRatio = 0.1,
                                drawIntegrationPoints = False,
                                drawNeighborhoods = False,
                                drawDofs = True,
                                drawLinks = False,
                                drawSurface = True,
                                drawSurfaceMapping = False,
                                drawError = False,
                                fixedBox = [-3, 3.5, -3, 3, 4.5, 3]
                                )
        objectNode2.createObject('MechanicalObject', name = 'MechAfter2', showObject = False, showObjectScale = 1,
                                position = "@EFG_manager2.positions")
        objectNode2.createObject('ScytherForceField', name = 'EFG_force2', printLog=True)

#        objectNode2.createObject('UniformMass', totalMass = 1000)
        objectNode2.createObject('ParticleDiagonalMass')
        objectNode2.createObject('UncoupledConstraintCorrection');


        # Collision
#        contactStiffness = 250;

        objectNode2ColSurf = objectNode2.createChild('Collision_Surf_2')
        contactMesh_2 = objectNode2ColSurf.createObject('MeshTopology', name = 'CollisionSurface_2', filename = meshFile2)
        objectNode2ColSurf.createObject('MechanicalObject', name='Col_Mech2', src = contactMesh_2,
            dx = object2Translation[0], dy = object2Translation[1], dz = object2Translation[2],
            rx = object2Rotation[0],    ry = object2Rotation[1],    rz = object2Rotation[2])
        objectNode2ColSurf.createObject('ScytherTriangleCollisionModel', name='Tri2')
#        objectNode2ColSurf.createObject('LineCollisionModel', name='Line2')
        objectNode2ColSurf.createObject('ScytherPointCollisionModel', name='Point2')
        objectNode2ColSurf.createObject('SofaCollisionModelMapping', name='Col_Model_Mapping_2')


        #################
        ## FLOOR       ##
        #################
#        floorMeshFile = 'mesh/floor.obj'

#        floorTranslation = [0, 0, 0]
#        floorRotation    = [0, 0, 0]

#        floorNode = self.rootNode.createChild('Floor')
#        floorNode.createObject('MeshObjLoader', name='loader', filename=floorMeshFile)
#        floorNode.createObject('MeshTopology', src='@loader')
#        floorNode.createObject('MechanicalObject', src='@loader',
#             dx = floorTranslation[0], dy = floorTranslation[1], dz = floorTranslation[2],
#             rx = floorRotation[0],    ry = floorRotation[1],    rz = floorRotation[2],
#             scale='1.0')
#        floorNode.createObject('TriangleCollisionModel', name='FloorTri',   simulated='0', moving='0')#, contactStiffness = floorContactStiffness)
#        floorNode.createObject('LineCollisionModel',     name='FloorLine',  simulated='0', moving='0')#, contactStiffness = floorContactStiffness)
#        floorNode.createObject('PointCollisionModel',    name='FloorPoint', simulated='0', moving='0')#, contactStiffness = floorContactStiffness)
#        floorNode.createObject('OglModel', name='FloorV', filename=floorMeshFile, scale='1', texturename='textures/floor.bmp')#, dy='-20')



        #################
        ## CONSTRAINTS ##
        #################
        self.fixedNodes = objectNode1.createObject('FixedConstraint', indices = '@EFG_manager1.fixedDofs')


        #################
        ## CONSTRAINTS ##
        #################
        #fixedBox = objectNode1.createObject('BoxROI', name='fixed_box', position = '@MechAfter.position',
        #                                    box = [-10, 6.0, -4, 10, 8, 4], drawBoxes = False, doUpdate = True)
        #objectNode1.createObject('FixedConstraint', indices = '@EFG_manager1.fixedDofs')
        
#        objectNode1.createObject('ConstantForceField', indices = [83], totalForce = [1200, -800, -200])

#        forceBox = objectNode1.createObject('BoxROI', name = 'force_box', position = '@MechAfter.position',
#                                            box = [-10, -8, -4, 10, -5, 4], drawBoxes = True)
#        objectNode1.createObject('ConstantForceField', indices = '@force_box.indices', totalForce = [100, 0, 0])

        ##########
        ## TOOL ##
        ##########
        
#        objectNode1.createObject('MouseMapping', name = "mousePos", moveSpeed = 0.8, printLog = True)
        


        return 0

def createScene(rootNode):
    obj = MainScene(rootNode)
    obj.createGraph(rootNode)
