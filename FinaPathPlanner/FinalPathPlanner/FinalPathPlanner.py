import logging
import os
from typing import Annotated, Optional

import vtk

import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)

from slicer import vtkMRMLScalarVolumeNode
import logging
import os
import time
from typing import Annotated, Optional

import vtk
import slicer
import numpy as np
import SimpleITK as sitk
import sitkUtils

from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)

from slicer import vtkMRMLScalarVolumeNode

#
# FinalPathPlanner
#


class FinalPathPlanner(ScriptedLoadableModule):
    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("FinalPathPlanner")
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "Examples")]
        self.parent.dependencies = []
        self.parent.contributors = ["Safa Khan"]
        self.parent.helpText = _("""This module selects a safe straight trajectory from entry to target, avoiding critical structures.""")
        self.parent.acknowledgementText = _("""This work was supported by coursework at King's College London.""")

# Register sample data sets in Sample Data module
def registerSampleData():
    """Add data sets to Sample Data module."""
    # It is always recommended to provide sample data for users to make it easy to try the module,
    # but if no sample data is available then this method (and associated startupCompeted signal connection) can be removed.

    import SampleData

    iconsPath = os.path.join(os.path.dirname(__file__), "Resources/Icons")

    # To ensure that the source code repository remains small (can be downloaded and installed quickly)
    # it is recommended to store data sets that are larger than a few MB in a Github release.

    # FinalPathPlanner1
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category="FinalPathPlanner",
        sampleName="FinalPathPlanner1",
        # Thumbnail should have size of approximately 260x280 pixels and stored in Resources/Icons folder.
        # It can be created by Screen Capture module, "Capture all views" option enabled, "Number of images" set to "Single".
        thumbnailFileName=os.path.join(iconsPath, "FinalPathPlanner1.png"),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        fileNames="FinalPathPlanner1.nrrd",
        # Checksum to ensure file integrity. Can be computed by this command:
        #  import hashlib; print(hashlib.sha256(open(filename, "rb").read()).hexdigest())
        checksums="SHA256:998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        # This node name will be used when the data set is loaded
        nodeNames="FinalPathPlanner1",
    )

    # FinalPathPlanner2
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category="FinalPathPlanner",
        sampleName="FinalPathPlanner2",
        thumbnailFileName=os.path.join(iconsPath, "FinalPathPlanner2.png"),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        fileNames="FinalPathPlanner2.nrrd",
        checksums="SHA256:1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        # This node name will be used when the data set is loaded
        nodeNames="FinalPathPlanner2",
    )


#
# FinalPathPlannerParameterNode
#


@parameterNodeWrapper
class FinalPathPlannerParameterNode:
    """
    The parameters needed by module.

    inputVolume - The volume to threshold.
    imageThreshold - The value at which to threshold the input volume.
    invertThreshold - If true, will invert the threshold.
    thresholdedVolume - The output volume that will contain the thresholded volume.
    invertedVolume - The output volume that will contain the inverted thresholded volume.
    """

    inputVolume: vtkMRMLScalarVolumeNode
    imageThreshold: Annotated[float, WithinRange(-100, 500)] = 100
    invertThreshold: bool = False
    thresholdedVolume: vtkMRMLScalarVolumeNode
    invertedVolume: vtkMRMLScalarVolumeNode


#
# FinalPathPlannerWidget
#


class FinalPathPlannerWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent=None) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)  # needed for parameter node observation
        self.logic = None
        self._parameterNode = None
        self._parameterNodeGuiTag = None

    def setup(self):
        print("Reload successful!")
        ScriptedLoadableModuleWidget.setup(self)

        uiWidget = slicer.util.loadUI(self.resourcePath("UI/FinalPathPlanner.ui"))
        self.layout.addWidget(uiWidget)
        self.ui = slicer.util.childWidgetVariables(uiWidget)

        # Set scene in MRML widgets only for the node selectors, not labels or sliders
        self.ui.targetLabelMapSelector.setMRMLScene(slicer.mrmlScene)
        self.ui.criticalStructureLabelMapSelector.setMRMLScene(slicer.mrmlScene)
        self.ui.entryPointsSelector.setMRMLScene(slicer.mrmlScene)
        self.ui.targetPointsSelector.setMRMLScene(slicer.mrmlScene)
        self.ui.TestButton.connect("clicked(bool)", self.onRunTestsButton)

        # Set up slider (no need for setMRMLScene here)
        self.ui.lengthThresholdSlider.minimum = 1
        self.ui.lengthThresholdSlider.maximum = 500
        self.ui.lengthThresholdSlider.value = 100

        # Initialize logic etc...
        self.logic = FinalPathPlannerLogic()

        # Other connections...
        self.ui.applyButton.connect("clicked(bool)", self.onApplyButton)

    def cleanup(self) -> None:
        """Called when the application closes and the module widget is destroyed."""
        self.removeObservers()

    def enter(self) -> None:
        """Called each time the user opens this module."""
        # Make sure parameter node exists and observed
        self.initializeParameterNode()

    def exit(self) -> None:
        """Called each time the user opens a different module."""
        # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
        if self._parameterNode:
            self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            self._parameterNodeGuiTag = None
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)

    def onSceneStartClose(self, caller, event) -> None:
        """Called just before the scene is closed."""
        # Parameter node will be reset, do not use it anymore
        self.setParameterNode(None)

    def onSceneEndClose(self, caller, event) -> None:
        """Called just after the scene is closed."""
        # If this module is shown while the scene is closed then recreate a new parameter node immediately
        if self.parent.isEntered:
            self.initializeParameterNode()

    def onRunTestsButton(self):
        import FinalPathPlanner
        import importlib
        importlib.reload(FinalPathPlanner)  # reload in case of changes

        from FinalPathPlanner import FinalPathPlannerTest

        tester = FinalPathPlannerTest()
        tester.setUp = lambda: None  # optional, if you want to keep current scene data
        tester.runTest()

    def initializeParameterNode(self) -> None:
        """Ensure parameter node exists and observed."""
        # Parameter node stores all user choices in parameter values, node selections, etc.
        # so that when the scene is saved and reloaded, these settings are restored.

        self.setParameterNode(self.logic.getParameterNode())

        # Select default input nodes if nothing is selected yet to save a few clicks for the user
        if not self._parameterNode.inputVolume:
            firstVolumeNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLScalarVolumeNode")
            if firstVolumeNode:
                self._parameterNode.inputVolume = firstVolumeNode

    def setParameterNode(self, inputParameterNode: Optional[FinalPathPlannerParameterNode]) -> None:
        """
        Set and observe parameter node.
        Observation is needed because when the parameter node is changed then the GUI must be updated immediately.
        """

        if self._parameterNode:
            self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
        self._parameterNode = inputParameterNode
        if self._parameterNode:
            # Note: in the .ui file, a Qt dynamic property called "SlicerParameterName" is set on each
            # ui element that needs connection.
            self._parameterNodeGuiTag = self._parameterNode.connectGui(self.ui)
            self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
            self._checkCanApply()

    def _checkCanApply(self, caller=None, event=None) -> None:
        if self.ui.targetLabelMap.currentNode() and self.ui.criticalStructureLabelMap.currentNode() and \
                self.ui.entryPoints.currentNode() and self.ui.targetPoints.currentNode():
            self.ui.applyButton.toolTip = _("Compute output volume")
            self.ui.applyButton.enabled = True
        else:
            self.ui.applyButton.toolTip = _("Select all input nodes")
            self.ui.applyButton.enabled = False

    def onApplyButton(self) -> None:
        with slicer.util.tryWithErrorDisplay(_("Failed to compute results."), waitCursor=True):
            targetNode = self.ui.targetLabelMapSelector.currentNode()
            criticalNode = self.ui.criticalStructureLabelMapSelector.currentNode()
            entryNode = self.ui.entryPointsSelector.currentNode()
            targetPointsNode = self.ui.targetPointsSelector.currentNode()
            maxLength = self.ui.lengthThresholdSlider.value

            validTrajectories, rejected = self.logic.findAllValidTrajectories(
                entryNode, targetPointsNode, criticalNode, targetNode, maxLength)

            if validTrajectories:
                bestTrajectory = max(validTrajectories, key=lambda x: x[2])
                bestEntry, bestTarget, bestScore = bestTrajectory

                print(f"Best trajectory score: {bestScore:.3f}, rejected: {rejected}")
                print(f"Entry point coordinates: {bestEntry}")
                print(f"Target point coordinates: {bestTarget}")

                self.logic.plotTrajectory((bestEntry, bestTarget))
            else:
                print(f"No valid trajectories found. Rejected: {rejected}")


#
# FinalPathPlannerLogic
#
class FinalPathPlannerLogic(ScriptedLoadableModuleLogic):
    def __init__(self):
        ScriptedLoadableModuleLogic.__init__(self)

    def trajectoryCollides(self, entry, target, labelMapNode, numSamples=500):
        if labelMapNode is None or labelMapNode.GetImageData() is None:
            return False

        mat = vtk.vtkMatrix4x4()
        labelMapNode.GetRASToIJKMatrix(mat)

        transform = vtk.vtkTransform()
        transform.SetMatrix(mat)

        linePoints = np.linspace(entry, target, numSamples)

        for point in linePoints:
            idx = transform.TransformPoint(point)
            x, y, z = int(idx[0]), int(idx[1]), int(idx[2])
            dims = labelMapNode.GetImageData().GetDimensions()
            if not (0 <= x < dims[0] and 0 <= y < dims[1] and 0 <= z < dims[2]):
                continue

            voxelValue = labelMapNode.GetImageData().GetScalarComponentAsDouble(x, y, z, 0)
            if voxelValue == 1:
                return True
        return False

    def computeDistanceMap(self, labelMapNode):
        distanceFilter = vtk.vtkImageEuclideanDistance()
        distanceFilter.SetInputData(labelMapNode.GetImageData())
        distanceFilter.SetConsiderAnisotropy(True)
        distanceFilter.Update()
        return distanceFilter.GetOutput()

    def sampleTrajectoryDistances(self, entry, target, distanceMap, labelMapNode, numSamples=500):
        linePoints = np.linspace(entry, target, numSamples)
        distances = []

        rasToIjk = vtk.vtkMatrix4x4()
        labelMapNode.GetRASToIJKMatrix(rasToIjk)

        dims = distanceMap.GetDimensions()

        for point in linePoints:
            ijk = [0, 0, 0, 0]
            rasToIjk.MultiplyPoint(list(point) + [1.0], ijk)
            i, j, k = map(int, ijk[:3])

            if (0 <= i < dims[0]) and (0 <= j < dims[1]) and (0 <= k < dims[2]):
                dist = distanceMap.GetScalarComponentAsDouble(i, j, k, 0)
                distances.append(dist)
            else:
                distances.append(0)

        if distances:
            return sum(distances) / len(distances)  # average distance along trajectory
        return 0

    def isPointInsideLabel(self, point, labelMapNode, neighborhood=1):
        if labelMapNode is None or labelMapNode.GetImageData() is None:
            return False

        mat = vtk.vtkMatrix4x4()
        labelMapNode.GetRASToIJKMatrix(mat)

        ijk_float = [0.0, 0.0, 0.0, 0.0]
        mat.MultiplyPoint(list(point) + [1.0], ijk_float)
        i_center, j_center, k_center = map(int, ijk_float[:3])

        dims = labelMapNode.GetImageData().GetDimensions()

        for di in range(-neighborhood, neighborhood + 1):
            for dj in range(-neighborhood, neighborhood + 1):
                for dk in range(-neighborhood, neighborhood + 1):
                    i = i_center + di
                    j = j_center + dj
                    k = k_center + dk

                    if not (0 <= i < dims[0] and 0 <= j < dims[1] and 0 <= k < dims[2]):
                        continue

                    voxel_value = labelMapNode.GetImageData().GetScalarComponentAsDouble(i, j, k, 0)
                    if voxel_value > 0:
                        return True

        return False

    def findAllValidTrajectories(self, entryFiducials, targetFiducials, criticalStructureNode, targetVolumeNode,
                                 maxLength, minSafeDistance=5.0):  # single critical structure

        distanceMap = self.computeDistanceMap(criticalStructureNode)

        validTrajectories = []
        rejected = 0
        epsilon = 1e-3  # small floor value to avoid zero scores

        for i in range(entryFiducials.GetNumberOfControlPoints()):
            entry = [0.0, 0.0, 0.0]
            entryFiducials.GetNthControlPointPositionWorld(i, entry)

            for j in range(targetFiducials.GetNumberOfControlPoints()):
                target = [0.0, 0.0, 0.0]
                targetFiducials.GetNthControlPointPositionWorld(j, target)

                # Check target inside target structure
                if not self.isPointInsideLabel(target, targetVolumeNode):
                    rejected += 1
                    continue

                # Reject if entry and target points are too close or identical
                if vtk.vtkMath.Distance2BetweenPoints(entry, target) < 1e-6:
                    rejected += 1
                    continue

                # Reject if trajectory length exceeds maximum allowed
                distanceSquared = vtk.vtkMath.Distance2BetweenPoints(entry, target)
                distance = distanceSquared ** 0.5
                if distance > maxLength:
                    rejected += 1
                    continue

                # Reject if trajectory collides with the critical structure
                if self.trajectoryCollides(entry, target, criticalStructureNode):
                    rejected += 1
                    continue

                # Compute safety score based on average distance to critical structure along trajectory
                score = self.sampleTrajectoryDistances(entry, target, distanceMap, criticalStructureNode)

                # Normalize score and ensure minimum floor epsilon
                normalized_score = max(score / (minSafeDistance + 1e-6), epsilon)

                validTrajectories.append((entry, target, normalized_score))

        return validTrajectories, rejected

    def plotTrajectory(self, trajectory):
        lineSource = vtk.vtkLineSource()
        lineSource.SetPoint1(trajectory[0])
        lineSource.SetPoint2(trajectory[1])
        lineSource.Update()

        modelNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode')
        modelNode.SetName("BestTrajectory")
        modelNode.SetAndObservePolyData(lineSource.GetOutput())

        displayNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelDisplayNode')
        modelNode.SetAndObserveDisplayNodeID(displayNode.GetID())

        displayNode.SetColor(1, 1, 0)  # bright yellow color
        displayNode.SetLineWidth(8)  # thick line
        displayNode.SetVisibility(True)

    def getParameterNode(self):
        return FinalPathPlannerParameterNode(super().getParameterNode())

    def process(self,
                inputVolume: vtkMRMLScalarVolumeNode,
                outputVolume: vtkMRMLScalarVolumeNode,
                imageThreshold: float,
                invert: bool = False,
                showResult: bool = True) -> None:

        if not inputVolume or not outputVolume:
            raise ValueError("Input or output volume is invalid")

        import time
        startTime = time.time()
        logging.info("Processing started")

        cliParams = {
            "InputVolume": inputVolume.GetID(),
            "OutputVolume": outputVolume.GetID(),
            "ThresholdValue": imageThreshold,
            "ThresholdType": "Above" if invert else "Below",
        }
        cliNode = slicer.cli.run(slicer.modules.thresholdscalarvolume, None, cliParams, wait_for_completion=True, update_display=showResult)
        slicer.mrmlScene.RemoveNode(cliNode)

        stopTime = time.time()
        logging.info(f"Processing completed in {stopTime-startTime:.2f} seconds")


class FinalPathPlannerTest(ScriptedLoadableModuleTest):
    """
    Test case for the FinalPathPlanner module logic with updated critical structure parameter.
    """

    def setUp(self):
        # Optionally skip clearing scene to keep data
        pass

    def runTest(self):
        self.setUp()

        print("Running test_FinalPathPlanner1")
        self.test_FinalPathPlanner1()
        print("test_FinalPathPlanner1 passed\n")

        print("Running test_PositiveTrajectorySelection")
        self.test_PositiveTrajectorySelection()
        print("test_PositiveTrajectorySelection passed\n")

        print("Running test_NegativeEmptyInputs")
        self.test_NegativeEmptyInputs()
        print("test_NegativeEmptyInputs passed\n")

        print("Running test_EdgeMaxLength")
        self.test_EdgeMaxLength()
        print("test_EdgeMaxLength passed\n")

        print("Running test_NegativeCollisionWithCriticalStructure")
        self.test_NegativeCollisionWithCriticalStructure()
        print("test_NegativeCollisionWithCriticalStructure passed\n")

        print("Running test_EdgeTrajectoryExceedsMaxLength")
        self.test_EdgeTrajectoryExceedsMaxLength()
        print("test_EdgeTrajectoryExceedsMaxLength passed\n")

        print("Running test_EdgeIdenticalEntryAndTarget")
        self.test_EdgeIdenticalEntryAndTarget()
        print("test_EdgeIdenticalEntryAndTarget passed\n")

    def test_FinalPathPlanner1(self):
        self.delayDisplay("Starting test_FinalPathPlanner1")

        import SampleData
        registerSampleData()
        SampleData.SampleDataLogic().downloadSample('FinalPathPlanner1')

        inputVolume = slicer.util.getNode('FinalPathPlanner1')
        if inputVolume is None:
            raise RuntimeError("Sample data node 'FinalPathPlanner1' not found")

        self.delayDisplay("Loaded test data set")

        inputScalarRange = inputVolume.GetImageData().GetScalarRange()
        self.assertEqual(inputScalarRange[0], 0)
        self.assertEqual(inputScalarRange[1], 695)

        outputVolume = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")
        threshold = 100

        logic = FinalPathPlannerLogic()

        logic.process(inputVolume, outputVolume, threshold, True)
        outputScalarRange = outputVolume.GetImageData().GetScalarRange()
        self.assertEqual(outputScalarRange[0], inputScalarRange[0])
        self.assertEqual(outputScalarRange[1], threshold)

        logic.process(inputVolume, outputVolume, threshold, False)
        outputScalarRange = outputVolume.GetImageData().GetScalarRange()
        self.assertEqual(outputScalarRange[0], inputScalarRange[0])
        self.assertEqual(outputScalarRange[1], inputScalarRange[1])

        self.delayDisplay("test_FinalPathPlanner1 passed")

    def test_PositiveTrajectorySelection(self):
        self.delayDisplay("Positive test: highest scoring trajectory selected")

        import SampleData
        registerSampleData()
        SampleData.SampleDataLogic().downloadSample('FinalPathPlanner1')

        entryFiducials = slicer.util.getNode('entries')
        targetFiducials = slicer.util.getNode('targets')
        criticalNode = slicer.util.getNode('brainstem')  # Or whichever critical structure is loaded
        targetVolumeNode = slicer.util.getNode('r_hippo')
        maxLength = 150
        minSafeDistance = 10  # 10 mm safety margin

        if None in [entryFiducials, targetFiducials, criticalNode, targetVolumeNode]:
            raise RuntimeError("One or more required nodes not found in scene")

        logic = FinalPathPlannerLogic()
        validTrajectories, rejected = logic.findAllValidTrajectories(
            entryFiducials, targetFiducials, criticalNode, targetVolumeNode, maxLength,
            minSafeDistance=minSafeDistance)

        print(f"Valid trajectories: {len(validTrajectories)}, Rejected: {rejected}")

        if len(validTrajectories) == 0:
            print("No valid trajectories found.")
            return

        bestTrajectory = max(validTrajectories, key=lambda x: x[2])

        print(f"Best trajectory score: {bestTrajectory[2]:.3f}")
        print(f"Entry point: {bestTrajectory[0]}")
        print(f"Target point: {bestTrajectory[1]}")

        logic.plotTrajectory(bestTrajectory)
        print("Best trajectory plotted.")

    def test_NegativeEmptyInputs(self):
        self.delayDisplay("Negative test: handle empty inputs")

        logic = FinalPathPlannerLogic()

        try:
            logic.findAllValidTrajectories(None, None, None, None, 150, 5.0)
        except Exception as e:
            print(f"Caught expected exception: {e}")
        else:
            self.fail("Expected exception not raised")

    def test_EdgeMaxLength(self):
        self.delayDisplay("Edge test: trajectories at maximum allowed length")

        import SampleData
        registerSampleData()
        SampleData.SampleDataLogic().downloadSample('FinalPathPlanner1')

        entryFiducials = slicer.util.getNode('entries')
        targetFiducials = slicer.util.getNode('targets')
        criticalNode = slicer.util.getNode('brainstem')
        targetVolumeNode = slicer.util.getNode('r_hippo')

        if None in [entryFiducials, targetFiducials, criticalNode, targetVolumeNode]:
            raise RuntimeError("One or more required nodes not found")

        maxLength = 150  # Define maxLength inside method

        logic = FinalPathPlannerLogic()

        validTrajectories, rejected = logic.findAllValidTrajectories(
            entryFiducials, targetFiducials, criticalNode, targetVolumeNode, maxLength,
            minSafeDistance=10)

        for traj in validTrajectories:
            dist = vtk.vtkMath.Distance2BetweenPoints(traj[0], traj[1]) ** 0.5
            self.assertLessEqual(dist, maxLength)  # Check against maxLength

        self.delayDisplay(f"Found {len(validTrajectories)} trajectories within max length {maxLength}")

    def test_NegativeCollisionWithCriticalStructure(self):
        print("Running test_NegativeCollisionWithCriticalStructure")
        logic = FinalPathPlannerLogic()

        entryFiducials = slicer.util.getNode('entries')
        targetFiducials = slicer.util.getNode('targets')
        criticalNode = slicer.util.getNode('brainstem')
        targetVolumeNode = slicer.util.getNode('r_hippo')

        validTrajectories, rejected = logic.findAllValidTrajectories(
            entryFiducials, targetFiducials, criticalNode, targetVolumeNode, 150,
            minSafeDistance=10)

        for entry, target, score in validTrajectories:
            assert not logic.trajectoryCollides(entry, target, criticalNode), \
                "Trajectory collides with critical structure but was not rejected"

        print("test_NegativeCollisionWithCriticalStructure passed")

    def test_EdgeTrajectoryExceedsMaxLength(self):
        print("Running test_EdgeTrajectoryExceedsMaxLength")
        logic = FinalPathPlannerLogic()

        entryFiducials = slicer.util.getNode('entries')
        targetFiducials = slicer.util.getNode('targets')
        criticalNode = slicer.util.getNode('brainstem')
        targetVolumeNode = slicer.util.getNode('r_hippo')

        maxLength = 1

        validTrajectories, rejected = logic.findAllValidTrajectories(
            entryFiducials, targetFiducials, criticalNode, targetVolumeNode, maxLength,
            minSafeDistance=10)

        assert len(validTrajectories) == 0, "Trajectories found but should be rejected due to length"
        print("test_EdgeTrajectoryExceedsMaxLength passed")

    def test_EdgeIdenticalEntryAndTarget(self):
        print("Running test_EdgeIdenticalEntryAndTarget")
        logic = FinalPathPlannerLogic()

        entryFiducials = slicer.vtkMRMLMarkupsFiducialNode()
        targetFiducials = slicer.vtkMRMLMarkupsFiducialNode()

        point = [10.0, 10.0, 10.0]
        entryFiducials.AddControlPoint(*point)
        targetFiducials.AddControlPoint(*point)

        criticalNode = slicer.util.getNode('brainstem')
        targetVolumeNode = slicer.util.getNode('r_hippo')

        validTrajectories, rejected = logic.findAllValidTrajectories(
            entryFiducials, targetFiducials, criticalNode, targetVolumeNode, 150,
            minSafeDistance=3.0)

        assert len(validTrajectories) == 0, "Trajectory with identical entry and target should be rejected"
        print("test_EdgeIdenticalEntryAndTarget passed")


def runAllTests():
    tester = FinalPathPlannerTest()
    tester.setUp = lambda: None  # optional: prevent scene clearing
    tester.runTest()

# Uncomment to run tests automatically on module reload:
# runAllTests()
