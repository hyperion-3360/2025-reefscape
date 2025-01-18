// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private Matrix<N3, N1> curStdDevs;

  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  private Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  Transform3d robotToCam =
      new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));

  /** Creates a new Odometry. */
  public Vision() {

    camera = new PhotonCamera("lml3");
    photonEstimator =
        new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(tagLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      cameraSim = new PhotonCameraSim(camera, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(cameraSim, robotToCam);

      cameraSim.enableDrawWireframe(true);
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (var change : camera.getAllUnreadResults()) {
      visionEst = photonEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());

      if (Robot.isSimulation()) {
        visionEst.ifPresentOrElse(
            (est ->
                getSimDebugField()
                    .getObject("VisionEstimation")
                    .setPose(est.estimatedPose.toPose2d())),
            () -> {
              getSimDebugField().getObject("VisionEstimation").setPoses();
            });
      }
    }
    return visionEst;
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

    if (estimatedPose.isEmpty()) {
      curStdDevs = singleTagStdDevs;
    } else {

      // pose preswnt, start running heuristic
      var estStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // precalc (how mny tags, avg dist metric)
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // no visiblw tags dflt to single tag
        curStdDevs = singleTagStdDevs;
      } else {
        // mpre tags, run full heuristic
        avgDist /= numTags;

        // decrase std devs if multiple visible
        if (numTags > 1) {
          estStdDevs = multiTagStdDevs;
        }

        // increase std devs based on "avg" dist
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        curStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // sim

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  // reset pose hist of robot in vision syst sim

  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) {
      visionSim.resetRobotPose(pose);
    }
  }

  // field2d fo visualizing robot and obj on field

  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
