// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  protected final PhotonCamera cameraLml3;
  protected final PhotonCamera cameraLml2;

  protected final PhotonPoseEstimator photonEstimatorLml3;
  protected final PhotonPoseEstimator photonEstimatorLml2;
  protected List<PhotonPipelineResult> unreadResults;

  protected Matrix<N3, N1> curStdDevs;

  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  private Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(3.8, 3.8, 7.6);
  private Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(3, 3, 6);

  AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  Transform3d robotToCamLml3 =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-2.75), Units.inchesToMeters(-1), 0.0),
          new Rotation3d(0, 0, 0));
  Transform3d robotToCamLml2 =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-4.75), 0.0, 0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(180)));

  /** Creates a new Odometry. */
  public Vision() {

    cameraLml3 = new PhotonCamera("lml3");
    cameraLml2 = new PhotonCamera("lml2");
    photonEstimatorLml3 =
        new PhotonPoseEstimator(
            tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamLml3);
    photonEstimatorLml3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonEstimatorLml2 =
        new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamLml2);
    // photonEstimatorLml2.setMultiTagFallbackStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  }

  public boolean limelight2Active() {
    return cameraLml2.isConnected();
  }

  public boolean limelight3Active() {
    return cameraLml3.isConnected();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    // for a change in target (latest result), estimation.update with latest
    // update estimation standard deviations with new estimation and new target
    unreadResults = cameraLml3.getAllUnreadResults();

    for (var changelml3 : unreadResults) {
      if (changelml3.hasTargets()) {

        visionEst = photonEstimatorLml3.update(changelml3);
        updateEstimationStdDevs(visionEst, changelml3.getTargets());
      }
    }

    if (unreadResults.isEmpty()) {
      for (var changelml2 : cameraLml2.getAllUnreadResults()) {
        if (changelml2.hasTargets()) {
          if (Math.hypot(
                  changelml2.getBestTarget().getBestCameraToTarget().getX(),
                  changelml2.getBestTarget().getBestCameraToTarget().getY())
              > 2) {
            if (changelml2.hasTargets() && changelml2.getBestTarget().poseAmbiguity < 0.05) {
              visionEst = photonEstimatorLml2.update(changelml2);
              updateEstimationStdDevs(visionEst, changelml2.getTargets());
            }
          }
        }
      }
    } else {
      cameraLml2.getAllUnreadResults();
    }
    return visionEst;
  }

  protected void updateEstimationStdDevs(
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
        var tagPose = photonEstimatorLml3.getFieldTags().getTagPose(tgt.getFiducialId());
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
        // no visiblw tags default to single tag
        curStdDevs = singleTagStdDevs;
      } else {
        // more tags, run full heuristic
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
}
