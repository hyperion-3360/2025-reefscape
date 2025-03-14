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
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  protected final PhotonCamera cameraLml3;
  protected final PhotonCamera cameraLml2Right;
  protected final PhotonCamera cameraLml2Left;

  protected final PhotonPoseEstimator photonEstimatorLml3;
  protected final PhotonPoseEstimator photonEstimatorLml2Right;
  protected final PhotonPoseEstimator photonEstimatorLml2Left;

  protected Matrix<N3, N1> curStdDevsLml3;
  protected Matrix<N3, N1> curStdDevsLml2;

  private Matrix<N3, N1> singleTagStdDevsLml3 = VecBuilder.fill(4.0, 4.0, 8.0);
  private Matrix<N3, N1> multiTagStdDevsLml3 = VecBuilder.fill(3, 3, 6);

  private Matrix<N3, N1> singleTagStdDevsLml2 = VecBuilder.fill(4.5, 4.5, 9);
  private Matrix<N3, N1> multiTagStdDevsLml2 = VecBuilder.fill(3.5, 3.5, 7);

  AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  Transform3d robotToCamLml3 =
      new Transform3d(
          new Translation3d(
              // Units.inchesToMeters(33.25)
              Units.inchesToMeters(-2.75), Units.inchesToMeters(0), Units.inchesToMeters(11.42375)),
          new Rotation3d(0, Units.degreesToRadians(10), 0.0));
  Transform3d robotToCamLml2Right =
      new Transform3d(
          // Units.inchesToMeters(7.625)
          new Translation3d(
              Units.inchesToMeters(12.25),
              Units.inchesToMeters(-11.125),
              Units.inchesToMeters(-13.9825)),
          new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(19.7)));
  Transform3d robotToCamLml2Left =
      new Transform3d(
          // Units.inchesToMeters(7.625)
          new Translation3d(
              Units.inchesToMeters(12.25),
              Units.inchesToMeters(11.0),
              Units.inchesToMeters(-13.9825)),
          new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-19.7)));

  /** Creates a new Odometry. */
  public Vision() {

    cameraLml3 = new PhotonCamera("lml3");
    cameraLml2Right = new PhotonCamera("lml2R");
    cameraLml2Left = new PhotonCamera("lml2L");
    photonEstimatorLml3 =
        new PhotonPoseEstimator(
            tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamLml3);
    photonEstimatorLml3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonEstimatorLml2Right =
        new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamLml2Right);
    photonEstimatorLml2Left =
        new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamLml2Left);
  }

  public boolean limelight2LeftActive() {
    return cameraLml2Left.isConnected();
  }

  public boolean limelight2RightActive() {
    return cameraLml2Right.isConnected();
  }

  public boolean limelight3Active() {
    return cameraLml3.isConnected();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLml3() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    // for a change in target (latest result), estimation.update with latest
    // update estimation standard deviations with new estimation and new target
    var unreadResults = cameraLml3.getAllUnreadResults();

    for (var changelml3 : unreadResults) {
      if (changelml3.hasTargets()) {

        visionEst = photonEstimatorLml3.update(changelml3);
        updateEstimationStdDevsLml3(visionEst, changelml3.getTargets());
      }
    }

    return visionEst;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLml2Right() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    // for a change in target (latest result), estimation.update with latest
    // update estimation standard deviations with new estimation and new target
    var unreadResults = cameraLml2Right.getAllUnreadResults();

    for (var changelml2R : unreadResults) {
      if (changelml2R.hasTargets()) {

        visionEst = photonEstimatorLml2Right.update(changelml2R);
        updateEstimationStdDevsLml2(visionEst, changelml2R.getTargets());
      }
    }

    return visionEst;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLml2Left() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    // for a change in target (latest result), estimation.update with latest
    // update estimation standard deviations with new estimation and new target
    var unreadResults = cameraLml2Left.getAllUnreadResults();

    for (var changelml2L : unreadResults) {
      if (changelml2L.hasTargets()) {

        visionEst = photonEstimatorLml2Left.update(changelml2L);
        updateEstimationStdDevsLml2(visionEst, changelml2L.getTargets());
      }
    }

    return visionEst;
  }

  protected void updateEstimationStdDevsLml3(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

    if (estimatedPose.isEmpty()) {
      curStdDevsLml3 = singleTagStdDevsLml3;
    } else {

      // pose preswnt, start running heuristic
      var estStdDevs = singleTagStdDevsLml3;
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
        curStdDevsLml3 = singleTagStdDevsLml3;
      } else {
        // more tags, run full heuristic
        avgDist /= numTags;

        // decrase std devs if multiple visible
        if (numTags > 1) {
          estStdDevs = multiTagStdDevsLml3;
        }

        // increase std devs based on "avg" dist
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        curStdDevsLml3 = estStdDevs;
      }
    }
  }

  protected void updateEstimationStdDevsLml2(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

    if (estimatedPose.isEmpty()) {
      curStdDevsLml2 = singleTagStdDevsLml2;
    } else {

      // pose preswnt, start running heuristic
      var estStdDevs = singleTagStdDevsLml2;
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
        curStdDevsLml2 = singleTagStdDevsLml2;
      } else {
        // more tags, run full heuristic
        avgDist /= numTags;

        // decrase std devs if multiple visible
        if (numTags > 1) {
          estStdDevs = multiTagStdDevsLml2;
        }

        // increase std devs based on "avg" dist
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        curStdDevsLml2 = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getEstimationStdDevsLml3() {
    return curStdDevsLml3;
  }

  public Matrix<N3, N1> getEstimationStdDevsLml2() {
    return curStdDevsLml2;
  }
}
