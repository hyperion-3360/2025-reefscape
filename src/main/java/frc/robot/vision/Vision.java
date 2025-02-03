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

  protected final PhotonCamera camera;
  protected final PhotonPoseEstimator photonEstimator;
  protected Matrix<N3, N1> curStdDevs;

  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  private Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  Transform3d robotToCam =
      new Transform3d(
          new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, Units.degreesToRadians(180)));

  /** Creates a new Odometry. */
  public Vision() {

    camera = new PhotonCamera("lml3");
    photonEstimator =
        new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    // for a change in target (latest result), estimation.update with latest
    // update estimation standard deviations with new estimation and new target
    for (var change : camera.getAllUnreadResults()) {
      visionEst = photonEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());
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
