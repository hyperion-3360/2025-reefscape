// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Conversions;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  public enum CameraSide {
    Left,
    Right
  }

  protected final PhotonCamera cameraLml3;
  // protected final PhotonCamera cameraLml2Right;
  // protected final PhotonCamera cameraLml2Left;

  protected final PhotonPoseEstimator photonEstimatorLml3;
  // protected final PhotonPoseEstimator photonEstimatorLml2Right;
  // protected final PhotonPoseEstimator photonEstimatorLml2Left;

  protected Matrix<N3, N1> curStdDevsLml3;
  protected Matrix<N3, N1> curStdDevsLml2Right;
  protected Matrix<N3, N1> curStdDevsLml2Left;

  private Matrix<N3, N1> singleTagStdDevsLml3 = VecBuilder.fill(4, 4, 8);
  private Matrix<N3, N1> multiTagStdDevsLml3 = VecBuilder.fill(3, 3, 6);

  private Matrix<N3, N1> singleTagStdDevsLml2 = VecBuilder.fill(4, 4, 8);
  private Matrix<N3, N1> multiTagStdDevsLml2 = VecBuilder.fill(3, 3, 6);
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

  private int m_lockID = 0;
  private List<Integer> m_allowedReefPegTag = new ArrayList<Integer>();
  private Optional<EstimatedRobotPose> visionEstLml3;
  private Optional<EstimatedRobotPose> visionEstLml2Left;
  private Optional<EstimatedRobotPose> visionEstLml2Right;
  private final double robotHalfLength = Units.inchesToMeters(18);
  private final double distTagToPeg = Units.inchesToMeters(6.25);
  private final double desiredDistFromTag = 1;
  private Translation2d minimumTranslationProcessor = new Translation2d();
  private Translation2d maximumTranslationProcessor = new Translation2d();
  private Pose2d processorAlignPosition = new Pose2d();

  private enum direction {
    left,
    right,
    back
  }

  /** Creates a new Odometry. */
  public Vision() {

    cameraLml3 = new PhotonCamera("lml3");
    cameraLml2Right = new PhotonCamera("lml2R");
    cameraLml2Left = new PhotonCamera("lml2L");
    // MULTI_TAG_PNP_ON_COPROCESSOR
    photonEstimatorLml3 =
        new PhotonPoseEstimator(Constants.tagLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamLml3);
    // photonEstimatorLml3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonEstimatorLml2Right =
        new PhotonPoseEstimator(
            Constants.tagLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamLml2Right);
    photonEstimatorLml2Left =
        new PhotonPoseEstimator(
            Constants.tagLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamLml2Left);

    try {
      var alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Blue) {
        minimumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(200.0), Units.inchesToMeters(0.0));
        maximumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(300.0), Units.inchesToMeters(100.0));
        processorAlignPosition =
            new Pose2d(
                Units.inchesToMeters(240),
                Units.inchesToMeters(30),
                new Rotation2d(Units.degreesToRadians(-90)));
        m_allowedReefPegTag.clear();
        m_allowedReefPegTag.add(18);
        m_allowedReefPegTag.add(17);
        m_allowedReefPegTag.add(22);
        m_allowedReefPegTag.add(21);
        m_allowedReefPegTag.add(20);
        m_allowedReefPegTag.add(19);
      } else if (alliance == Alliance.Red) {
        minimumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(420.0), Units.inchesToMeters(217.0));
        maximumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(490.0), Units.inchesToMeters(500));
        processorAlignPosition =
            new Pose2d(
                Units.inchesToMeters(455.15),
                Units.inchesToMeters(299.455),
                new Rotation2d(Units.degreesToRadians(90)));
        m_allowedReefPegTag.clear();
        m_allowedReefPegTag.add(7);
        m_allowedReefPegTag.add(8);
        m_allowedReefPegTag.add(9);
        m_allowedReefPegTag.add(10);
        m_allowedReefPegTag.add(11);
        m_allowedReefPegTag.add(6);
      } else {
        m_allowedReefPegTag.clear();
      }

    } catch (NoSuchElementException e) {
      m_allowedReefPegTag.clear();
    }
  }

  // public boolean limelight2LeftActive() {
  //   return cameraLml2Left.isConnected();
  // }

  // public boolean limelight2RightActive() {
  //   return cameraLml2Right.isConnected();
  // }

  public boolean limelight3Active() {
    return cameraLml3.isConnected();
  }

  private boolean allowedTarget(PhotonTrackedTarget target) {
    return m_allowedReefPegTag.contains(target.getFiducialId());
  }

  private boolean isGoodResult(PhotonPipelineResult result) {
    boolean rc = true;
    do {
      if (!result.hasTargets()) {
        rc = false;
        break;
      }

      for (var target : result.getTargets()) {
        if (target.getPoseAmbiguity() > 0.2) {
          rc = false;
          break;
        }
      }

    } while (false);

    return rc;
  }

  public void doPeriodic() {
    visionEstLml3 = Optional.empty();

    // for a change in target (latest result), estimation.update with latest
    // update estimation standard deviations with new estimation and new target
    var unreadResultsLml3 = cameraLml3.getAllUnreadResults();

    for (var changelml3 : unreadResultsLml3) {
      if (isGoodResult(changelml3)) {
        visionEstLml3 = photonEstimatorLml3.update(changelml3);
        updateEstimationStdDevsLml3(visionEstLml3, changelml3.getTargets());
      }
    }
    visionEstLml2Right = Optional.empty();

    // for a change in target (latest result), estimation.update with latest
    // update estimation standard deviations with new estimation and new target
    var unreadResultsLml2Right = cameraLml2Right.getAllUnreadResults();

    for (var changelml2R : unreadResultsLml2Right) {
      if (isGoodResult(changelml2R)) {

        visionEstLml2Right = photonEstimatorLml2Right.update(changelml2R);
        updateEstimationStdDevsLml2(visionEstLml2Right, changelml2R.getTargets(), CameraSide.Right);
      }
    }

    visionEstLml2Left = Optional.empty();

    // for a change in target (latest result), estimation.update with latest
    // update estimation standard deviations with new estimation and new target
    var unreadResultsLml2Left = cameraLml2Left.getAllUnreadResults();

    for (var changelml2L : unreadResultsLml2Left) {
      if (isGoodResult(changelml2L)) {
        visionEstLml2Left = photonEstimatorLml2Left.update(changelml2L);
        updateEstimationStdDevsLml2(visionEstLml2Left, changelml2L.getTargets(), CameraSide.Left);
      }
    }

    // setLockTargetfsa
    // get all results from all cameras
    var allResults =
        Stream.of(
                unreadResultsLml3.stream(),
                unreadResultsLml2Right.stream(),
                unreadResultsLml2Left.stream())
            .flatMap(i -> i)
            .collect(Collectors.toList());

    // initialize variables to large or impossible values
    double targetAmbibuity = 10.0;
    double targetDistance = 10.0;
    // PhotonTrackedTarget bestTarget = null;
    int bestID = 0;

    // iterate through all results and find
    // the best target that is allowed
    for (var result : allResults) {
      if (result.hasTargets()) {
        var currentTarget = result.getBestTarget();
        var distance = currentTarget.getBestCameraToTarget().getTranslation().getNorm();

        // if the target is allowed, closer than the current target
        if (allowedTarget(currentTarget)
            && (distance < targetDistance)
            && (currentTarget.getPoseAmbiguity() < targetAmbibuity)) {
          targetDistance = distance;
          targetAmbibuity = currentTarget.getPoseAmbiguity();
          bestID = currentTarget.getFiducialId();
          // bestTarget = currentTarget;
        }
      }
    }
    // if a target is found, set the lockID and trackedTarget
    // if (bestID != 0) {
    m_lockID = bestID;
    // }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLml3() {
    return visionEstLml3;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLml2Right() {
    return visionEstLml2Right;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLml2Left() {
    return visionEstLml2Left;
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
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets,
      CameraSide side) {

    if (estimatedPose.isEmpty()) {
      if (side == CameraSide.Left) {
        curStdDevsLml2Left = singleTagStdDevsLml2;
      } else {
        curStdDevsLml2Right = singleTagStdDevsLml2;
      }
    } else {

  //     // pose preswnt, start running heuristic
  //     var estStdDevs = singleTagStdDevsLml2;
  //     int numTags = 0;
  //     double avgDist = 0;

  //     // precalc (how mny tags, avg dist metric)
  //     for (var tgt : targets) {
  //       var tagPose = photonEstimatorLml3.getFieldTags().getTagPose(tgt.getFiducialId());
  //       if (tagPose.isEmpty()) continue;
  //       numTags++;
  //       avgDist +=
  //           tagPose
  //               .get()
  //               .toPose2d()
  //               .getTranslation()
  //               .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
  //     }

      if (numTags == 0) {
        // no visiblw tags default to single tag
        if (side == CameraSide.Left) {
          curStdDevsLml2Left = singleTagStdDevsLml2;
        } else {
          curStdDevsLml2Right = singleTagStdDevsLml2;
        }
      } else {
        // more tags, run full heuristic
        avgDist /= numTags;

  //       // decrase std devs if multiple visible
  //       if (numTags > 1) {
  //         estStdDevs = multiTagStdDevsLml2;
  //       }

  //       // increase std devs based on "avg" dist
  //       if (numTags == 1 && avgDist > 4) {
  //         estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  //       } else {
  //         estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
  //       }

        if (side == CameraSide.Left) {
          curStdDevsLml2Left = estStdDevs;
        } else {
          curStdDevsLml2Right = estStdDevs;
        }
      }
    }
  }

  public Matrix<N3, N1> getEstimationStdDevsLml3() {
    return curStdDevsLml3;
  }

  public Matrix<N3, N1> getEstimationStdDevsLml2Left() {
    return curStdDevsLml2Left;
  }

  public Matrix<N3, N1> getEstimationStdDevsLml2Right() {
    return curStdDevsLml2Right;
  }

  // #region lockID

  // public void periodic() {
  // setLockTarget(); //done in doPeriodic()
  // isInBoundsForProcessor(); //TBD
  // System.out.println(lockID);
  // }

  public double getLockID() {
    return m_lockID;
  }

  private double GetTagYaw() {
    if (m_lockID != 0) {
      return Constants.tagLayout.getTagPose(m_lockID).get().getRotation().getZ();
    }
    return 0.0;
  }

  private Translation2d GetTagTranslation() {

    if (m_lockID != 0) {

      var x = Constants.tagLayout.getTagPose(m_lockID).get().getX();
      var y = Constants.tagLayout.getTagPose(m_lockID).get().getY();

      return new Translation2d(x, y);
    }

    return new Translation2d();
  }

  public boolean isInBoundsForProcessor(Pose2d currentPose) {
    if (m_lockID == 0) {
      if (currentPose.getX() < maximumTranslationProcessor.getX()
          && currentPose.getX() > minimumTranslationProcessor.getX()
          && currentPose.getY() < maximumTranslationProcessor.getY()
          && currentPose.getY() > minimumTranslationProcessor.getY()) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  private Pose2d computeNewPoseFromTag(int tagID, direction dir) {
    // 0 deg in front of the robot
    var tagPose = Conversions.Pose3dToPose2d(Constants.tagLayout.getTagPose(tagID).get());
    tagPose =
        new Pose2d(
            tagPose.getX(),
            tagPose.getY(),
            tagPose.getRotation().rotateBy(new Rotation2d(Math.toRadians(180))));

    double translationX = 0.0;
    double translationY = 0.0;
    switch (dir) {
      case left:
        translationX -= robotHalfLength;
        translationY += distTagToPeg;
        break;
      case right:
        translationX -= robotHalfLength;
        translationY -= distTagToPeg;
        break;
      case back:
        translationX -= desiredDistFromTag;
        break;
    }
    var translation =
        tagPose
            .getTranslation()
            .plus(new Translation2d(translationX, translationY).rotateBy(tagPose.getRotation()));
    return new Pose2d(translation, tagPose.getRotation());
  }

  public Pose2d getDesiredPoseRight() {
    if (m_lockID == 0) {
      return Pose2d.kZero;
    }
    return computeNewPoseFromTag(m_lockID, direction.right);
  }

  public Pose2d getDesiredPoseLeft() {
    if (m_lockID == 0) {
      return Pose2d.kZero;
    }
    return computeNewPoseFromTag(m_lockID, direction.left);
  }

  public Pose2d getDesiredPoseAlgae(Pose2d currentPose) {
    if (m_lockID != 0) {
      return computeNewPoseFromTag(m_lockID, direction.back);
    } else if (m_lockID == 0 && isInBoundsForProcessor(currentPose)) {
      return processorAlignPosition;
    } else {
      return Pose2d.kZero;
    }
  }
  // #endregion
}
