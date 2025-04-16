// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.desiredHeight;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  private final int kNumAprilTagReefScape = 22;

  // tags from 1.. N + 0 for special case where no tag is detected
  private double m_AprilTagsScore[] = new double[kNumAprilTagReefScape + 1];
  private final double kPrimaryTagDistanceFactor = 2.0;
  private final double kPrimaryTagAmbiguityFactor = 2.0;
  private final double kSecondaryTagDistanceFactor = 0.5;
  private final double kSecondaryTagAmbiguityFactor = 0.5;
  private final double kNoTagFoundValue = 0.5;
  // initial score completely vanishes after 25 x 20ms = 0.5s
  private final double kDecimationFator = 1.0 / 1.15;
  private final double kMinimumValidScore = 0.01;

  private VisionCamera m_cameras[] = new VisionCamera[3];
  private boolean m_tagFound = true;

  desiredHeight currentAlgaeHeight = desiredHeight.LOW;

  Transform3d m_LL2_Right =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(12.25),
              Units.inchesToMeters(-11.125),
              Units.inchesToMeters(7.25)),
          new Rotation3d(
              Units.degreesToRadians(0),
              Units.degreesToRadians(-15),
              Units.degreesToRadians(19.7)));

  Transform3d m_LL2_Left =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(12.25),
              Units.inchesToMeters(11.125),
              Units.inchesToMeters(7.25)),
          new Rotation3d(
              Units.degreesToRadians(0),
              Units.degreesToRadians(-15),
              Units.degreesToRadians(-19.7)));

  Transform3d m_LL3 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-2.75), Units.inchesToMeters(0), Units.inchesToMeters(34)),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(+15), 0.0));

  private int m_lockID = 0;
  private int m_printCounter = 0;
  private List<Integer> m_allowedReefPegTag = new ArrayList<Integer>();

  private final double krobotHalfLength = Units.inchesToMeters(18);
  private final double kdistTagToPeg = Units.inchesToMeters(6.25);
  private final double kdesiredDistFromTag = 1;

  private Translation2d m_minimumTranslationProcessor = new Translation2d();
  private Translation2d m_maximumTranslationProcessor = new Translation2d();
  private Pose2d m_processorAlignPosition = new Pose2d();
  // we want to be close to the reef to intake an algae but we don't want to slam into the reef
  private double desiredCloseUpDistFromTag = krobotHalfLength;

  private enum direction {
    left,
    right,
    back,
    close
  }

  /** Creates a new Odometry. */
  public Vision() {

    m_cameras[0] =
        new VisionCamera("lml3", m_LL3, kPrimaryTagDistanceFactor, kPrimaryTagAmbiguityFactor);
    m_cameras[1] =
        new VisionCamera(
            "lml2R", m_LL2_Right, kSecondaryTagDistanceFactor, kSecondaryTagAmbiguityFactor);
    m_cameras[2] =
        new VisionCamera(
            "lml2L", m_LL2_Left, kSecondaryTagDistanceFactor, kSecondaryTagAmbiguityFactor);

    try {
      var alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Blue) {
        m_minimumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(200.0), Units.inchesToMeters(0.0));
        m_maximumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(300.0), Units.inchesToMeters(100.0));
        m_processorAlignPosition =
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
        m_minimumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(420.0), Units.inchesToMeters(217.0));
        m_maximumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(490.0), Units.inchesToMeters(500));
        m_processorAlignPosition =
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

  private boolean allowedTarget(PhotonTrackedTarget target) {
    return m_allowedReefPegTag.contains(target.getFiducialId());
  }

  public void doPeriodic() {
    //    visionEstLml3 = Optional.empty();

    for (var camera : m_cameras) {
      camera.updateEstimatedPose();
    }

    m_tagFound = false;

    /*
     * loop through all peg valid peg values and time decimate the score
     */
    for (var tag : m_allowedReefPegTag) m_AprilTagsScore[tag] *= kDecimationFator;
    // don't forget the no tag score!!!
    m_AprilTagsScore[0] *= kDecimationFator;

    /*
     * 1. Loop through all camera poses
     * 2. If the target is allowed, calculate the score
     * 3. If at least one target has been found, reset the no tag score
     */

    for (var camera : m_cameras) {
      camera
          .bestTarget()
          .ifPresentOrElse(
              target -> {
                if (allowedTarget(target)) {
                  m_tagFound = true;
                  var distance = target.getBestCameraToTarget().getTranslation().getNorm();
                  var ambiguity = target.getPoseAmbiguity();
                  var distanceContribution = Math.exp(-0.5 * distance) * camera.distanceFactor;
                  var ambiguityContribution = Math.exp(-4 * ambiguity) * camera.ambiguityFactor;
                  var tagId = target.getFiducialId();
                  m_AprilTagsScore[tagId] += (distanceContribution * ambiguityContribution);
                }
                // there were results from the cameras but the target seen is not allowed
                else {
                  m_AprilTagsScore[0] += kNoTagFoundValue;
                }
              },
              () -> m_AprilTagsScore[0] += kNoTagFoundValue);
    }

    // handle the case where no tag is found
    if (m_tagFound == true) {
      m_AprilTagsScore[0] = 0;
    }

    // find the tag with the highest score
    double maxScore = 0;
    int maxScoreTag = 0;
    //    System.out.println("Tag: 0 Score: " + m_AprilTagsScore[0]);
    for (var tag : m_allowedReefPegTag) {
      if (m_AprilTagsScore[tag] > kMinimumValidScore && m_AprilTagsScore[tag] > maxScore) {
        maxScore = m_AprilTagsScore[tag];
        maxScoreTag = tag;
      }
    }

    //   System.out.println("Max Score: " + maxScore + " Max Score Tag: " + maxScoreTag);

    // if the max score is 0, no tag was found
    if (m_AprilTagsScore[0] > maxScore) {
      m_lockID = 0;
    } else {
      m_lockID = maxScoreTag;
    }
  }

  public int getLockID() {
    return m_lockID;
  }

  public int getLockIDIndex() {
    return m_allowedReefPegTag.indexOf(m_lockID) + 1;
  }

  public boolean isInBoundsForProcessor(Pose2d currentPose) {
    if (m_lockID == 0) {
      if (currentPose.getX() < m_maximumTranslationProcessor.getX()
          && currentPose.getX() > m_minimumTranslationProcessor.getX()
          && currentPose.getY() < m_maximumTranslationProcessor.getY()
          && currentPose.getY() > m_minimumTranslationProcessor.getY()) {
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
        translationX -= krobotHalfLength;
        translationY += kdistTagToPeg;
        break;
      case right:
        translationX -= krobotHalfLength;
        translationY -= kdistTagToPeg;
        break;
      case back:
        translationX -= kdesiredDistFromTag;
        break;
      case close:
        translationX -= desiredCloseUpDistFromTag;
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

  public Pose2d getDesiredPoseAlgae(Supplier<Pose2d> currentPose) {
    if (m_lockID != 0) {
      return computeNewPoseFromTag(m_lockID, direction.back);
    } else if (m_lockID == 0 && isInBoundsForProcessor(currentPose.get())) {
      return m_processorAlignPosition;
    } else {
      return Pose2d.kZero;
    }
  }

  public Pose2d getDesiredCloseUpPoseAlgae() {
    if (m_lockID != 0) {
      return computeNewPoseFromTag(m_lockID, direction.close);
    } else {
      return Pose2d.kZero;
    }
  }

  // #endregion
  public desiredHeight getAlgaeHeight() {

    if (m_allowedReefPegTag.indexOf(m_lockID) == 0
        || m_allowedReefPegTag.indexOf(m_lockID) == 2
        || m_allowedReefPegTag.indexOf(m_lockID) == 4) {
      currentAlgaeHeight = desiredHeight.ALGAEL3;

    } else if (m_allowedReefPegTag.indexOf(m_lockID) == 1
        || m_allowedReefPegTag.indexOf(m_lockID) == 3
        || m_allowedReefPegTag.indexOf(m_lockID) == 5) {
      currentAlgaeHeight = desiredHeight.ALGAEL2;

    } else {
      currentAlgaeHeight = desiredHeight.LOW;
    }

    return currentAlgaeHeight;
  }

  public VisionCamera[] cameras() {
    return m_cameras;
  }
}
