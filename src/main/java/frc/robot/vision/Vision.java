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
import java.util.Arrays;
import java.util.List;
import java.util.NoSuchElementException;

public class Vision extends SubsystemBase {
  public enum CameraSide {
    Left,
    Right
  }

  public final double kTagAmbiguityThreshold = 1;

  protected Matrix<N3, N1> curStdDevsLml3;
  protected Matrix<N3, N1> curStdDevsLml2Right;
  protected Matrix<N3, N1> curStdDevsLml2Left;

  // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  // -0,06985, 0, 0,2794, 0,0,0
  // Transform3d robotToCamLml3 =
  //     new Transform3d(
  //         new Translation3d(
  //             Units.inchesToMeters(-2.75), Units.inchesToMeters(0), Units.inchesToMeters(11.5)),
  //         new Rotation3d(0, 0, 0));
  // 0,3048, -0,282575, -0,2286, 0, -15, 19.95
  Transform3d robotToCamLml2Right =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(12.25), Units.inchesToMeters(-11.125), Units.inchesToMeters(-9)),
          new Rotation3d(0, -15, 19.95));
  // 0,3048, 0,282575, -0,2286, 0, -15, -19.96
  Transform3d robotToCamLml2Left =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(12.25), Units.inchesToMeters(11.125), Units.inchesToMeters(-9)),
          new Rotation3d(0, -15, -19.96));

  private int m_lockID = 0;
  private List<Integer> m_allowedReefPegTag = new ArrayList<Integer>();
  private final double robotHalfLength = Units.inchesToMeters(18);
  private final double distTagToPeg = Units.inchesToMeters(6.25);
  private final double desiredDistFromTag = 1;
  private Translation2d minimumTranslationProcessor = new Translation2d();
  private Translation2d maximumTranslationProcessor = new Translation2d();
  private Pose2d processorAlignPosition = new Pose2d();
  // LimelightHelpers.PoseEstimate lml3Measurement;
  LimelightHelpers.PoseEstimate lml2RMeasurement;
  LimelightHelpers.PoseEstimate lml2LMeasurement;

  // private String limelight3 = "limelight";
  private String limelight2R = "limelight-right";
  private String limelight2L = "limelight-left";

  Alliance currentAlliance;

  private enum direction {
    left,
    right,
    back
  }

  /** Creates a new Odometry. */
  public Vision() {

    try {
      currentAlliance = DriverStation.getAlliance().get();
      if (currentAlliance == Alliance.Blue) {
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
      } else if (currentAlliance == Alliance.Red) {
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
      currentAlliance = Alliance.Blue;
      m_allowedReefPegTag.clear();
    }

    // LimelightHelpers.setCameraPose_RobotSpace(
    //     limelight3,
    //     robotToCamLml3.getX(),
    //     robotToCamLml3.getY(),
    //     robotToCamLml3.getZ(),
    //     robotToCamLml3.getRotation().getX(),
    //     robotToCamLml3.getRotation().getY(),
    //     robotToCamLml3.getRotation().getZ());
    LimelightHelpers.setCameraPose_RobotSpace(
        limelight2L,
        robotToCamLml2Left.getX(),
        robotToCamLml2Left.getY(),
        robotToCamLml2Left.getZ(),
        robotToCamLml2Left.getRotation().getX(),
        robotToCamLml2Left.getRotation().getY(),
        robotToCamLml2Left.getRotation().getZ());
    LimelightHelpers.setCameraPose_RobotSpace(
        limelight2R,
        robotToCamLml2Right.getX(),
        robotToCamLml2Right.getY(),
        robotToCamLml2Right.getZ(),
        robotToCamLml2Right.getRotation().getX(),
        robotToCamLml2Right.getRotation().getY(),
        robotToCamLml2Right.getRotation().getZ());
  }

  public boolean limelight2LeftActive() {
    // return cameraLml2Left.isConnected();
    return true;
  }

  public boolean limelight2RightActive() {
    // return cameraLml2Right.isConnected();
    return true;
  }

  public boolean limelight3Active() {
    // return cameraLml3.isConnected();
    return true;
  }

  private LimelightHelpers.PoseEstimate filterAmbiguousMeasurement(
      LimelightHelpers.PoseEstimate pose) {
    if (pose == null) return null;

    // compute the average ambiguity of the pose by looping through all the ambiguite of the
    // rawfiducial
    // poses and averaging them
    double avgAmbiguity = 0;
    for (var fiducial : pose.rawFiducials) {
      avgAmbiguity += fiducial.ambiguity;
    }

    avgAmbiguity /= pose.rawFiducials.length;

    if (avgAmbiguity > kTagAmbiguityThreshold) {
      // if the average ambiguity is greater than 0.2, then we will return the pose as null
      return null;
    }

    return pose;
  }

  private boolean allowedTarget(LimelightHelpers.RawFiducial target) {
    return m_allowedReefPegTag.contains(target.id);
  }

  private void selectLockID() {
    ArrayList<LimelightHelpers.RawFiducial> allTags = new ArrayList<LimelightHelpers.RawFiducial>();
    // if (lml3Measurement != null) allTags.addAll(Arrays.asList(lml3Measurement.rawFiducials));
    if (lml2LMeasurement != null) allTags.addAll(Arrays.asList(lml2LMeasurement.rawFiducials));
    if (lml2RMeasurement != null) allTags.addAll(Arrays.asList(lml2RMeasurement.rawFiducials));

    // initialize variables to large or impossible values
    double targetAmbibuity = 10.0;
    double targetDistance = 10.0;
    // PhotonTrackedTarget bestTarget = null;
    int bestID = 0;

    // iterate through all results and find
    // the best target that is allowed
    for (var tag : allTags) {
      if ((allowedTarget(tag))
          && (tag.distToCamera < targetDistance)
          && (tag.ambiguity < targetAmbibuity)) {
        targetDistance = tag.distToCamera;
        targetAmbibuity = tag.ambiguity;
        bestID = tag.id;
      }
    }
    m_lockID = bestID;
  }

  public void doPeriodic(double robotYaw) {
    // First, tell Limelight your robot's current orientation
    // LimelightHelpers.SetRobotOrientation(limelight3, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.SetRobotOrientation(limelight2L, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.SetRobotOrientation(limelight2R, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    // if (currentAlliance == Alliance.Blue) {
    // lml3Measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight3);
    lml2LMeasurement =
        filterAmbiguousMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight2L));
    lml2RMeasurement =
        filterAmbiguousMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight2R));
    // } else {
    //   lml3Measurement = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
    //   // lml2LMeasurement =
    //   //     filterAmbiguousMeasurement(
    //   //         LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-left"));
    //   // lml2RMeasurement =
    //   //     filterAmbiguousMeasurement(
    //   //         LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-right"));
    // }

    selectLockID();
  }

  // public LimelightHelpers.PoseEstimate getEstimatedGlobalPoseLml3() {
  //   return lml3Measurement;
  // }

  public LimelightHelpers.PoseEstimate getEstimatedGlobalPoseLml2Right() {
    return lml2RMeasurement;
  }

  public LimelightHelpers.PoseEstimate getEstimatedGlobalPoseLml2Left() {
    return lml2LMeasurement;
  }

  public double getLockID() {
    return m_lockID;
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

  public void getEstimatedStdDevs(LimelightHelpers.PoseEstimate estimate) {

    var b = 0.5;
    var xDevs = 4.0;
    var yDevs = 4.0;
    var rotDevs = 9999999;
    var dist = estimate.avgTagDist;
    var stdDevs = VecBuilder.fill(xDevs, yDevs, rotDevs);

    xDevs = xDevs / dist * b;
    if (estimate == lml2LMeasurement) {

    } else if (estimate == lml2RMeasurement) {

      // } else if (estimate == lml3Measurement) {

      // } else {

    }
  }
}
