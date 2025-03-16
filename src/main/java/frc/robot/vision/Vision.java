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
import frc.robot.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Vision extends SubsystemBase {
  public enum CameraSide {
    Left,
    Right
  }

  protected Matrix<N3, N1> curStdDevsLml3;
  protected Matrix<N3, N1> curStdDevsLml2Right;
  protected Matrix<N3, N1> curStdDevsLml2Left;

  private Matrix<N3, N1> singleTagStdDevsLml3 = VecBuilder.fill(4, 4, 8);
  private Matrix<N3, N1> multiTagStdDevsLml3 = VecBuilder.fill(3, 3, 6);

  private Matrix<N3, N1> singleTagStdDevsLml2 = VecBuilder.fill(4, 4, 8);
  private Matrix<N3, N1> multiTagStdDevsLml2 = VecBuilder.fill(3, 3, 6);

  // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  Transform3d robotToCamLml3 =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-2.75), Units.inchesToMeters(0), 0.0),
          new Rotation3d(0, 0, 0));
  Transform3d robotToCamLml2Right =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(12.25), Units.inchesToMeters(-11.125), 0.0),
          new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(19.95)));
  Transform3d robotToCamLml2Left =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(12.25), Units.inchesToMeters(11.125), 0.0),
          new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-19.96)));

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
  LimelightHelpers.PoseEstimate lml3Measurement;
  LimelightHelpers.PoseEstimate lml2RMeasurement;
  LimelightHelpers.PoseEstimate lml2LMeasurement;
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

  public void doPeriodic(double robotYaw) {
    // First, tell Limelight your robot's current orientation
    LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    if (currentAlliance == Alliance.Blue) {
      lml3Measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("lml3");
      lml2LMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("lml2L");
      lml2RMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("lml2R");
    } else {
      lml3Measurement = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("lml3");
      lml2LMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("lml2L");
      lml2RMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("lml2R");
    }
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
