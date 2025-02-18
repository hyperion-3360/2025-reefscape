package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Selection extends Vision {

  Swerve swerve;
  List<Integer> reefPegTag = new ArrayList<Integer>();
  int lockID = 0;
  PhotonTrackedTarget trackedTarget;

  // field units are in meters, so we want to be approx 1 meter from target
  double desiredDistFromTag = 1;
  Pose2d desiredPoseCenterAlign = new Pose2d();
  Pose2d origin = new Pose2d();

  double robotHalfLength = Units.inchesToMeters(17.5);
  double distTagToPeg = Units.inchesToMeters(7);

  Pose2d desiredPoseRelativeToCenterRotated = new Pose2d();
  double angleToRotateBy = 0.0;

  Translation2d reefCenter = new Translation2d();

  double desiredRotation = 0.0;

  Translation2d minimumTranslationProcessor = new Translation2d();
  Translation2d maximumTranslationProcessor = new Translation2d();
  Pose2d processorAlignPosition = new Pose2d();
  boolean isInBounds = false;

  public enum direction {
    left,
    right
  }

  public Selection(Swerve swerve) {
    this.swerve = swerve;

    var alliance = DriverStation.getAlliance().get();
    if (alliance == Alliance.Blue) {
      minimumTranslationProcessor =
          new Translation2d(Units.inchesToMeters(200.0), Units.inchesToMeters(0.0));
      maximumTranslationProcessor =
          new Translation2d(Units.inchesToMeters(270.0), Units.inchesToMeters(50.0));
      processorAlignPosition =
          new Pose2d(
              Units.inchesToMeters(235.643424), Units.inchesToMeters(17.5), new Rotation2d(-90));

      reefCenter = new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));
      origin =
          new Pose2d(
              tagLayout.getTagPose(18).get().getX(),
              tagLayout.getTagPose(18).get().getY(),
              tagLayout.getTagPose(18).get().getRotation().toRotation2d());
      reefPegTag.clear();
      reefPegTag.add(18);
      reefPegTag.add(17);
      reefPegTag.add(22);
      reefPegTag.add(21);
      reefPegTag.add(20);
      reefPegTag.add(19);

    } else if (alliance == Alliance.Red) {

      minimumTranslationProcessor =
          new Translation2d(Units.inchesToMeters(420.0), Units.inchesToMeters(217.0));
      maximumTranslationProcessor =
          new Translation2d(Units.inchesToMeters(490.0), Units.inchesToMeters(500));
      processorAlignPosition =
          new Pose2d(
              Units.inchesToMeters(455.15), Units.inchesToMeters(299.455), new Rotation2d(90));

      reefCenter = new Translation2d(Units.inchesToMeters(514.14), Units.inchesToMeters(158.5));
      origin =
          new Pose2d(
              tagLayout.getTagPose(7).get().getX(),
              tagLayout.getTagPose(7).get().getY(),
              tagLayout.getTagPose(7).get().getRotation().toRotation2d());
      reefPegTag.clear();
      reefPegTag.add(7);
      reefPegTag.add(8);
      reefPegTag.add(9);
      reefPegTag.add(10);
      reefPegTag.add(11);
      reefPegTag.add(6);

    } else {
      reefPegTag.clear();
    }

    SmartDashboard.putNumber("lock ID", lockID);
  }

  @Override
  public void periodic() {
    setLockTarget();
    SmartDashboard.putNumber("desiredPose x", desiredPoseRelativeToCenterRotated.getX());
    SmartDashboard.putNumber("desiredPose y", desiredPoseRelativeToCenterRotated.getY());
    SmartDashboard.putBoolean("in Bounds For Processor", isInBounds);

    // System.out.println(lockID);
  }

  public boolean isInBoundsForProcessor() {
    if (lockID == 0) {
      if (swerve.getPose().getX() < maximumTranslationProcessor.getX()
          && swerve.getPose().getX() > minimumTranslationProcessor.getX()
          && swerve.getPose().getY() < maximumTranslationProcessor.getY()
          && swerve.getPose().getY() > minimumTranslationProcessor.getY()) {
        isInBounds = true;
      }
    } else {
      isInBounds = false;
    }
    return isInBounds;
  }

  public Pose2d getDesiredposeAlgae() {
    if (lockID != 0) {
      if (GetYaw() + Math.toRadians(180) > Units.degreesToRadians(180)) {
        desiredRotation = GetYaw() - Math.toRadians(180);
      } else {
        desiredRotation = GetYaw() + Math.toRadians(180);
      }

      angleToRotateBy = reefPegTag.indexOf(lockID) * 60;

      desiredPoseCenterAlign =
          new Pose2d(
              GetTagTranslation().getX() + (Math.cos(GetYaw()) * desiredDistFromTag),
              GetTagTranslation().getY() + (Math.sin(GetYaw()) * desiredDistFromTag),
              new Rotation2d(desiredRotation));

    } else if (lockID == 0 && isInBoundsForProcessor()) {
      desiredPoseCenterAlign = processorAlignPosition;
    } else {
      desiredPoseCenterAlign = Pose2d.kZero;
    }
    return desiredPoseCenterAlign;
  }

  public Pose2d getDesiredposeLeft() {
    var robotTranslationLeft = new Translation2d(robotHalfLength, -distTagToPeg);
    var robotPoseRelativeToCenter =
        origin.transformBy(
            new Transform2d(robotTranslationLeft, new Rotation2d(Math.toRadians(-180))));
    desiredPoseRelativeToCenterRotated =
        robotPoseRelativeToCenter.rotateAround(
            reefCenter, new Rotation2d(Math.toRadians(angleToRotateBy)));
    return desiredPoseRelativeToCenterRotated;
  }

  public Pose2d getDesiredposeRight() {
    var robotTranslationRight = new Translation2d(robotHalfLength, distTagToPeg);
    var robotPoseRelativeToCenter =
        origin.transformBy(
            new Transform2d(robotTranslationRight, new Rotation2d(Math.toRadians(-180))));
    desiredPoseRelativeToCenterRotated =
        robotPoseRelativeToCenter.rotateAround(
            reefCenter, new Rotation2d(Math.toRadians(angleToRotateBy)));
    return desiredPoseRelativeToCenterRotated;
  }

  public Command MovePeg(direction direction) {
    // will have to do trigo, but make sure align works first.
    switch (direction) {
      case left:
        break;

      case right:
        break;

      default:
        break;
    }

    return null;
  }

  private void setLockTarget() {

    for (var change : camera.getAllUnreadResults()) {

      if (change.hasTargets()) {
        trackedTarget = change.getBestTarget();
        lockID = trackedTarget.fiducialId;
      } else {
        lockID = 0;
      }
    }
  }

  private double GetYaw() {
    if (trackedTarget != null) {
      return tagLayout.getTagPose(lockID).get().getRotation().getZ();
    }
    return 0.0;
  }

  private Translation2d GetTagTranslation() {

    if (lockID != 0) {

      var x = tagLayout.getTagPose(lockID).get().getX();
      var y = tagLayout.getTagPose(lockID).get().getY();

      return new Translation2d(x, y);
    }

    return new Translation2d();
  }
}
