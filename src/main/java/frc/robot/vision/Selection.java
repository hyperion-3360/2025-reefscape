package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
  Pose2d desiredPoseAlgae = new Pose2d();
  Pose2d origin = new Pose2d();

  double robotHalfLength = Units.inchesToMeters(16.5);
  double distTagToPeg = Units.inchesToMeters(7);

  Pose2d desiredPoseRelativeToCenterRotated = new Pose2d();
  double angleToRotateBy = 0.0;

  Translation2d reefCenter = new Translation2d();

  double desiredRotation = 0.0;

  public enum direction {
    left,
    right
  }

  public Selection(Swerve swerve) {
    this.swerve = swerve;

    var alliance = DriverStation.getAlliance().get();
    if (alliance == Alliance.Blue) {
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
    if (lockID != 0) {
      if (GetYaw() + Math.toRadians(180) > Units.degreesToRadians(180)) {
        desiredRotation = GetYaw() - Math.toRadians(180);
      } else {
        desiredRotation = GetYaw() + Math.toRadians(180);
      }

      angleToRotateBy = reefPegTag.indexOf(lockID) * 60;

      desiredPoseAlgae =
          new Pose2d(
              GetTagTranslation().getX() + (Math.cos(GetYaw()) * desiredDistFromTag),
              GetTagTranslation().getY() + (Math.sin(GetYaw()) * desiredDistFromTag),
              new Rotation2d(desiredRotation));

    } else {
      desiredPoseAlgae = Pose2d.kZero;
    }

    SmartDashboard.putNumber("desiredPose x", desiredPoseRelativeToCenterRotated.getX());
    SmartDashboard.putNumber("desiredPose y", desiredPoseRelativeToCenterRotated.getY());

    // System.out.println(lockID);
  }

  public Pose2d getDesiredposeAlgae() {
    return desiredPoseAlgae;
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

  public void teleopInit() {
    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    var tagPose = layout.getTagPose(18);
    var origin =
        new Pose2d(
            tagPose.get().getX(), tagPose.get().getY(), tagPose.get().getRotation().toRotation2d());

    var a = Units.inchesToMeters(16.5);
    var b = Units.inchesToMeters(7);
    var robotTranslation = new Translation2d(a, b);
    var robotCenter =
        origin.transformBy(new Transform2d(robotTranslation, new Rotation2d(Math.toRadians(-180))));

    var reefCenter = new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));

    var rotatedCenter = robotCenter.rotateAround(reefCenter, new Rotation2d(Math.toRadians(60)));
  }
}
