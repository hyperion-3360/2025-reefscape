package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  double distanceTagToPeg = Units.inchesToMeters(4.5);

  Pose2d desiredPoseAlgae = new Pose2d();
  Pose2d desiredPoseLeft = new Pose2d();
  Pose2d desiredPoseRight = new Pose2d();

  double desiredRotation = 0.0;

  public enum direction {
    left,
    right
  }

  public Selection(Swerve swerve) {
    this.swerve = swerve;

    var alliance = DriverStation.getAlliance().get();
    if (alliance == Alliance.Blue) {
      reefPegTag.clear();
      reefPegTag.add(22);
      reefPegTag.add(21);
      reefPegTag.add(20);
      reefPegTag.add(19);
      reefPegTag.add(18);
      reefPegTag.add(17);

    } else if (alliance == Alliance.Red) {
      reefPegTag.clear();
      reefPegTag.add(6);
      reefPegTag.add(7);
      reefPegTag.add(8);
      reefPegTag.add(9);
      reefPegTag.add(10);
      reefPegTag.add(11);

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

      desiredPoseAlgae =
          new Pose2d(
              GetTagTranslation().getX() + (Math.cos(GetYaw()) * desiredDistFromTag),
              GetTagTranslation().getY() + (Math.sin(GetYaw()) * desiredDistFromTag),
              new Rotation2d(desiredRotation));

      desiredPoseLeft = new Pose2d(
        GetTagTranslation().getX() + (Math.cos(GetYaw()) * desiredDistFromTag),
        GetTagTranslation().getY() + (Math.sin(GetYaw()) * desiredDistFromTag),
        new Rotation2d(desiredRotation));

        desiredPoseRight = new Pose2d(
          GetTagTranslation().getX() + (Math.cos(GetYaw()) * desiredDistFromTag),
          GetTagTranslation().getY() + (Math.sin(GetYaw()) * desiredDistFromTag),
          new Rotation2d(desiredRotation));

    } else {
      desiredPoseAlgae = Pose2d.kZero;
    }
    // System.out.println(lockID);
  }

  public Pose2d getDesiredposeAlgae() {
    return desiredPoseAlgae;
  }

  public Pose2d getDesiredposeLeft() {
    return desiredPoseLeft;
  }
  
  public Pose2d getDesiredposeRight() {
    return desiredPoseRight;
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
