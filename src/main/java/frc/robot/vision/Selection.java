package frc.robot.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Selection extends Vision {

  Swerve swerve;
  List<Integer> reefPegTag = new ArrayList<Integer>();
  int lockID = -1;
  PhotonTrackedTarget trackedTarget;
  Translation2d desiredTranslation;
  // field units are in meters, so we want to be approx 1 meter from target
  double desiredDistFromTag = 1;
  double orientationMultipleY = 0;

  Pose2d desiredPose = new Pose2d();

  double kp = 0.03;
  double ki = 0;
  double kd = 0;
  PIDController m_pid = new PIDController(kp, ki, kd);

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
  }

  @Override
  public void periodic() {
    setLockTarget();
    if (lockID != 0) {

      desiredPose =
          new Pose2d(
              GetTagTranslation().getX() + (Math.cos(GetYaw()) * desiredDistFromTag),
              GetTagTranslation().getY() + (Math.sin(GetYaw()) * desiredDistFromTag),
              new Rotation2d(GetYaw() + Math.toRadians(180)));
    } else {
      desiredPose = new Pose2d();
    }
    // System.out.println(currentPose);
    // System.out.println(lockID);
  }

  public Pose2d getDesiredpose() {
    return desiredPose;
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

    if (lockID != -1) {

      var x = tagLayout.getTagPose(lockID).get().getX();
      var y = tagLayout.getTagPose(lockID).get().getY();

      return new Translation2d(x, y);
    }

    return new Translation2d();
  }
}
