package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Selection extends Vision {

  Swerve swerve;
  List<Integer> reefPegTag = new ArrayList<Integer>();
  int lockID = -1;
  PhotonTrackedTarget trackedTarget;
  Translation2d desiredcoordinates;
  Pose2d currentcoordinates;
  // field units are in meters, so we want to be approx 1 meter from target
  double desiredDistFromTag = 1;
  double orientationMultipleY = 0;

  double kPtranslation = 3; // stolen from rambrandt to be tuned
  double kProtation = 0.06; // stolen from rambrandt to be tuned
  double rotationFromTag = 1; // stolen from rambrandt to be tuned


  DriveToPose drivetoPose = new DriveToPose(kPtranslation, kProtation, desiredDistFromTag, rotationFromTag);

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
    currentcoordinates = swerve.getPose();

    setLockTarget();
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
        System.out.println(lockID);
      } else {
        lockID = 0;
      }
    }
  }

  // will probably refactor
  public Command Align() {
    if (lockID != 0) {

      desiredcoordinates =
          new Translation2d(
              // the sin and cos from Math take radians
              GetTagTranslation().getX() + (Math.cos(GetYaw()) * desiredDistFromTag),
              GetTagTranslation().getY() + (Math.sin(GetYaw()) * desiredDistFromTag));

              // no fking cliue if this works
      // var speed =
      //     new Translation2d(
      //         (Math.abs(desiredcoordinates.getX()) - currentcoordinates.getX()) * Math.signum(Math.cos(GetYaw())),
      //         (Math.abs(desiredcoordinates.getY()) - currentcoordinates.getY()) * Math.signum(Math.sin(GetYaw())));

      var desiredRotation = new Rotation2d(GetYaw());
      var desiredPose = new Pose2d(desiredcoordinates, desiredRotation);

      drivetoPose.setRobotSpeed(swerve.getSpeeds());
      var desiredSpeed = drivetoPose.getTargetSpeeds(currentcoordinates, desiredPose);
      return Commands.run(() -> swerve.driveRobotRelative(desiredSpeed));
    }
    return Commands.runOnce(() -> System.out.println("null"));
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
