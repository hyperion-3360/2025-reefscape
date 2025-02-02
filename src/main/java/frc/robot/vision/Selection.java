package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;

public class Selection extends Vision {

  Swerve swerve;
  List<Integer> reefPegTag = new ArrayList<Integer>();
  int lockID = -1;
  PhotonTrackedTarget trackedTarget;
  Translation2d desiredTranslation;
  // field units are in meters, so we want to be approx 1 meter from target
  double desiredDistFromTag = 1;
  double orientationMultipleY = 0;

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

    SmartDashboard.putBoolean("has vision", hasVision());
    // camera.getAllUnreadResults();
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
      trackedTarget = change.getBestTarget();
      lockID = trackedTarget.fiducialId;
      System.out.println(lockID);
    }
  }

  // will probably refactor
  public Command Align() {
    if (hasVision()) {

      desiredTranslation =
          new Translation2d(
              // the sin and cos from Math take radians
              GetTagTranslation().getX()
                  + (Math.cos(Math.toRadians(GetYaw())) * desiredDistFromTag),
              GetTagTranslation().getY()
                  + (Math.sin(Math.toRadians(GetYaw())) * desiredDistFromTag));
      // not sure abt the 360-yaw
      // return Commands.run(() -> swerve.drive(desiredTranslation, 360 - GetYaw(), true, false));
      return Commands.runOnce(
          // returns 0.0 1.0
          // () -> System.out.println(desiredTranslation.getX() + " " + desiredTranslation.getY()));
          () -> System.out.println(GetTagTranslation().getX() + " " + GetTagTranslation().getY()));

    }
    return null;
  }

  public boolean hasVision() {
    return !super.camera.getAllUnreadResults().isEmpty();
  }

  private double GetYaw() {
    if (trackedTarget != null) {
      return trackedTarget.yaw;
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
