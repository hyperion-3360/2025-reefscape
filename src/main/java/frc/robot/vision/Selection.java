package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;

public class Selection extends Vision {

  Swerve swerve;
  List<Integer> reefPegTag = new ArrayList<Integer>();
  int lockID;
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
    if ( alliance == Alliance.Blue) {
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

  public Command MovePeg(direction direction) {
// will have to do trigo, but make sure align works first.
    switch(direction) {
      case left:
        break;

      case right:
      
        break;

      default:

        break;
      
    }

    return null;
  }

  // will probably refactor
  public Command Align() {
    if (hasVision()) {

      desiredTranslation = new Translation2d(
        // the sin and cos from Math take radians
        GetTranslation().getX() + (Math.sin(Math.toRadians(GetYaw())) * desiredDistFromTag), 
        GetTranslation().getY() + (Math.cos(Math.toRadians(GetYaw())) * desiredDistFromTag));
// not sure abt the 360-yaw
      return Commands.run(() -> swerve.drive(desiredTranslation, 360 - GetYaw(), true, false));
    }
    return null;
  }

  public boolean hasVision() {
    return !super.camera.getAllUnreadResults().isEmpty();
  }

  private int GetId() {

    return GetTarget().getFiducialId();
  }

  private PhotonTrackedTarget GetTarget() {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      for (var change : results) {
        return change.getBestTarget();
      }
    }
    return null;
  }

  private double GetYaw() {

    var targetId = GetId();

    for (var pegId : reefPegTag) {
      if (pegId == targetId) {
        lockID = pegId;
        break; 
      }
    }
    return GetTarget().yaw;
  }

  private Translation2d GetTranslation() {

    var x = tagLayout.getTagPose(lockID).get().getX();
    var y = tagLayout.getTagPose(lockID).get().getY();

    return new Translation2d(x, y);
  }

}
