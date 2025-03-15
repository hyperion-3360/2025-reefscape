package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Selection extends Vision {

  public enum HeightOfAlgae {
    L2,
    L3,
    NULL
  }

  Swerve swerve;
  List<Integer> reefPegTag = new ArrayList<Integer>();
  int lockID = 0;
  PhotonTrackedTarget trackedTarget;

  // field units are in meters, so we want to be approx 1 meter from target
  double desiredDistFromTag = 1;
  // we want to be close to the reef to intake an algae but we don't want to slam into the reef
  double desiredCloseUpDistFromTag = (Constants.Swerve.robotWidth / 2);
  Pose2d desiredPoseCenterAlign = new Pose2d();
  Pose2d desiredPoseCloseCenterAlign = new Pose2d();
  Pose2d origin = new Pose2d();

  double robotHalfLength = Units.inchesToMeters(18);
  double distTagToPeg = Units.inchesToMeters(6.25);

  Pose2d desiredPoseRelativeToCenterRotated = new Pose2d();
  double angleToRotateBy = 0.0;

  Translation2d reefCenter = new Translation2d();

  double desiredRotation = 0.0;

  Translation2d minimumTranslationProcessor = new Translation2d();
  Translation2d maximumTranslationProcessor = new Translation2d();
  Pose2d processorAlignPosition = new Pose2d();
  boolean isInBounds = false;
  boolean lml3NoTarget = true;
  boolean lml2LNoTarget = true;
  boolean lml2RNoTarget = true;

  HeightOfAlgae currentAlgaeHeight = HeightOfAlgae.NULL;

  public enum direction {
    left,
    right
  }

  public Selection(Swerve swerve) {
    this.swerve = swerve;

    try {
      var alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Blue) {
        minimumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(200.0), Units.inchesToMeters(0.0));
        maximumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(300.0), Units.inchesToMeters(100.0));
        processorAlignPosition =
            new Pose2d(
                Units.inchesToMeters(240),
                Units.inchesToMeters(30),
                new Rotation2d(Units.degreesToRadians(-90)));

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
                Units.inchesToMeters(455.15),
                Units.inchesToMeters(299.455),
                new Rotation2d(Units.degreesToRadians(90)));

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

    } catch (NoSuchElementException e) {
      reefPegTag.clear();
    }
  }

  @Override
  public void periodic() {
    setLockTarget();
    isInBoundsForProcessor();
    System.out.println(lockID);
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

  public double getLockID() {
    return lockID;
  }

  private void setDesiredAlignPose() {
    if (lockID != 0) {
      double tagYaw = GetTagYaw();
      if (tagYaw + Math.toRadians(180) > Units.degreesToRadians(180)) {
        desiredRotation = tagYaw - Math.toRadians(180);
      } else {
        desiredRotation = tagYaw + Math.toRadians(180);
      }

      desiredPoseCenterAlign =
          new Pose2d(
              GetTagTranslation().getX() + (Math.cos(tagYaw) * desiredDistFromTag),
              GetTagTranslation().getY() + (Math.sin(tagYaw) * desiredDistFromTag),
              new Rotation2d(desiredRotation));

      desiredPoseCloseCenterAlign =
          new Pose2d(
              GetTagTranslation().getX() + (Math.cos(tagYaw) * desiredCloseUpDistFromTag),
              GetTagTranslation().getY() + (Math.sin(tagYaw) * desiredCloseUpDistFromTag),
              new Rotation2d(desiredRotation));

    } else if (lockID == 0 && isInBoundsForProcessor()) {
      desiredPoseCenterAlign = processorAlignPosition;
    } else {
      desiredPoseCenterAlign = Pose2d.kZero;
    }
  }

  public Pose2d getDesiredposeAlgae() {
    setDesiredAlignPose();
    return desiredPoseCenterAlign;
  }

  public Pose2d getDesiredCloseUpPoseAlgae() {
    setDesiredAlignPose();
    return desiredPoseCloseCenterAlign;
  }

  public Pose2d getDesiredposeLeft() {
    if (lockID == 0) {
      return Pose2d.kZero;
    }
    var robotTranslationLeft =
        new Translation2d(robotHalfLength, -distTagToPeg + Units.inchesToMeters(2.5));
    var robotPoseRelativeToCenter =
        origin.transformBy(
            new Transform2d(robotTranslationLeft, new Rotation2d(Math.toRadians(-180))));
    angleToRotateBy = reefPegTag.indexOf(lockID) * 60;

    desiredPoseRelativeToCenterRotated =
        robotPoseRelativeToCenter.rotateAround(
            reefCenter, new Rotation2d(Math.toRadians(angleToRotateBy)));
    return desiredPoseRelativeToCenterRotated;
  }

  public Pose2d getDesiredposeRight() {
    if (lockID == 0) {
      return Pose2d.kZero;
    }
    var robotTranslationRight =
        new Translation2d(robotHalfLength, distTagToPeg + Units.inchesToMeters(3));
    var robotPoseRelativeToCenter =
        origin.transformBy(
            new Transform2d(robotTranslationRight, new Rotation2d(Math.toRadians(-180))));
    angleToRotateBy = reefPegTag.indexOf(lockID) * 60;

    desiredPoseRelativeToCenterRotated =
        robotPoseRelativeToCenter.rotateAround(
            reefCenter, new Rotation2d(Math.toRadians(angleToRotateBy)));

    return desiredPoseRelativeToCenterRotated;
  }

  private void setLockTarget() {

    for (var change : cameraLml3.getAllUnreadResults()) {

      if (change.hasTargets()) {
        lml3NoTarget = false;
        trackedTarget = change.getBestTarget();
        lockID = trackedTarget.fiducialId;
        if (reefPegTag.indexOf(lockID) == -1) {
          lml3NoTarget = true;
        }
      } else {
        lml3NoTarget = true;
      }
    }

    for (var change : cameraLml2Left.getAllUnreadResults()) {

      if (change.hasTargets()) {
        lml2LNoTarget = false;
        trackedTarget = change.getBestTarget();
        lockID = trackedTarget.fiducialId;
        if (reefPegTag.indexOf(lockID) == -1) {
          lml2LNoTarget = true;
        }
      } else {
        lml2LNoTarget = true;
      }
    }

    for (var change : cameraLml2Right.getAllUnreadResults()) {

      if (change.hasTargets()) {
        lml2RNoTarget = false;
        trackedTarget = change.getBestTarget();
        lockID = trackedTarget.fiducialId;
        if (reefPegTag.indexOf(lockID) == -1) {
          lml2RNoTarget = true;
        }
      } else {
        lml2RNoTarget = true;
      }
    }

    if (lml3NoTarget && lml2LNoTarget && lml2RNoTarget) {
      lockID = 0;
    }
  }

  private double GetTagYaw() {
    if (lockID != 0) {
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

  public HeightOfAlgae getAlgaeHeight() {

    if (reefPegTag.indexOf(lockID) == 0 || reefPegTag.indexOf(lockID) == 2 || reefPegTag.indexOf(lockID) == 4) {
      currentAlgaeHeight = HeightOfAlgae.L3;

    } else if (reefPegTag.indexOf(lockID) == 1 || reefPegTag.indexOf(lockID) == 3 || reefPegTag.indexOf(lockID) == 5) {
      currentAlgaeHeight = HeightOfAlgae.L2;

    } else {
      currentAlgaeHeight = HeightOfAlgae.NULL;
    }


    return currentAlgaeHeight;

  }
}
