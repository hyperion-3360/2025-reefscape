package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Selection extends Vision {

  Swerve m_swerve;
  List<Integer> m_reefPegTag = new ArrayList<Integer>();
  int m_lockID = 0;
  PhotonTrackedTarget m_trackedTarget;

  // field units are in meters, so we want to be approx 1 meter from target
  double m_desiredDistFromTag = 1;
  Pose2d m_desiredPoseCenterAlign = new Pose2d();
  Pose2d m_origin = new Pose2d();

  final double robotHalfLength = Units.inchesToMeters(20);
  final double distTagToPeg = Units.inchesToMeters(5.75);

  Pose2d m_desiredPoseRelativeToCenterRotated = new Pose2d();
  double m_angleToRotateBy = 0.0;

  Translation2d m_reefCenter = new Translation2d();

  double m_desiredRotation = 0.0;

  Translation2d m_minimumTranslationProcessor = new Translation2d();
  Translation2d m_maximumTranslationProcessor = new Translation2d();
  Pose2d m_processorAlignPosition = new Pose2d();
  boolean m_isInBounds = false;

  public enum direction {
    left,
    right
  }

  public Selection(Swerve swerve) {
    this.m_swerve = swerve;

    try {
      var alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Blue) {
        m_minimumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(200.0), Units.inchesToMeters(0.0));
        m_maximumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(300.0), Units.inchesToMeters(100.0));
        m_processorAlignPosition =
            new Pose2d(
                Units.inchesToMeters(240),
                Units.inchesToMeters(30),
                new Rotation2d(Units.degreesToRadians(-90)));

        // real reef values : 176.75, 158.5
        // measured : 176.25, 157.625
        m_reefCenter = new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));
        m_origin =
            new Pose2d(
                tagLayout.getTagPose(18).get().getX(),
                tagLayout.getTagPose(18).get().getY(),
                tagLayout.getTagPose(18).get().getRotation().toRotation2d());
        m_reefPegTag.clear();
        m_reefPegTag.add(18);
        m_reefPegTag.add(17);
        m_reefPegTag.add(22);
        m_reefPegTag.add(21);
        m_reefPegTag.add(20);
        m_reefPegTag.add(19);

      } else if (alliance == Alliance.Red) {

        m_minimumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(420.0), Units.inchesToMeters(217.0));
        m_maximumTranslationProcessor =
            new Translation2d(Units.inchesToMeters(490.0), Units.inchesToMeters(500));
        m_processorAlignPosition =
            new Pose2d(
                Units.inchesToMeters(455.15),
                Units.inchesToMeters(299.455),
                new Rotation2d(Units.degreesToRadians(90)));

        m_reefCenter = new Translation2d(Units.inchesToMeters(514.14), Units.inchesToMeters(158.5));
        m_origin =
            new Pose2d(
                tagLayout.getTagPose(7).get().getX(),
                tagLayout.getTagPose(7).get().getY(),
                tagLayout.getTagPose(7).get().getRotation().toRotation2d());
        m_reefPegTag.clear();
        m_reefPegTag.add(7);
        m_reefPegTag.add(8);
        m_reefPegTag.add(9);
        m_reefPegTag.add(10);
        m_reefPegTag.add(11);
        m_reefPegTag.add(6);
      } else {
        m_reefPegTag.clear();
      }

    } catch (NoSuchElementException e) {
      m_reefPegTag.clear();
    }
  }

  @Override
  public void periodic() {
    setLockTarget();
    isInBoundsForProcessor();
    System.out.println(m_lockID);
  }

  public boolean isInBoundsForProcessor() {
    if (m_lockID == 0) {
      if (m_swerve.getPose().getX() < m_maximumTranslationProcessor.getX()
          && m_swerve.getPose().getX() > m_minimumTranslationProcessor.getX()
          && m_swerve.getPose().getY() < m_maximumTranslationProcessor.getY()
          && m_swerve.getPose().getY() > m_minimumTranslationProcessor.getY()) {
        m_isInBounds = true;
      }
    } else {
      m_isInBounds = false;
    }
    return m_isInBounds;
  }

  public double getLockID() {
    return m_lockID;
  }

  private void setDesiredAlignPose() {
    if (m_lockID != 0) {
      double tagYaw = GetTagYaw();
      if (tagYaw + Math.toRadians(180) > Units.degreesToRadians(180)) {
        m_desiredRotation = tagYaw - Math.toRadians(180);
      } else {
        m_desiredRotation = tagYaw + Math.toRadians(180);
      }

      m_desiredPoseCenterAlign =
          new Pose2d(
              GetTagTranslation().getX() + (Math.cos(tagYaw) * m_desiredDistFromTag),
              GetTagTranslation().getY() + (Math.sin(tagYaw) * m_desiredDistFromTag),
              new Rotation2d(m_desiredRotation));

    } else if (m_lockID == 0 && isInBoundsForProcessor()) {
      m_desiredPoseCenterAlign = m_processorAlignPosition;
    } else {
      m_desiredPoseCenterAlign = Pose2d.kZero;
    }
  }

  public Pose2d getDesiredposeAlgae() {
    setDesiredAlignPose();
    return m_desiredPoseCenterAlign;
  }

  public Pose2d getDesiredposeLeft() {
    if (m_lockID == 0) {
      return Pose2d.kZero;
    }
    var robotTranslationLeft = new Translation2d(robotHalfLength, -distTagToPeg);
    var robotPoseRelativeToCenter =
        m_origin.transformBy(
            new Transform2d(robotTranslationLeft, new Rotation2d(Math.toRadians(-180))));
    m_angleToRotateBy = m_reefPegTag.indexOf(m_lockID) * 60;

    m_desiredPoseRelativeToCenterRotated =
        robotPoseRelativeToCenter.rotateAround(
            m_reefCenter, new Rotation2d(Math.toRadians(m_angleToRotateBy)));
    return m_desiredPoseRelativeToCenterRotated;
  }

  public Pose2d getDesiredposeRight() {
    if (m_lockID == 0) {
      return Pose2d.kZero;
    }
    var robotTranslationRight =
        //        new Translation2d(robotHalfLength, distTagToPeg + Units.inchesToMeters(1.5));
        new Translation2d(robotHalfLength, distTagToPeg);
    var robotPoseRelativeToCenter =
        m_origin.transformBy(
            new Transform2d(robotTranslationRight, new Rotation2d(Math.toRadians(-180))));
    m_angleToRotateBy = m_reefPegTag.indexOf(m_lockID) * 60;

    m_desiredPoseRelativeToCenterRotated =
        robotPoseRelativeToCenter.rotateAround(
            m_reefCenter, new Rotation2d(Math.toRadians(m_angleToRotateBy)));

    return m_desiredPoseRelativeToCenterRotated;
  }

  private boolean allowedTarget(PhotonTrackedTarget target) {
    return m_reefPegTag.contains(target.getFiducialId());
  }

  private void setLockTarget() {
    // get all results from all cameras
    var allResults =
        Stream.of(
                cameraLml3.getAllUnreadResults().stream(),
                cameraLml2Left.getAllUnreadResults().stream(),
                cameraLml2Right.getAllUnreadResults().stream())
            .flatMap(i -> i)
            .collect(Collectors.toList());

    // initialize variables to large or impossible values
    double targetAmbibuity = 10.0;
    double targetDistance = 10.0;
    PhotonTrackedTarget bestTarget = null;
    int bestID = 0;

    // iterate through all results and find
    // the best target that is allowed
    for (var result : allResults) {
      if (result.hasTargets()) {
        var currentTarget = result.getBestTarget();
        var distance = currentTarget.getBestCameraToTarget().getTranslation().getNorm();

        // if the target is allowed, closer than the current target
        if (allowedTarget(currentTarget)
            && (distance < targetDistance)
            && (currentTarget.getPoseAmbiguity() < targetAmbibuity)) {
          targetDistance = distance;
          targetAmbibuity = currentTarget.getPoseAmbiguity();
          bestID = currentTarget.getFiducialId();
          bestTarget = currentTarget;
        }
      }
      // if a target is found, set the lockID and trackedTarget
      if (bestID != 0) {
        m_lockID = bestID;
        m_trackedTarget = bestTarget;
      }
    }
  }

  private double GetTagYaw() {
    if (m_lockID != 0) {
      return tagLayout.getTagPose(m_lockID).get().getRotation().getZ();
    }
    return 0.0;
  }

  private Translation2d GetTagTranslation() {

    if (m_lockID != 0) {

      var x = tagLayout.getTagPose(m_lockID).get().getX();
      var y = tagLayout.getTagPose(m_lockID).get().getY();

      return new Translation2d(x, y);
    }

    return new Translation2d();
  }
}
