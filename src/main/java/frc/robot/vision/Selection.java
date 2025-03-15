// package frc.robot.vision;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.robot.Constants;
// import java.util.NoSuchElementException;
// import org.photonvision.targeting.PhotonTrackedTarget;

// public class Selection {

//   Vision m_vision;

//   PhotonTrackedTarget trackedTarget;

//   // field units are in meters, so we want to be approx 1 meter from target
//   double desiredDistFromTag = 1;
//   Pose2d desiredPoseCenterAlign = new Pose2d();
//   Pose2d origin = new Pose2d();

//   double robotHalfLength = Units.inchesToMeters(18);
//   double distTagToPeg = Units.inchesToMeters(6.25);

//   Pose2d desiredPoseRelativeToCenterRotated = new Pose2d();
//   double angleToRotateBy = 0.0;

//   Translation2d reefCenter = new Translation2d();

//   double desiredRotation = 0.0;

//   Translation2d minimumTranslationProcessor = new Translation2d();
//   Translation2d maximumTranslationProcessor = new Translation2d();
//   Pose2d processorAlignPosition = new Pose2d();
//   boolean isInBounds = false;
//   boolean lml3NoTarget = true;
//   boolean lml2LNoTarget = true;
//   boolean lml2RNoTarget = true;

//   public Selection(Vision vision) {
//     m_vision = vision;
//     try {
//       var alliance = DriverStation.getAlliance().get();
//       if (alliance == Alliance.Blue) {

//         reefCenter = new Translation2d(Units.inchesToMeters(176.75),
// Units.inchesToMeters(158.5));
//         origin =
//             new Pose2d(
//                 Constants.tagLayout.getTagPose(18).get().getX(),
//                 Constants.tagLayout.getTagPose(18).get().getY(),
//                 Constants.tagLayout.getTagPose(18).get().getRotation().toRotation2d());

//       } else if (alliance == Alliance.Red) {

//         minimumTranslationProcessor =
//             new Translation2d(Units.inchesToMeters(420.0), Units.inchesToMeters(217.0));
//         maximumTranslationProcessor =
//             new Translation2d(Units.inchesToMeters(490.0), Units.inchesToMeters(500));
//         processorAlignPosition =
//             new Pose2d(
//                 Units.inchesToMeters(455.15),
//                 Units.inchesToMeters(299.455),
//                 new Rotation2d(Units.degreesToRadians(90)));

//         reefCenter = new Translation2d(Units.inchesToMeters(514.14),
// Units.inchesToMeters(158.5));
//         origin =
//             new Pose2d(
//                 Constants.tagLayout.getTagPose(7).get().getX(),
//                 Constants.tagLayout.getTagPose(7).get().getY(),
//                 Constants.tagLayout.getTagPose(7).get().getRotation().toRotation2d());
//       }
//     } catch (NoSuchElementException e) {
//     }
//   }

//   @Override
//   public void periodic() {
//     setLockTarget();
//     isInBoundsForProcessor();
//     // System.out.println(lockID);
//   }

//   public boolean isInBoundsForProcessor() {
//     if (lockID == 0) {
//       if (swerve.getPose().getX() < maximumTranslationProcessor.getX()
//           && swerve.getPose().getX() > minimumTranslationProcessor.getX()
//           && swerve.getPose().getY() < maximumTranslationProcessor.getY()
//           && swerve.getPose().getY() > minimumTranslationProcessor.getY()) {
//         isInBounds = true;
//       }
//     } else {
//       isInBounds = false;
//     }
//     return isInBounds;
//   }

//   private void setDesiredAlignPose() {
//     if (lockID != 0) {
//       double tagYaw = GetTagYaw();
//       if (tagYaw + Math.toRadians(180) > Units.degreesToRadians(180)) {
//         desiredRotation = tagYaw - Math.toRadians(180);
//       } else {
//         desiredRotation = tagYaw + Math.toRadians(180);
//       }

//       desiredPoseCenterAlign =
//           new Pose2d(
//               GetTagTranslation().getX() + (Math.cos(tagYaw) * desiredDistFromTag),
//               GetTagTranslation().getY() + (Math.sin(tagYaw) * desiredDistFromTag),
//               new Rotation2d(desiredRotation));

//     } else if (lockID == 0 && isInBoundsForProcessor()) {
//       desiredPoseCenterAlign = processorAlignPosition;
//     } else {
//       desiredPoseCenterAlign = Pose2d.kZero;
//     }
//   }

//   public Pose2d getDesiredposeAlgae() {
//     setDesiredAlignPose();
//     return desiredPoseCenterAlign;
//   }

//   public Pose2d getDesiredposeLeft() {
//     if (lockID == 0) {
//       return Pose2d.kZero;
//     }
//     var robotTranslationLeft =
//         new Translation2d(robotHalfLength, -distTagToPeg + Units.inchesToMeters(2.5));
//     var robotPoseRelativeToCenter =
//         origin.transformBy(
//             new Transform2d(robotTranslationLeft, new Rotation2d(Math.toRadians(-180))));
//     angleToRotateBy = reefPegTag.indexOf(lockID) * 60;

//     desiredPoseRelativeToCenterRotated =
//         robotPoseRelativeToCenter.rotateAround(
//             reefCenter, new Rotation2d(Math.toRadians(angleToRotateBy)));
//     return desiredPoseRelativeToCenterRotated;
//   }

//   public Pose2d getDesiredposeRight() {
//     if (lockID == 0) {
//       return Pose2d.kZero;
//     }
//     var robotTranslationRight =
//         new Translation2d(robotHalfLength, distTagToPeg + Units.inchesToMeters(3));
//     var robotPoseRelativeToCenter =
//         origin.transformBy(
//             new Transform2d(robotTranslationRight, new Rotation2d(Math.toRadians(-180))));
//     angleToRotateBy = reefPegTag.indexOf(lockID) * 60;

//     desiredPoseRelativeToCenterRotated =
//         robotPoseRelativeToCenter.rotateAround(
//             reefCenter, new Rotation2d(Math.toRadians(angleToRotateBy)));

//     return desiredPoseRelativeToCenterRotated;
//   }

//   private void setLockTarget() {

//     for (var change : cameraLml3.getAllUnreadResults()) {

//       if (change.hasTargets()) {
//         lml3NoTarget = false;
//         trackedTarget = change.getBestTarget();
//         lockID = trackedTarget.fiducialId;
//         if (reefPegTag.indexOf(lockID) == -1) {
//           lml3NoTarget = true;
//         }
//       } else {
//         lml3NoTarget = true;
//       }
//     }

//     for (var change : cameraLml2Left.getAllUnreadResults()) {

//       if (change.hasTargets()) {
//         lml2LNoTarget = false;
//         trackedTarget = change.getBestTarget();
//         lockID = trackedTarget.fiducialId;
//         if (reefPegTag.indexOf(lockID) == -1) {
//           lml2LNoTarget = true;
//         }
//       } else {
//         lml2LNoTarget = true;
//       }
//     }

//     for (var change : cameraLml2Right.getAllUnreadResults()) {

//       if (change.hasTargets()) {
//         lml2RNoTarget = false;
//         trackedTarget = change.getBestTarget();
//         lockID = trackedTarget.fiducialId;
//         if (reefPegTag.indexOf(lockID) == -1) {
//           lml2RNoTarget = true;
//         }
//       } else {
//         lml2RNoTarget = true;
//       }
//     }

//     if (lml3NoTarget && lml2LNoTarget && lml2RNoTarget) {
//       lockID = 0;
//     }
//   }

//   private double GetTagYaw() {
//     if (lockID != 0) {
//       return Constants.tagLayout.getTagPose(lockID).get().getRotation().getZ();
//     }
//     return 0.0;
//   }

//   private Translation2d GetTagTranslation() {

//     if (lockID != 0) {

//       var x = Constants.tagLayout.getTagPose(lockID).get().getX();
//       var y = Constants.tagLayout.getTagPose(lockID).get().getY();

//       return new Translation2d(x, y);
//     }

//     return new Translation2d();
//   }
// }
