package frc.robot.Auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class AutoWaypoints {

  // everything is in meters and degrees
  static AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  static double distanceTagToPeg = Units.inchesToMeters(-6.5);
  static double robotHalfLength = Units.inchesToMeters(19);

  static double distanceRobotCenterToReef = 1.5;
  static double distanceRotbotCenterTag = 0.0;

  // #region BLUE alliance
  public static class BlueAlliance {

    static Translation2d reefCenter =
        new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));

    static Pose2d origin =
        new Pose2d(
            tagLayout.getTagPose(18).get().getX(),
            tagLayout.getTagPose(18).get().getY(),
            tagLayout.getTagPose(18).get().getRotation().toRotation2d());

    // #region Left
    public static class LeftSide {

      public static class pegWaypoints {

        static Pose2d branchA =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(0)));
        static Pose2d branchL =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(300)));
        static Pose2d branchK =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(300)));
        static Pose2d branchJ =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(240)));
        static Pose2d branchI =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(240)));
        static Pose2d branchH =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(180)));
      }

      public static class feederWaypoints {
        public static Pose2d feederRight = new Pose2d(1.450, 7.400, Rotation2d.fromDegrees(306));
        public static Pose2d feederLeft = new Pose2d(0.666, 6.839, Rotation2d.fromDegrees(306));
      }
    }

    // #region Right
    public static class RightSide {

      public class pegWaypoints {

        static Pose2d branchB =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(0)));
        static Pose2d branchC =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(60)));
        static Pose2d branchD =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(60)));
        static Pose2d branchE =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(120)));
        static Pose2d branchF =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(120)));
        static Pose2d branchG =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(180)));
      }

      public static class feederWaypoints {
        public static Pose2d feederRight = new Pose2d(1.450, 0.614, Rotation2d.fromDegrees(54));
        public static Pose2d feederLeft = new Pose2d(0.666, 1.192, Rotation2d.fromDegrees(54));
      }
    }

    // #region Stop
    public static class stopPathplannerWaypoint {

      static Pose2d sideOne =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(0)));
      static Pose2d sideTwo =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(60)));
      static Pose2d sideThree =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(120)));
      static Pose2d sideFour =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(180)));
      static Pose2d sideFive =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(240)));
      static Pose2d sideSix =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(300)));
    }
  }

  // #region RED alliance
  public static class RedAlliance {

    static Translation2d reefCenter =
        new Translation2d(Units.inchesToMeters(514.14), Units.inchesToMeters(158.5));

    static Pose2d origin =
        new Pose2d(
            tagLayout.getTagPose(10).get().getX(),
            tagLayout.getTagPose(10).get().getY(),
            tagLayout.getTagPose(10).get().getRotation().toRotation2d());

    // #region Left
    public static class LeftSide {

      public class pegWaypoints {

        static Pose2d branchA =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(0)));
        static Pose2d branchL =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(300)));
        static Pose2d branchK =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(300)));
        static Pose2d branchJ =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(240)));
        static Pose2d branchI =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(240)));
        static Pose2d branchH =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(180)));
      }

      public static class feederWaypoints {
        public static Pose2d feederRight = new Pose2d(16.835, 6.716, Rotation2d.fromDegrees(234));
        public static Pose2d feederLeft = new Pose2d(15.970, 7.354, Rotation2d.fromDegrees(234));

      }
    }

    // #region Right
    public static class RightSide {

      public static class pegWaypoints {

        static Pose2d branchB =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(0)));
        static Pose2d branchC =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(60)));
        static Pose2d branchD =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(60)));
        static Pose2d branchE =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(120)));
        static Pose2d branchF =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(120)));
        static Pose2d branchG =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(180)));
      }

      public static class feederWaypoints {
        public static Pose2d feederRight = new Pose2d(15.959, 0.666, Rotation2d.fromDegrees(126));
        public static Pose2d feederLeft = new Pose2d(16.873, 1.365, Rotation2d.fromDegrees(126));
}
    }

    // #region Stop
    public static class stopPathplannerWaypoint {

      static Pose2d sideOne =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(0)));
      static Pose2d sideTwo =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(60)));
      static Pose2d sideThree =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(120)));
      static Pose2d sideFour =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(180)));
      static Pose2d sideFive =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(240)));
      static Pose2d sideSix =
          origin
              .transformBy(
                  new Transform2d(
                      distanceRobotCenterToReef,
                      distanceRotbotCenterTag,
                      new Rotation2d(Math.toRadians(-180))))
              .rotateAround(reefCenter, new Rotation2d(Math.toRadians(300)));
    }
  }
}
