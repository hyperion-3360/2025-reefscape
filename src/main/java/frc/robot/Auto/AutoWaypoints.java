package frc.robot.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class AutoWaypoints {

  // everything is in meters and degrees
  static double distanceTagToPeg = Units.inchesToMeters(-7.725);
  static double robotHalfLength = Units.inchesToMeters(17);

  static double distanceRobotCenterToReef = 1.5;
  static double distanceRotbotCenterTag = 0.0;

  // #region BLUE alliance
  public static class BlueAlliance {

    static Translation2d reefCenter =
        new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));

    static Pose2d origin =
        new Pose2d(
            Constants.tagLayout.getTagPose(18).get().getX(),
            Constants.tagLayout.getTagPose(18).get().getY(),
            Constants.tagLayout.getTagPose(18).get().getRotation().toRotation2d());

    // #region Left
    public static class LeftSide {

      public static class NetWaypoint {
        static Pose2d net = new Pose2d(6.900, 5.157, new Rotation2d(Math.toRadians(0)));
        static Pose2d netSecondAlgae = new Pose2d(6.900, 5.157, new Rotation2d(Math.toRadians(0)));
        static Pose2d netAngled = new Pose2d(7.453, 4.0, new Rotation2d(Math.toRadians(45.821)));
      }

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

      public static class AlgaeWaypoint {
        static Pose2d AlgaeHG = new Pose2d(5.242, 4, new Rotation2d(Math.toRadians(180)));
      }

      public static class feederWaypoints {
        public static Pose2d feederRight = new Pose2d(1.405, 7.246, Rotation2d.fromDegrees(306));
        public static Pose2d feederLeft = new Pose2d(0.887, 6.828, Rotation2d.fromDegrees(306));
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
        public static Pose2d feederRight = new Pose2d(1.529, 0.737, Rotation2d.fromDegrees(54));
        public static Pose2d feederLeft = new Pose2d(0.666, 1.370, Rotation2d.fromDegrees(54));
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
            Constants.tagLayout.getTagPose(7).get().getX(),
            Constants.tagLayout.getTagPose(7).get().getY(),
            Constants.tagLayout.getTagPose(7).get().getRotation().toRotation2d());

    // #region Left
    public static class LeftSide {

      public static class AlgaeWaypoint {
        static Pose2d AlgaeHG = new Pose2d(12.307, 4, new Rotation2d(Math.toRadians(0)));
      }

      public static class NetWaypoint {
        static Pose2d net = new Pose2d(10.420, 3.057, new Rotation2d(Math.toRadians(180)));
        static Pose2d netSecondAlgae =
            new Pose2d(10.420, 1.847, new Rotation2d(Math.toRadians(180)));
        static Pose2d netAngled =
            new Pose2d(10.010, 3.884, new Rotation2d(Math.toRadians(-138.905)));
      }

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
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(60)));
        static Pose2d branchK =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(60)));
        static Pose2d branchJ =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(120)));
        static Pose2d branchI =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(120)));
        static Pose2d branchH =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(180)));
      }

      public static class feederWaypoints {
        public static Pose2d feederRight = new Pose2d(15.840, 7.405, Rotation2d.fromDegrees(234));
        public static Pose2d feederLeft = new Pose2d(16.892, 6.660, Rotation2d.fromDegrees(234));
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
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(300)));
        static Pose2d branchD =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(300)));
        static Pose2d branchE =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(240)));
        static Pose2d branchF =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(240)));
        static Pose2d branchG =
            origin
                .transformBy(
                    new Transform2d(
                        robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180))))
                .rotateAround(reefCenter, new Rotation2d(Math.toRadians(180)));
      }

      public static class feederWaypoints {
        public static Pose2d feederRight = new Pose2d(15.876, 0.646, Rotation2d.fromDegrees(126));
        public static Pose2d feederLeft = new Pose2d(16.857, 1.392, Rotation2d.fromDegrees(126));
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
