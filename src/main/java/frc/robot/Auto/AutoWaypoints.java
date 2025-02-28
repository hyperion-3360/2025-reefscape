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
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    double distanceTagToPeg = Units.inchesToMeters(6.5);
    double robotHalfLength = Units.inchesToMeters(-19);

    double distanceRobotCenterToReef = 1.5;
    double distanceRotbotCenterTag = 0.0;
  
    public class BlueAlliance {

        Translation2d reefCenter = new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));

        Pose2d origin =
        new Pose2d(
            tagLayout.getTagPose(18).get().getX(),
            tagLayout.getTagPose(18).get().getY(),
            tagLayout.getTagPose(18).get().getRotation().toRotation2d());
    

        public class LeftSide {


            public class pegWaypoints {

                Pose2d branchA = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(0)));
                Pose2d branchL = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(300)));
                Pose2d branchK = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(300)));
                Pose2d branchJ = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(240)));
                Pose2d branchI = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(240)));
                Pose2d branchH = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(180)));

            }


            public class feederWaypoints {

            }

        }

        public class RightSide {


            public class pegWaypoints {

                Pose2d branchB = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(0)));
                Pose2d branchC = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(60)));
                Pose2d branchD = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(60)));
                Pose2d branchE = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(120)));
                Pose2d branchF = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(120)));
                Pose2d branchG = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(180)));

            }

            public class feederWaypoints {

            }

        }

        public class stopPathplannerWaypoint {

            Pose2d sideOne = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(0)));
            Pose2d sideTwo = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(60)));
            Pose2d sideThree = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(120)));
            Pose2d sideFour = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(180)));
            Pose2d sideFive = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(240)));
            Pose2d sideSix = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(300)));
        }

    }

    public class RedAlliance {

        Translation2d reefCenter = new Translation2d(Units.inchesToMeters(514.14), Units.inchesToMeters(158.5));

        Pose2d origin =
        new Pose2d(
            tagLayout.getTagPose(10).get().getX(),
            tagLayout.getTagPose(10).get().getY(),
            tagLayout.getTagPose(10).get().getRotation().toRotation2d());

        public class LeftSide {


            public class pegWaypoints {

                Pose2d branchA = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(0)));
                Pose2d branchL = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(300)));
                Pose2d branchK = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(300)));
                Pose2d branchJ = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(240)));
                Pose2d branchI = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(240)));
                Pose2d branchH = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(180)));

            }



            public class feederWaypoints {

            }

        }

        public class RightSide {

            public class pegWaypoints {

                Pose2d branchB = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(0)));
                Pose2d branchC = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(60)));
                Pose2d branchD = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(60)));
                Pose2d branchE = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(120)));
                Pose2d branchF = origin.transformBy(
                    new Transform2d(robotHalfLength, -distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(120)));
                Pose2d branchG = origin.transformBy(
                    new Transform2d(robotHalfLength, distanceTagToPeg, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                            reefCenter, new Rotation2d(Math.toRadians(180)));
            }



            public class feederWaypoints {

            }

        }

        public class stopPathplannerWaypoint {

            Pose2d sideOne = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(0)));
            Pose2d sideTwo = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(60)));
            Pose2d sideThree = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(120)));
            Pose2d sideFour = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(180)));
            Pose2d sideFive = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(240)));
            Pose2d sideSix = origin.transformBy(
                new Transform2d(distanceRobotCenterToReef, distanceRotbotCenterTag, new Rotation2d(Math.toRadians(-180)))).rotateAround(
                                        reefCenter, new Rotation2d(Math.toRadians(300)));

        }

    }

}