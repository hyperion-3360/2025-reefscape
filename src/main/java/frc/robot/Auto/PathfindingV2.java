package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.swerve.Swerve;

public class PathfindingV2 extends Command {
  Shooter m_shooter;
  Elevator m_elevator;
  LEDs m_leds;
  Swerve m_swerve;

  PathConstraints constraints = new PathConstraints(5.0, 4.0, 1, 2);

  public PathfindingV2(Shooter shooter, Elevator elevator, LEDs leds, Swerve swerve) {
    m_shooter = shooter;
    m_elevator = elevator;
    m_leds = leds;
    m_swerve = swerve;

    addRequirements(m_shooter, m_elevator, m_leds, m_swerve);
  }

  public Command goThere(Pose2d pose) {
    return goThere(pose, 0);
  }

  public Command goThere(Pose2d pose, double goalEndVelocity) {
    return AutoBuilder.pathfindToPose(pose, constraints, goalEndVelocity);
  }

  private Pose2d computeRobotOffset(Pose2d pose) {
    double robotLengthPlusBuffer = (Constants.Swerve.robotLength / 2) * 1.1;
    double robotWidthPlusBuffer = (Constants.Swerve.robotWidth / 2) * 1.1;

    Rotation2d rotation = pose.getRotation();

    // calculates the coordinates to displace the robot actual wanted position
    // relative to the POI
    Translation2d widthToBacktrack =
        new Translation2d(
            pose.getX() + robotLengthPlusBuffer * rotation.getCos(),
            pose.getY() + robotWidthPlusBuffer * rotation.getSin());

    return new Pose2d(widthToBacktrack, rotation);
  }

  public Pose2d offsetPose(Pose2d originalPose, double offset) {
    // Offset by 0.05 meters in the direction of the current angle
    double offsetX =
        originalPose.getTranslation().getX()
            + offset * Math.cos(originalPose.getRotation().getRadians());
    double offsetY =
        originalPose.getTranslation().getY()
            + offset * Math.sin(originalPose.getRotation().getRadians());

    // Create a new Pose2d with the offset translation and the same rotation
    Translation2d offsetTranslation = new Translation2d(offsetX, offsetY);
    Rotation2d sameRotation = originalPose.getRotation();

    return new Pose2d(offsetTranslation, sameRotation);
  }

  public Command auto() {
    var branchERot = AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchE.getRotation();
    var feederRot = AutoWaypoints.tagLayout.getTagPose(12).get().getRotation().toRotation2d();
    var branchDRot = AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchD.getRotation();
    var branchCRot = AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchC.getRotation();

    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());
    pathfindingSequence.addCommands(
        goThere(new Pose2d(6, 1.5, branchERot), 3.0),
        new InstantCommand(
            () ->
                m_swerve.drivetoTarget(AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchE)),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new PrintCommand("Dropping coral to L4"),
        goThere(new Pose2d(2.5, 1.75, feederRot), 3.0),
        new InstantCommand(
            () ->
                m_swerve.drivetoTarget(
                    computeRobotOffset(
                        Conversions.Pose3dToPose2d(AutoWaypoints.tagLayout.getTagPose(12).get())))),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new PrintCommand("Feeding coral"),
        goThere(new Pose2d(2.5, 1.75, branchDRot), 3.0),
        new InstantCommand(
            () ->
                m_swerve.drivetoTarget(AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchD)),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        goThere(new Pose2d(2.5, 1.75, feederRot), 3.0),
        new InstantCommand(
            () ->
                m_swerve.drivetoTarget(
                    computeRobotOffset(
                        Conversions.Pose3dToPose2d(AutoWaypoints.tagLayout.getTagPose(12).get())))),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new PrintCommand("Feeding coral"),
        goThere(new Pose2d(2.5, 1.75, branchCRot), 3.0),
        new InstantCommand(
            () ->
                m_swerve.drivetoTarget(AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchC)),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new PrintCommand("Dropping coral to L4"));
    return pathfindingSequence;
  }

  public Command hexagon() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());
    pathfindingSequence.addCommands(
        goThere(AutoWaypoints.BlueAlliance.stopPathplannerWaypoint.sideOne, 2.0),
        goThere(AutoWaypoints.BlueAlliance.stopPathplannerWaypoint.sideTwo, 2.0),
        goThere(AutoWaypoints.BlueAlliance.stopPathplannerWaypoint.sideThree, 2.0),
        goThere(AutoWaypoints.BlueAlliance.stopPathplannerWaypoint.sideFour, 2.0),
        goThere(AutoWaypoints.BlueAlliance.stopPathplannerWaypoint.sideFive, 2.0),
        goThere(AutoWaypoints.BlueAlliance.stopPathplannerWaypoint.sideSix, 2.0));

    return pathfindingSequence;
  }
}
