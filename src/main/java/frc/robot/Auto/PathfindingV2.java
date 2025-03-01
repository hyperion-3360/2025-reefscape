package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.Conversions;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.swerve.Swerve;

public class PathfindingV2 extends Command {
  Shooter m_shooter;
  Elevator m_elevator;
  LEDs m_leds;
  Swerve m_swerve;

  PathConstraints constraints = new PathConstraints(1.0, 1.0, 1, 2);

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

  public Command auto() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());
    pathfindingSequence.addCommands(
        goThere(new Pose2d(7.3, 3, Rotation2d.k180deg), 1.0),
        new InstantCommand(
            () ->
                m_swerve.drivetoTarget(AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchE)),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        new PrintCommand("Dropping coral to L4"),
        goThere(new Pose2d(1, 1, Rotation2d.kZero), 1.0),
        new InstantCommand(
            () ->
                m_swerve.drivetoTarget(
                    Conversions.Pose3dToPose2d(AutoWaypoints.tagLayout.getTagPose(12).get()))),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        new PrintCommand("Feeding coral"));
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
