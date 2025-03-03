package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.Conversions;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.Supplier;

public class PathfindingV2 extends Command {
  Shooter m_shooter;
  Elevator m_elevator;
  LEDs m_leds;
  Swerve m_swerve;
  public static final double robotLength = Units.inchesToMeters(35.0); // with bumper: 32.5
  public static final double robotWidth = Units.inchesToMeters(35.0); // with bumper: 32.5

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

  public Command goThere(Supplier<Pose2d> pose) {
    return goThere(pose.get(), 0);
  }

  public Command goThere(Pose2d pose, double goalEndVelocity) {
    return AutoBuilder.pathfindToPose(pose, constraints, goalEndVelocity);
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

  public boolean isCloseTo(Pose2d pose, double distance) {
    return m_swerve.getPose().getTranslation().getDistance(pose.getTranslation()) < distance;
  }

  private Command driveAndShootCycle(Pose2d targetPos) {
    var approachPose = offsetPose(targetPos, -1);
    SequentialCommandGroup shootSequence = new SequentialCommandGroup(Commands.none());
    shootSequence.addCommands(
        goThere(approachPose, 0.0),
        new WaitCommand(4.0),
        new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.L4)),
        new InstantCommand(() -> m_swerve.drivetoTarget(targetPos)),
        new ParallelDeadlineGroup(
            new WaitCommand(3.0), // will be elevatecmd(L4) later
            new WaitUntilCommand(() -> m_swerve.targetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new InstantCommand(() -> m_shooter.openBlocker()),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.L4)),
        new WaitCommand(2),
        new InstantCommand(() -> m_shooter.stop()),
        new InstantCommand(() -> m_shooter.closeBlocker()),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.FEEDER)));
    return shootSequence;
  }

  private Command driveAndIntakeCycle(Pose2d targetPos) {
    var approachPose = offsetPose(targetPos, (robotLength / 2) - 0.1);
    SequentialCommandGroup intakeSequence = new SequentialCommandGroup(Commands.none());
    intakeSequence.addCommands(
        goThere(approachPose, 0.0),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.INTAKE)),
        new WaitUntilCommand(() -> m_shooter.isCoralIn()),
        new WaitCommand(0.2), // will be elevatecmd(L4) later
        new InstantCommand(() -> m_shooter.stop()));

    return intakeSequence;
  }

  public Command auto() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());

    pathfindingSequence.addCommands(
        driveAndShootCycle(AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchE),
        driveAndIntakeCycle(
            Conversions.Pose3dToPose2d(AutoWaypoints.tagLayout.getTagPose(12).get())),
        driveAndShootCycle(AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchD),
        driveAndIntakeCycle(
            Conversions.Pose3dToPose2d(AutoWaypoints.tagLayout.getTagPose(12).get())),
        driveAndShootCycle(AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchC));

    return pathfindingSequence;
  }
}
