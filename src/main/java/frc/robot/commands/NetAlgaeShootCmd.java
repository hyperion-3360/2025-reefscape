package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Auto.PathfindingV2;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;

public class NetAlgaeShootCmd extends SequentialCommandGroup {
  double targetX;
  Rotation2d targetRotation;

  public NetAlgaeShootCmd(
      AlgaeIntake m_algaeIntake,
      LEDs m_leds,
      Elevator m_elevator,
      Swerve m_swerve,
      PathfindingV2 m_pathfinding) {
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addRequirements(m_elevator);

    // if we don't have any alliance assume blue alliance
    try {
      targetX = DriverStation.getAlliance().get().equals(Alliance.Blue) ? 7.3 : 10.3;
    } catch (Exception e) {
      targetX = 7.3;
    }

    try {
      targetRotation =
          DriverStation.getAlliance().get().equals(Alliance.Blue)
              ? Rotation2d.kZero
              : Rotation2d.k180deg;
    } catch (Exception e) {
      targetRotation = Rotation2d.kZero;
    }

    addCommands(
        // new ParallelCommandGroup(
        //     new DeferredCommand(
        //         () ->
        //             m_pathfinding.goThere(
        //                 () -> new Pose2d(targetX, m_swerve.getPose().getY(), targetRotation)),
        //         Set.of(m_swerve)),
        //     new WaitUntilCommand(
        //             () ->
        //                 m_pathfinding.isCloseTo(
        //                     new Pose2d(targetX, m_swerve.getPose().getY(), targetRotation), 1))
        //         .andThen(
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.NET)),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(elevation.NET)),
        new WaitCommand(1.3),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE)),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.NET)),
        new WaitUntilCommand(() -> !m_algaeIntake.sensorTriggered()),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        new WaitCommand(0.5),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
