package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Auto.Pathfinding;
import frc.robot.Auto.Pathfinding.POI;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class AutoNet extends SequentialCommandGroup {
  public AutoNet(Elevator m_elevator, LEDs m_leds, AlgaeIntake m_algaeIntake) {
    addRequirements(m_elevator, m_algaeIntake, m_leds);
    addCommands(
        Commands.sequence(
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.L2)),
                Commands.runOnce(() -> m_algaeIntake.setShootingAngle(elevation.NET)),
                new WaitCommand(0.5),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
                new WaitUntilCommand(() -> Pathfinding.isCloseToPOI(POI.NET)),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.NET)),
                Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE)),
                new WaitCommand(0.3),
                Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.NET)),
                new WaitUntilCommand(() -> !m_algaeIntake.sensorTriggered()))
            .finallyDo(
                () ->
                    Commands.sequence(
                        new WaitCommand(0.5),
                        Commands.runOnce(
                            () -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
                        Commands.runOnce(
                            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED)),
                        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.L1)),
                        new WaitCommand(1.4),
                        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
                        new WaitCommand(0.5),
                        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)))));
  }
}
