package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class AutoFeed extends SequentialCommandGroup {
  public AutoFeed(Elevator m_elevator, Shooter m_shooter, LEDs m_leds) {
    addRequirements(m_elevator);
    addRequirements(m_shooter);
    addRequirements(m_leds);
    addCommands(
        Commands.sequence(
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.FEEDER)),
                Commands.runOnce(() -> m_shooter.closeBlocker()),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.INTAKE)),
                Commands.run(() -> m_shooter.setIntake()).until(() -> m_shooter.isCoralIn()),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.READY)))
            .finallyDo(
                () ->
                    Commands.sequence(
                        new WaitCommand(1.2),
                        Commands.runOnce(() -> m_shooter.stop()),
                        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)))));
  }
}
