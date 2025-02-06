package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class ElevateCmd extends SequentialCommandGroup {

  public ElevateCmd(Elevator m_elevator, Shooter m_shooter, LEDs m_leds, desiredHeight height) {
    addRequirements(m_elevator);
    addRequirements(m_shooter);
    addRequirements(m_leds);
    addCommands(
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_shooter.openBlocker()),
        Commands.runOnce(() -> m_elevator.SetHeight(height)),
        new WaitCommand(1.5),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
