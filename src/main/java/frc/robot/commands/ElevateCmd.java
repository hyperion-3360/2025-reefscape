package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;

public class ElevateCmd extends SequentialCommandGroup {

  public ElevateCmd(Elevator m_elevator, LEDs m_leds, desiredHeight height) {
    addRequirements(m_elevator);
    addRequirements(m_leds);
    addCommands(
        Commands.runOnce(() -> m_leds.elevatingColor()),
        Commands.runOnce(() -> m_elevator.SetHeight(height)),
        new WaitUntilCommand(() -> m_elevator.isAtElevation(0.07)),
        m_leds.idleColor());
  }
}
