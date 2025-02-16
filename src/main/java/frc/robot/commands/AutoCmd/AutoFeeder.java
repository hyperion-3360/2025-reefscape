package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;

public class AutoFeeder extends SequentialCommandGroup {
  public AutoFeeder(Elevator m_elevator, Shooter m_shooter, LEDs m_leds) {
    addCommands(new WaitUntilCommand(null));
  }
}
