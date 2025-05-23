package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.shooting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class AutoCancel extends SequentialCommandGroup {
  public AutoCancel(
      Elevator m_elevator, Shooter m_shooter, LEDs m_leds, AlgaeIntake m_algaeIntake) {
    addCommands(
        // Commands.runOnce(() -> m_shooter.setShoot(shootSpeed.STOP)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(shooting.STORED)),
        // Commands.runOnce(() -> m_shooter.closeBlocker()),
        // Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
