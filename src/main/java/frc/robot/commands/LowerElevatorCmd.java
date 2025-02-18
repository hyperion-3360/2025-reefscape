package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.shooting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class LowerElevatorCmd extends SequentialCommandGroup {
  public LowerElevatorCmd(
      Elevator m_elevator, LEDs m_leds, Shooter m_shooter, AlgaeIntake m_algaeIntake) {
    addRequirements(m_elevator);
    addRequirements(m_leds);
    addRequirements(m_shooter);
    addRequirements(m_algaeIntake);
    addCommands(
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(shooting.STORED)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_shooter.closeBlocker()),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.DONTPOUND))
            .unless(() -> !m_elevator.getTargetHeight().equals(desiredHeight.L4)),
        new WaitCommand(1.4)
            .unless(() -> !m_elevator.getTargetHeight().equals(desiredHeight.DONTPOUND)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        new WaitCommand(1.5),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
