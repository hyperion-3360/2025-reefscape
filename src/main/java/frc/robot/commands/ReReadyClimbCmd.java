package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

public class ReReadyClimbCmd extends SequentialCommandGroup {
  public ReReadyClimbCmd(Climber m_climber) {
    addRequirements(m_climber);
    addCommands(
        Commands.runOnce(() -> m_climber.fingerOpen()),
        // we need to winch a bit so that the finger can actually open
        Commands.run(() -> m_climber.winchDeepClimb()).withTimeout(0.2),
        // when the finger is removed we need to come back to the climbing setpoint
        m_climber.goForthChild(),
        Commands.runOnce(() -> m_climber.stopDeepClimb()),
        Commands.runOnce(() -> m_climber.setClimberActivated()));
  }
}
