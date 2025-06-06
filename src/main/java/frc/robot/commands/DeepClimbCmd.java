package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class DeepClimbCmd extends SequentialCommandGroup {

  public DeepClimbCmd(Climber m_climber, LEDs m_leds) {

    addRequirements(m_climber);
    addRequirements(m_leds);
    addCommands(
        m_climber.goForthChild(),
        Commands.runOnce(() -> m_climber.stopDeepClimb()),
        new WaitCommand(0.5),
        Commands.runOnce(() -> m_climber.fingerClose()),
        Commands.runOnce(() -> m_climber.setClimberActivated()),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.DEEPCLIMB)),
        Commands.runOnce(() -> m_climber.winchDeepClimb()),
        new WaitUntilCommand(() -> m_climber.SensorDetected() || m_climber.reachedSafetyPosition()),
        new WaitCommand(0.2),
        Commands.runOnce(() -> m_climber.stopDeepClimb()),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.CLIMBER)));
  }
}
