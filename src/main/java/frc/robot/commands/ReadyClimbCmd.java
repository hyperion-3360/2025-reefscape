package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class ReadyClimbCmd extends SequentialCommandGroup {

  public ReadyClimbCmd(Climber m_climber, LEDs m_leds, AlgaeIntake m_algaeintake) {

    addRequirements(m_climber);
    addRequirements(m_leds);
    addRequirements(m_algaeintake);
    addCommands(
        Commands.runOnce(() -> m_algaeintake.setShootingAngle(AlgaeIntake.elevation.FLOOR)),
        new WaitCommand(.5),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.DEEPCLIMB)),
        Commands.runOnce(() -> m_climber.Penis90()),
        m_climber.goForthChild(),
        Commands.runOnce(() -> m_climber.stopDeepClimb()),
        // Commands.runOnce(() -> m_climber.fingerClose()),
        Commands.runOnce(() -> m_climber.setClimberActivated()));
  }
}
