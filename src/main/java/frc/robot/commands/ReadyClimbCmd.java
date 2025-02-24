package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class ReadyClimbCmd extends SequentialCommandGroup{

    ReadyClimbCmd(Climber m_climber, LEDs m_leds){

        addRequirements(m_climber);
        addRequirements(m_leds);
        addCommands(
            Commands.runOnce(() -> m_leds.SetPattern(Pattern.DEEPCLIMB)),
            Commands.runOnce(() -> m_climber.raiseServo()),
            Commands.runOnce(() -> m_climber.dewinchDeepClimb()),
            new WaitUntilCommand(0.2), //idk this is a random value but we need to dewinch a bit to get ready
            Commands.runOnce(() -> m_climber.stopDeepClimb()),
            Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE))
            );
    }
    
}
