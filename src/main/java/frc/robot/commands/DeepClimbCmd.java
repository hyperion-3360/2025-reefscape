package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class DeepClimbCmd extends SequentialCommandGroup {

    DeepClimbCmd(Climber m_climber, LEDs m_leds){

        addRequirements(m_climber);
        addRequirements(m_leds);
        addCommands(
            Commands.runOnce(()-> m_leds.SetPattern(Pattern.DEEPCLIMB)),
            Commands.runOnce(() -> m_climber.winchDeepClimb()), 
            new WaitUntilCommand(() -> m_climber.SensorDetected()),
            Commands.runOnce(()-> m_climber.stopDeepClimb())
        );
    }
    
}
