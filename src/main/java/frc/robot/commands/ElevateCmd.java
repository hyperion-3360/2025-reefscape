package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;

public class ElevateCmd extends SequentialCommandGroup{
    Elevator m_elevator;
    
    public Command ElevateCmd(desiredHeight height){
        addRequirements(m_elevator);
       return Commands.runOnce(()-> m_elevator.SetHeight(height));
    }
}
