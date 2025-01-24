package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeIntake;

public class ShootAlgaeCmd extends SequentialCommandGroup {
     AlgaeIntake m_algaeIntake;


     public ShootAlgaeCmd(AlgaeIntake m_algaeIntake, AlgaeIntake.elevation shootingAngle) {
        addCommands(
            Commands.run(
               () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE), m_algaeIntake),
               new WaitUntilCommand(0.1),
            Commands.run(
               () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.NET), m_algaeIntake),
               new WaitUntilCommand(1),
            Commands.run(
                () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED), m_algaeIntake));
}
}