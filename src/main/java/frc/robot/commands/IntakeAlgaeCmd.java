// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;

public class IntakeAlgaeCmd extends SequentialCommandGroup {
  AlgaeIntake m_algaeIntake;

  // start the intake rollers and wait until the algae is in the intake
  // then reduce the speed of the intake rollers and lift the intake to a preset position
  // then stop the intake rollers when desired elevation is reached

  public IntakeAlgaeCmd(AlgaeIntake m_algaeIntake, AlgaeIntake.elevation angle) {
    addCommands(
        m_algaeIntake
            .shootAlgae(AlgaeIntake.shooting.INTAKE)
            .until(() -> m_algaeIntake.isAlgaeIn()),
        m_algaeIntake
            .shootAlgae(AlgaeIntake.shooting.STORING)
            .alongWith(m_algaeIntake.pivotAlgae(angle))
            .until(() -> m_algaeIntake.isAtAngle()),
        m_algaeIntake
            .shootAlgae(AlgaeIntake.shooting.STORED)
            .alongWith(m_algaeIntake.pivotAlgae(AlgaeIntake.elevation.STORED)));
  }

public Command cancelCommand() {
   return Commands.runOnce(() -> this.cancel());
}
}
