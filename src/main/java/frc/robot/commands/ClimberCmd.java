// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

// spotless:off
/**
 * TELEOP COMMAND 
 *
 * the 1 climber methods is : 
 * ClimberMove : "ground" coral intake, in auto 
 * AlgaeAuto : "tower" algae intake, in auto
 * AlgaeGround : ground algae intake, in teleop 
 * CoralFeeder : feeder coral intake, in teleop
 *
 * this method uses a switch case to determine what type it is using and the necessary subsystems
 * along with it.
 *
 * the possible subsystems are : 
 * Algae intake 
 * Coral intake 
 * Coral feeder intake
 *
 * TODO: add stuff to complete this command
 */
// spotless:on
public class ClimberCmd extends Command {

  public enum ClimberType {
    CLIMBERGRAB,
    CLIMBERLIFT
  }

  private ClimberType currentClimberType = null;
  private Climber m_climber = null;

  public ClimberCmd(ClimberType type, Climber subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    currentClimberType = type;
    m_climber = subsystem;

    addRequirements(m_climber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentClimberType) {
      case CLIMBERGRAB:
        m_climber.move(Constants.climberAction.GRAB);
        break;
      case CLIMBERLIFT:
        m_climber.move(Constants.climberAction.LIFT);
        break;
      default:
        break;
    }
  }
}
