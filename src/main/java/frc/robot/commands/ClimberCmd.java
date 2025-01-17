// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

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

  private boolean isFinished = false;

  public enum ClimberType {
    CLIMBERGRAB,
    CLIMBERLIFT
  }

  private Subsystem subsystem = null;
  private ClimberType currentClimberType = null;

  public ClimberCmd(ClimberType type) {
    // Use addRequirements() here to declare subsystem dependencies.
    currentClimberType = type;

    switch (currentClimberType) {
      // the lack of break is INTENTIONAL :
      // it is because in both cases i want the same result :)
      case CLIMBERGRAB:
        this.subsystem = RobotContainer.m_climber;
        break;
      case CLIMBERLIFT:
        this.subsystem = RobotContainer.m_climber;
        break;

      default:
        this.subsystem = null;
        break;
    }

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  // you can set a led setting here to indicate what is happening
  @Override
  public void initialize() {
    switch (currentClimberType) {
      case CLIMBERGRAB:
        /** this should : 1. runOnce put the climber in grab position */
        break;

      case CLIMBERLIFT:
        /**
         * this should : 1. runOnce place the algae intake in teleop intake lvl | 2. lift the
         * elevator to leave space for the algae to go in (not sure, we'll see when the robot is
         * built)
         */
        break;

      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentClimberType) {
      case CLIMBERGRAB:

        /**
         * this should : 1. run wheel intake speed until limit switch is true example below
         * RobotContainer.m_algaeIntake.run(null).until(null).andThen(() -> isFinished = true);
         */
        break;
      case CLIMBERLIFT:
        /** this should : 1. run wheel intake speed until limit switch is true */
        break;

      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  // this method will most likely be in use to set the leds to the right colour once the command
  // ends
  @Override
  public void end(boolean interrupted) {
    switch (currentClimberType) {
      case CLIMBERGRAB:
        break;

      case CLIMBERLIFT:
        break;

      default:
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
