// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

// spotless:off
/**
 * AUTO AND TELEOP COMMAND 
 *
 * the 4 intaking methods are : 
 * CoralAuto : "ground" coral intake, in auto 
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
public class IntakeCmd extends Command {

  private boolean isFinished = false;

  public enum IntakeType {
    AlgaeGround,
    AlgaeAuto,
    CoralAuto,
    CoralFeeder
  }

  private Subsystem subsystem = null;
  private IntakeType currentIntakeType = null;

  public IntakeCmd(IntakeType type) {
    // Use addRequirements() here to declare subsystem dependencies.
    currentIntakeType = type;

    switch (currentIntakeType) {
      // the lack of break is INTENTIONAL :
      // it is because in both cases i want the same result :)
      case AlgaeAuto:
      case AlgaeGround:
        this.subsystem = RobotContainer.m_algaeIntake;
        break;
      case CoralAuto:
        this.subsystem = RobotContainer.m_coralClaw;
        break;

      default:
        this.subsystem = null;
        break;
    }

    addRequirements(subsystem);
    addRequirements(RobotContainer.m_elevator);
  }

  // Called when the command is initially scheduled.
  // you can set a led setting here to indicate what is happening
  @Override
  public void initialize() {
    switch (currentIntakeType) {
      case AlgaeAuto:

      
        /**
         * this should : 1. runOnce place the AlgaeIntake in auto intake lvl | 2. lift the elevator
         * to leave space for the algae to go in (not sure, we'll see when the robot is built)
         */
        break;

      case AlgaeGround:
        /**
         * this should : 1. runOnce place the algae intake in teleop intake lvl | 2. lift the
         * elevator to leave space for the algae to go in (not sure, we'll see when the robot is
         * built)
         */
        break;
      case CoralAuto:

        /**
         * this should : 1. runOnce place the coral intake in intake angles | 2. place the elevator
         * in handoff position
         */
        break;
      case CoralFeeder:
        /** this should : 1. lift the elevator to feeder height */
        break;

      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentIntakeType) {
      case AlgaeAuto:

        /**
         * this should : 1. run wheel intake speed until limit switch is true example below
         * RobotContainer.m_algaeIntake.run(null).until(null).andThen(() -> isFinished = true);
         */
        break;
      case AlgaeGround:
        /** this should : 1. run wheel intake speed until limit switch is true */
        break;
      case CoralAuto:
        /**
         * this should : 1. spin the coral intake wheels until limitshwitch | 2. lift coral intake
         * to handoff angle | 3. spin coral feeder intake wheels to intake speed until beambreak 4.
         * isFinished = true
         */
        break;
      case CoralFeeder:
        /**
         * this should : 1. spin wheels intake lvl until beambreak | 2. (spin wheels if needed to
         * keep the coral in intake) | 3. (lower elevator to travel height if needed) | 4.
         * isFinished = true
         */
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
    switch (currentIntakeType) {
      case AlgaeAuto:
        break;

      case AlgaeGround:
        break;

      case CoralAuto:
        break;

      case CoralFeeder:
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
