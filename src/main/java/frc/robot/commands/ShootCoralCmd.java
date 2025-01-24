// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

// spotless:off
/**
 * TELEOP AND AUTO COMMAND
 * 
 * this command shoots the game objects to different settings :
 * AlgaeProcessor :
 *    the algae is shot into the processor, teleop
 * AlgaeNet :
 *    the algae is shot in the net, teleop
 *    * this command is unconfirmed and might be deleted
 * AlgaeDitch :
 *    the algae is shot out of the robot in case of emergency, teleop
 * CoralAuto :
 *    that one time where we quickly drop off the coral in L1 while passing by, auto
 * CoralL1 :
 *    shoots coral in L1, teleop (and auto if needed)
 * CoralL2 :
 *    shoots coral in L2, teleop (and auto if needed)
 * CoralL3 : 
 *    shoots coral in L3, teleop (and auto if needed)
 * CoralL4 :
 *    shoots coral in L4, teleop and auto
 */
// spotless:on

public class ShootCoralCmd extends Command {

  // there might be more things to add (like maybe a coral vomit if needed)
  public enum ShootCoralType {
    CoralAuto,
    CoralL1,
    CoralL2,
    CoralL3,
    CoralL4
  }

  private ShootCoralType currentCoralShootType = null;
  private Shooter m_shooter = null;

  private boolean isFinished = false;

  public ShootCoralCmd(ShootCoralType type, Shooter theShooter) {

    currentCoralShootType = type;
    m_shooter = theShooter;
    // the lack of break is INTENTIONAL :
    // it is because in 3 cases i want the same result :)

    switch (currentCoralShootType) {
      case CoralAuto:
        break;

      case CoralL1:
        // RobotContainer.m_elevator.SetHeight(desiredHeight.L1);
        break;

      case CoralL2:
        // RobotContainer.m_elevator.SetHeight(desiredHeight.L2);
        break;

      case CoralL3:
        // RobotContainer.m_elevator.SetHeight(desiredHeight.L3);
        break;

      case CoralL4:
        // RobotContainer.m_elevator.SetHeight(desiredHeight.L4);
        break;

      default:
        break;
    }

    addRequirements(m_shooter);
    // addRequirements(RobotContainer.m_elevator);
  }

  // Called when the command is initially scheduled.
  // you can set a led setting here to indicate what is happening
  @Override
  public void initialize() {
    /**
     * this method should set the height of the elevator to the desired lvl, with the help of a
     * switch case (refer to IntakeCmd)
     */
    switch (currentCoralShootType) {
      case CoralAuto:
        /** this should put the wrist angle at shoot position */
        break;

      case CoralL1:
        /** this should set the elevator height to the desired height */
        // RobotContainer.m_elevator.SetHeight(desiredHeight.L1);
        ;
        break;

      case CoralL2:
        /** this should set the elevator height to the desired height */
        // RobotContainer.m_elevator.SetHeight(desiredHeight.L2);
        break;

      case CoralL3:
        /** this should set the elevator height to the desired height */
        // RobotContainer.m_elevator.SetHeight(desiredHeight.L3);
        break;

      case CoralL4:
        /** this should set the elevator height to the desired height */
        // RobotContainer.m_elevator.SetHeight(desiredHeight.L4);
        break;

      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * this method should use vision to align the robot to the right place depending on the
     * shootType, as well as activate the wheels at the right speed, until end condition is met
     * (e.g. limitswitch, beambreak, etc)
     *
     * <p>here we will have to determine if we are in auto or not, eggpecially with the coral L1,
     * L2, L3 and L4. this is because it will determine with vision when to release the coral into
     * the peg, instead of waiting for the button input from the controller. (refer to IntakeCmd and
     * previous method) example below to verify if is in auto or not
     */
    if (DriverStation.isAutonomous()) {
    } else {
    }
  }

  // Called once the command ends or is interrupted.
  // this will most likely also be used to change the leds back to idle
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
