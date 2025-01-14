// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemInfo;

/** 1 falcon winch and unwinch* */
// La classe devrait avoir un option pour seulement s'activer a 30sec de la fin du jeu,
// pour eviter un mouvement par accident du pilot
public class Climber extends SubsystemBase {

  private WPI_TalonSRX m_climberMotor = new WPI_TalonSRX(SubsystemInfo.kClimberMotorID);
  private double GrabSpeed = -0.25;
  private double LiftSpeed = 0.25;

  public enum climberAction {
    GRAB,
    LIFT
  }

  public Climber() {
    // How do you fully reset a motor to ensure start position?
    m_climberMotor.set(0);
  }

  // create falcons motor control
  // need function to move with grab/lift
  // pour les falcons utilise talon ex: private WPI_TalonSRX m_exemple = new
  // WPI_TalonSRX(kidexemple);
  public void move(climberAction action) {
    // Take a direction from climberAction enum
    if (DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30) {
      switch (action) {
        case GRAB:
          // Set Falcon to go down
          m_climberMotor.set(GrabSpeed);
          break;

        case LIFT:
          // set Falcon to go up
          m_climberMotor.set(LiftSpeed);
          break;
      }
    }
  }

  public void stop() {
    // stops the motors
    m_climberMotor.stopMotor();
  }

  @Override
  public void periodic() {}
}
