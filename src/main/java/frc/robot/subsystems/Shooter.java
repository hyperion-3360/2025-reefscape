// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private WPI_TalonSRX m_shooter = new WPI_TalonSRX(Constants.SubsystemInfo.kCoralShooterTalonID);
  private Servo m_coralBlocker = new Servo(Constants.SubsystemInfo.kCoralShooterServoID);
  private DigitalInput m_shooterIR = new DigitalInput(Constants.SubsystemInfo.kCoralShooterBeambreakID);
  public boolean getShooterIR = m_shooterIR.get();

  /** 1 TalonFX controlling 2 BAGs and 1 Servo */
  public Shooter() {
    // Config (directly on the srx because srxconfig object is kinda limited)
    m_shooter.configFactoryDefault();
    m_shooter.setNeutralMode(Constants.CoralShooterConfig.kCoralShooterNeutralMode);
    m_shooter.configContinuousCurrentLimit(Constants.CoralShooterConfig.kCoralShooterCurrentLimit);
    m_shooter.enableCurrentLimit(true);
    
  }

  @Override
  public void periodic() {
    getShooterIR = m_shooterIR.get();
  }
}
