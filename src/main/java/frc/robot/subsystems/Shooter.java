// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Wait;

public class Shooter extends SubsystemBase {
  private WPI_TalonSRX m_shooter = new WPI_TalonSRX(Constants.SubsystemInfo.kCoralShooterTalonID);
  private Servo m_coralBlocker = new Servo(Constants.SubsystemInfo.kCoralShooterServoID);
  private DigitalInput m_shooterIR =
      new DigitalInput(Constants.SubsystemInfo.kCoralShooterBeambreakID);
  public boolean getShooterIR = m_shooterIR.get();
  public double CoralShooterSpeed = m_shooter.get();
  public double TestSpeed = 0.0;
  public double SpeedTestTime = 0.0;

  /** 1 TalonFX controlling 2 BAGs, 1 Servo and 1 beambreak */
  public Shooter() {
    // Srx config (directly on the srx because srxconfig object is kinda limited)
    m_shooter.configFactoryDefault();
    m_shooter.setNeutralMode(Constants.CoralShooterVariables.kCoralShooterNeutralMode);
    m_shooter.configContinuousCurrentLimit(
        Constants.CoralShooterVariables.kCoralShooterCurrentLimit);
    m_shooter.enableCurrentLimit(true);
    // TODO: Find good ramp rate
    m_shooter.configOpenloopRamp(Constants.CoralShooterVariables.kCoralShooterRamprate);

    // SmartDashboard
    SmartDashboard.putBoolean("Coral shooter has note", !getShooterIR);
    SmartDashboard.putNumber("Coral shooter speed", CoralShooterSpeed);
    SmartDashboard.putNumber("Shooter test speed", TestSpeed);
    SmartDashboard.putNumber("Shooter test time", SpeedTestTime);
  }

  public void shoot() {
    openBlocker();
    m_shooter.set(Constants.CoralShooterVariables.kShootSpeed);
    Wait.waitUntil(() -> getShooterIR);
    Wait.waitSecs(0.1);
    stop();
    closeBlocker();
  }

  public void intake() {
    closeBlocker();
    m_shooter.set(Constants.CoralShooterVariables.kIntakeSpeed);
    Wait.waitUntil(() -> !getShooterIR);
    stop();
  }

  public void testSpeed() {
    m_shooter.set(SmartDashboard.getNumber(getName(), CoralShooterSpeed));
    Wait.waitSecs(SmartDashboard.getNumber(getName(), SpeedTestTime));
    stop();
  }

  public void stop() {
    m_shooter.set(0);
  }

  public void openBlocker() {
    m_coralBlocker.setAngle(Constants.CoralShooterVariables.kCoralShooterOpen);
  }

  public void closeBlocker() {
    m_coralBlocker.setAngle(Constants.CoralShooterVariables.kCoralShooterClosed);
  }

  @Override
  public void periodic() {
    getShooterIR = m_shooterIR.get();
  }
}
