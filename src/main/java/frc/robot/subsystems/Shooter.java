// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Wait;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private WPI_TalonSRX m_shooter = new WPI_TalonSRX(Constants.SubsystemInfo.kCoralShooterTalonID);
  private Servo m_coralBlocker = new Servo(Constants.SubsystemInfo.kCoralShooterServoID);
  private DigitalInput m_shooterIR =
      new DigitalInput(Constants.SubsystemInfo.kCoralShooterBeambreakID);
      
  private boolean getShooterIR = m_shooterIR.get();
  private double CoralShooterSpeed = m_shooter.get();
  private double TestSpeed = 0.0;
  private double SpeedTestTime = 0.0;
  private double speed = 0.0;

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

  public void testSpeed() {
    m_shooter.set(SmartDashboard.getNumber(getName(), CoralShooterSpeed));
    Wait.waitSecs(SmartDashboard.getNumber(getName(), SpeedTestTime));
    stop();
  }

  public void stop() {
   speed = 0.0;
  }

  public Command openBlocker() {
    return this.runOnce(
        () -> m_coralBlocker.setAngle(Constants.CoralShooterVariables.kCoralShooterOpen));
  }

  public Command closeBlocker() {
    return this.runOnce(
        () -> m_coralBlocker.setAngle(Constants.CoralShooterVariables.kCoralShooterClosed));
  }

  @Override
  public void periodic() {
    getShooterIR = m_shooterIR.get();
    if (DriverStation.isDisabled()) {
      speed = 0.0;
    }
    m_shooter.set(speed);
  }

  /**
   * this method is used when manually testing the shooter using joysticks
   *
   * @param joystick the position of the joystick to influence motor speed
   * @return a command that runs the motor at a speed equivalent to the joystick position squared
   */
  public Command manualTest(DoubleSupplier joystick) {
    // the speed input is squard to have a smoother control curve

    return this.run(
        () -> {
          double throttle = joystick.getAsDouble();
          m_shooter.set(-1.0 * throttle * throttle);
        });
  }

  public void setShoot() {
 speed = -0.5;
  }

  public void setIntake() {
   speed = 0.5;
  }

  public boolean isCoralIn() {
    return getShooterIR;
  }
}
