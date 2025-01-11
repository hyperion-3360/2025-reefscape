// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private boolean isAtDesiredHeight = false;

  // pid values

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private PIDController m_pid = new PIDController(kP, kI, kD);

  private TalonFXConfiguration m_rightMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_leftMotorConfig = new TalonFXConfiguration();
  private TalonFX m_rightElevatorMotor = new TalonFX(Constants.SubsystemInfo.kRightElevatorMotorID);
  private TalonFX m_leftElevatorMotor = new TalonFX(Constants.SubsystemInfo.kLeftElevatorMotorID);

  /** Creates a new Elevator. */
  public Elevator() {

    // motor configs
    //TODO: modify this according to needs
    m_rightMotorConfig.MotorOutput.Inverted = Constants.SubsystemInfo.kRightElevatorMotorInversion;
    m_leftMotorConfig.MotorOutput.Inverted = Constants.SubsystemInfo.kLeftElevatorMotorInversion;
    m_rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_rightMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.SubsystemInfo.kElevatorMotorCurrentLimit;

    // m_elevatorMotorConfig.ClosedLoopRamps = ; might need, but just a pid should be fine


    m_rightElevatorMotor.getConfigurator().apply(m_rightMotorConfig);
    m_rightElevatorMotor.getConfigurator().apply(m_leftMotorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetHeight(double desiredHeight) {
    // add things to move to desired height
    isAtDesiredHeight = true;
  }

  public boolean isAtDesiredHeight() {
    return isAtDesiredHeight;
  }
}