// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {

  public enum desiredHeight {
    HANDOFF,
    PROCESSOR,
    NET,
    FEEDER,
    ALGAELOW,
    LOW,
    L1,
    L2,
    L3,
    L4
  }

  // pid values

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private PIDController m_pid = new PIDController(kP, kI, kD);

  private TalonFXConfiguration m_rightMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_leftMotorConfig = new TalonFXConfiguration();
  private Follower m_follower = new Follower(Constants.SubsystemInfo.kRightElevatorMotorID, true);
  private TalonFX m_rightElevatorMotor =
      new TalonFX(Constants.SubsystemInfo.kRightElevatorMotorID, "CANivore_3360");
  private TalonFX m_leftElevatorMotor =
      new TalonFX(Constants.SubsystemInfo.kLeftElevatorMotorID, "CANivore_3360");

  private double m_elevatorTarget = Constants.ElevatorConstants.kElevatorDown;

  public Elevator() {

    // motor configs
    m_rightMotorConfig.MotorOutput.Inverted =
        Constants.ElevatorConstants.kRightElevatorMotorNotInverted;

    m_rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_rightMotorConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.ElevatorConstants.kElevatorMotorCurrentLimit;
    m_leftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_leftMotorConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.ElevatorConstants.kElevatorMotorCurrentLimit;

    m_rightElevatorMotor.getConfigurator().apply(m_rightMotorConfig);
    m_leftElevatorMotor.getConfigurator().apply(m_leftMotorConfig);
    m_leftElevatorMotor.setControl(m_follower);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      m_rightElevatorMotor.setPosition(0.0);
      m_leftElevatorMotor.setPosition(0.0);
      m_pid.reset();
      m_elevatorTarget = Constants.ElevatorConstants.kElevatorDown;
    } else {
      m_rightElevatorMotor.set(
          m_pid.calculate(m_rightElevatorMotor.getPosition().getValueAsDouble(), m_elevatorTarget));
    }
    SmartDashboard.putNumber("Target", m_elevatorTarget);
    SmartDashboard.putData("Elevator pid", m_pid);
    SmartDashboard.putNumber("Right motor encoder",m_rightElevatorMotor.getPosition().getValueAsDouble());
  }

  public void SetHeight(desiredHeight height) {
    // add things to move to desired height
    switch (height) {
      case LOW:
        m_elevatorTarget = Constants.ElevatorConstants.kElevatorDown;
        break;

      case L1:
        m_elevatorTarget = Constants.ElevatorConstants.kElevatorL1;
        break;

      case L2:
        m_elevatorTarget = Constants.ElevatorConstants.kElevatorL2;
        break;

      case L3:
        m_elevatorTarget = Constants.ElevatorConstants.kElevatorL3;
        break;

      case L4:
        m_elevatorTarget = Constants.ElevatorConstants.kElevatorL4;
        break;

      case PROCESSOR:
        m_elevatorTarget = Constants.ElevatorConstants.kElevatorProcessor;
        break;

      case NET:
        m_elevatorTarget = Constants.ElevatorConstants.kElevatorNet;
        break;

      case HANDOFF:
        m_elevatorTarget = Constants.ElevatorConstants.kElevatorHandoff;
        break;

      case ALGAELOW:
        m_elevatorTarget = Constants.ElevatorConstants.kElevatorAlgaeLow;
        break;

      case FEEDER:
        m_elevatorTarget = Constants.ElevatorConstants.kElevatorFeeder;
        break;
    }
  }

  public Command manualTest(DoubleSupplier up, DoubleSupplier down) {
    return run(
        () -> {
          System.out.println(String.format("%f : %f", up.getAsDouble(), down.getAsDouble()));
          if (up.getAsDouble() > 0.0) m_rightElevatorMotor.set(up.getAsDouble());
          else m_rightElevatorMotor.set(-down.getAsDouble());
        });
  }
}
