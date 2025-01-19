// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** 2 neo one controls elevation and one closes pinchers * */
public class CoralClaw extends SubsystemBase {
  public enum ClawState {
    OPEN,
    CLOSE
  }

  public enum ClawPosition {
    HANDOFF,
    INTAKE
  }

  // pour les neo utiliser revlib et spark max ex: private CANSparkMax m_exemple =
  // new
  // CANSparkMax(kidexemple);
  private SparkMax m_ElevationNeo = new SparkMax(Constants.SubsystemInfo.kCoralIntakeMotorElbowID, MotorType.kBrushless);
  private SparkMax m_PinchNeo = new SparkMax(Constants.SubsystemInfo.kCoralIntakeMotorClawID, MotorType.kBrushless);
  private SparkMaxConfig m_pinchNeoConfig = new SparkMaxConfig();
  private SparkMaxConfig m_elevationNeoConfig = new SparkMaxConfig();

  // pid configs
  private double kp = 0.1;
  private double ki = 0.0;
  private double kd = 0.0;
  private PIDController m_PID = new PIDController(kp, ki, kd);
  private double m_setpoint = Constants.CoralIntakeVariables.kIntakeAngle;
  private double m_setpointangle = Constants.CoralIntakeVariables.Closedposition;

  private DigitalInput m_beamBrake =
      new DigitalInput(Constants.SubsystemInfo.kCoralAutoIntakeBeamBrake);

  public CoralClaw() {
    m_elevationNeoConfig.openLoopRampRate(0.2);

    m_elevationNeoConfig.smartCurrentLimit(15);

    m_pinchNeoConfig.apply(m_elevationNeoConfig);

    m_ElevationNeo.configure(
        m_elevationNeoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_PinchNeo.configure(
        m_pinchNeoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command clawCommand(ClawState state, ClawPosition position) {
    // sets the pid if there are no recorded values take the default ones instead
    m_PID.setPID(
        SmartDashboard.getNumber("kp", kp),
        SmartDashboard.getNumber("ki", ki),
        SmartDashboard.getNumber("kd", kd));

    ClawAngle(state);
    ClawPosition(position);

    return this.run(
        () -> {
          m_ElevationNeo.set(
              m_PID.calculate(m_ElevationNeo.getAbsoluteEncoder().getPosition(), m_setpoint));
          m_PinchNeo.set(
              m_PID.calculate(m_PinchNeo.getAbsoluteEncoder().getPosition(), m_setpointangle));
        });
  }

  private void ClawAngle(ClawState state) {
    switch (state) {
      case OPEN:
        m_setpointangle = Constants.CoralIntakeVariables.Openposition;
        break;

      case CLOSE:
        m_setpointangle = Constants.CoralIntakeVariables.Closedposition;
        break;

      default:
        break;
    }
  }

  private void ClawPosition(ClawPosition position) {
    switch (position) {
      case HANDOFF:
        m_setpoint = Constants.CoralIntakeVariables.kHandoffAngle;

        break;
      case INTAKE:
        m_setpoint = Constants.CoralIntakeVariables.kIntakeAngle;
        break;

      default:
        break;
    }
  }

  /**
   * this checks if the coral claw has a coral
   *
   * @return the output of the beam break sensor false if the coral isn't there true if it is
   */
  public boolean hasCoral() {
    return !m_beamBrake.get();
  }

  @Override
  public void periodic() {
    // inputs and updates SmartDashBoard data
    SmartDashboard.putNumber("kp", kp);
    SmartDashboard.putNumber("ki", ki);
    SmartDashboard.putNumber("kd", kd);
    SmartDashboard.putNumber(
        "ElevationPosition", m_ElevationNeo.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("PinchPosition", m_PinchNeo.getAbsoluteEncoder().getPosition());
    SmartDashboard.putBoolean("beam break", hasCoral());
  }
}

