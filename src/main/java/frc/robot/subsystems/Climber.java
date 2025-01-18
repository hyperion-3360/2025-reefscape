// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SubsystemInfo;
import frc.robot.Constants.climberAction;

/** 1 falcon winch and unwinch* */
// La classe devrait avoir un option pour seulement s'activer a 30sec de la fin du jeu,
// pour eviter un mouvement par accident du pilot
public class Climber extends SubsystemBase {

  private double kDt = ClimberConstants.kDt;
  private double kMaxVelocity = ClimberConstants.kMaxAcceleration;
  private double kMaxAcceleration = ClimberConstants.kMaxAcceleration;
  private double kP = ClimberConstants.kP;
  private double kI = ClimberConstants.kI;
  private double kD = ClimberConstants.kD;
  private double kS = ClimberConstants.kS;
  private double kG = ClimberConstants.kG;
  private double kV = ClimberConstants.kG;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private TalonFX m_climberMotor = new TalonFX(SubsystemInfo.kClimberMotorID);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(kS, kG, kV);

  private static double GrabPosition = 30;
  private static double LiftPosition = 0;
  private double m_climberTarget = LiftPosition;

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
    // if needed to block operation before end of game since climber only works once
    // if (DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30)
    switch (action) {
      case GRAB:
        // Set Falcon to go down
        m_climberTarget = GrabPosition;
        break;

      case LIFT:
        // set Falcon to go up
        m_climberTarget = LiftPosition;
        break;
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      // Make the motor get to the target
      m_climberMotor.set(
          m_controller.calculate(m_climberMotor.getPosition().getValueAsDouble(), m_climberTarget)
              + m_feedforward.calculate(
                  m_climberMotor.getPosition().getValueAsDouble(),
                  m_climberMotor.getVelocity().getValueAsDouble()));
    }
  }

  public Double getGrabTarget() {
    return GrabPosition;
  }

  public void setGrabTarget(Double value) {
    GrabPosition = value;
  }

  public Double getLiftTarget() {
    return LiftPosition;
  }

  public void setLiftTarget(Double value) {
    LiftPosition = value;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Climber");
    builder.addDoubleProperty("GrabTarget", this::getGrabTarget, this::setGrabTarget);
    builder.addDoubleProperty("LiftTarget", this::getLiftTarget, this::setLiftTarget);
  }
}
