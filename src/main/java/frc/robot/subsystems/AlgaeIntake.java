// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.annotation.Target;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
/** 3 neos 550 (mini-neo) one to go up and down and two to intake* */
public class AlgaeIntake extends SubsystemBase {

  public enum elevation {
    UP,
    FLOOR,
    MID,
    STORED
  }

  public enum shooting {
    INTAKE,
    PROCESSOR,
    NET
  }

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private PIDController m_pid = new PIDController(kP, kI, kD);

  
  private SparkMaxConfig m_intake_leftConfig = new SparkMaxConfig();
  private SparkMaxConfig m_intake_rightConfig = new SparkMaxConfig();
  private SparkMaxConfig m_directionConfig = new SparkMaxConfig();

  private SparkMax m_direction = new SparkMax (0, MotorType.kBrushless);
  private SparkMax m_intake_left = new SparkMax (0, MotorType.kBrushless);
  private SparkMax m_intake_right = new SparkMax (0, MotorType.kBrushless);

  private RelativeEncoder m_intake_encoder = m_intake_left.getEncoder();
  private AbsoluteEncoder m_direction_encoder = m_direction.getAbsoluteEncoder();
  
  private double m_AnglesTarget = Constants.AlgaeIntakeVariables.kStartingAngle;
  private double m_SpeedTarget = Constants.AlgaeIntakeVariables.kIntakeSpeed; 

public AlgaeIntake() {

    m_intake_leftConfig.follow(m_intake_right, true);

    m_intake_leftConfig.smartCurrentLimit(15);
    m_intake_rightConfig.smartCurrentLimit(15);
    m_directionConfig.smartCurrentLimit(15);

    m_intake_left.configure(m_intake_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_intake_right.configure(m_intake_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_direction.configure(m_directionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {


  }






  public void intake(shooting speed) {

    switch (speed) {

      case INTAKE:
      this.m_SpeedTarget = Constants.AlgaeIntakeVariables.kIntakeSpeed;
      break;

      case NET:
      this.m_SpeedTarget = Constants.AlgaeIntakeVariables.kNetSpeed;
      break;

      case PROCESSOR:
      this.m_SpeedTarget = Constants.AlgaeIntakeVariables.kProcessorSpeed;
      break;
    }
  }


// nom a changer
  public void direction(elevation angles) {

    switch(angles) {

      case UP:
      this.m_AnglesTarget = Constants.AlgaeIntakeVariables.kNetAngle;
      break;
      
      case FLOOR:
      this.m_AnglesTarget = Constants.AlgaeIntakeVariables.kFloorIntakeAngle;
      break;

      case MID:
      this.m_AnglesTarget = Constants.AlgaeIntakeVariables.kProcessorAngle;
      break;

      case STORED:
      this.m_AnglesTarget = Constants.AlgaeIntakeVariables.kStartingAngle;
      break;
    }

  }
}
