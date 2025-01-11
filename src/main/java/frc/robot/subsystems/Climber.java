// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
/** 1 falcon and 1 neo 550 (mini-neo) falcon winch and unwinch mini neo closes claws**/
public class Climber extends SubsystemBase {
  public Climber() {}
  //pour les neo utiliser revlib et spark max ex: private CANSparkMax m_exemple = new CANSparkMax(kidexemple);
  //pour les falcons utilise talon ex: private WPI_TalonSRX m_exemple = new WPI_TalonSRX(kidexemple);

  @Override
  public void periodic() {
  }

}