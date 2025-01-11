// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** 1 falcon winch and unwinch mini neo closes claws* */
// La classe devrait avoir un option pour seulement s'activer a 30sec de la fin du jeu,
// pour eviter un mouvement par accident du pilot
public class Climber extends SubsystemBase {
  public Climber() {}

  // pour les neo utiliser revlib et spark max ex: private CANSparkMax m_exemple = new
  // CANSparkMax(kidexemple);
  // pour les falcons utilise talon ex: private WPI_TalonSRX m_exemple = new
  // WPI_TalonSRX(kidexemple);

  @Override
  public void periodic() {}
}
