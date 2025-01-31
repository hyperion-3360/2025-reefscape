package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Dumper extends SubsystemBase {
  private Servo leftServo = new Servo(Constants.SubsystemInfo.kCoralDumperLeftServoID);
  private Servo rightServo = new Servo(Constants.SubsystemInfo.kCoralDumperRightServoID);

  public Dumper() {}

  public void dumpRight() {
    rightServo.setAngle(60);
  }

  public void dumpLeft() {
    leftServo.setAngle(60);
  }
}
