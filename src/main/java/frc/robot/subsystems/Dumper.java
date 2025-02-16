package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.TestBindings;
import frc.robot.Constants;

public class Dumper extends SubsystemBase implements TestBindings {
  private Servo leftServo = new Servo(Constants.SubsystemInfo.kCoralDumperLeftServoID);
  private Servo rightServo = new Servo(Constants.SubsystemInfo.kCoralDumperRightServoID);

  public Dumper() {}

  public void dumpRight() {
    rightServo.setAngle(55);
    System.out.println(rightServo.getAngle());
  }

  public void dumpLeft() {
    leftServo.setAngle(85);
    System.out.println(leftServo.getAngle());
  }

  public void closeDump() {
    leftServo.setAngle(55);
    rightServo.setAngle(80);
  }

  @Override
  public void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller) {
    moduleTrigger
        .and(controller.b())
        .onTrue(
            Commands.runOnce(
                () -> {
                  System.out.println("Dumping!!! ");
                  dumpRight();
                  dumpLeft();
                }));
  }
}
