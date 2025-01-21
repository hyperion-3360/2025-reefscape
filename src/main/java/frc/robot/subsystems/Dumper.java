
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dumper extends SubsystemBase {
    private Servo leftServo = new Servo(7);
    private Servo rightServo = new Servo(9);
    public Dumper() {}

    public void dumpRight() {
        rightServo.setAngle(60);
    }

    public void dumpLeft() {
        leftServo.setAngle(60);
}

}