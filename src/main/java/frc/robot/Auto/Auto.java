package frc.robot.Auto;

/** this class handles the shuffleboard implementation for the pathfinding commands */
public class Auto {

  private static Auto instance = null;

  /** Initializes the widgets for the auto mode */
  public static void initAutoWidget() {
    if (instance == null) {
      instance = new Auto();
    }
  }

  /** this is where you initalize the widgets you want to use in auto mode */
  private Auto() {
    Pathfinding.makeChooserWidget();
  }
}
