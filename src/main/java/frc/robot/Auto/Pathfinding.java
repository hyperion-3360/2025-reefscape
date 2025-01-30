package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/**
 * Pathfinding this class functions by cycles. A cycle consists of:
 *
 * <p>1. The calling of the {@code doPathfinding()} method .
 *
 * <p>2. Checking of the closest and most profitable POI using the {@code FilterPOIs} method.
 *
 * <p>3. If the POI contains a list of possible Pose2d: give the closest one
 *
 * <p>4. Finally pathfind to the best POI and once you reach it: execute the method or sequence
 * attached to it
 *
 * <p>Basically this class acts like a big filter that funels the best POI given the constraints and
 * vomits that to the Pathfinder class
 */
public class Pathfinding extends Command {
  // #region POI enum logic
  public enum POI {
    ALGAE(
        Constants.AlgaeCoralStand.kStands,
        () -> Commands.runOnce(() -> System.out.println("Hello Algae")),
        Constants.Priorities.kIntakeCoral,
        true,
        () -> !Constants.Conditions.hasAlgae()),
    CORAL(
        Constants.AlgaeCoralStand.kStands,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kIntakeCoral,
        true,
        () -> !Constants.Conditions.hasCoral()),
    BRANCHES(
        Constants.Pegs.kPegs,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kShootCoralL4,
        true,
        () -> Constants.Conditions.hasCoral()),
    FEEDERS(
        Constants.Feeders.kFeeders,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kShootCoralL4,
        false,
        () -> Constants.Conditions.hasCoral()),
    PROCESSOR(
        10.0,
        5.3,
        180.0,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kShootingProcessor,
        () -> Constants.Conditions.hasAlgae()),
    NET(
        7.734,
        4,
        180.0,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kShootNet,
        () -> Constants.Conditions.hasAlgae()),
    DUMPINGUP(
        4.073906,
        4.745482,
        210,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kIntakeCoral,
        () -> Constants.Conditions.hasCoral()),
    DUMPINGDOWN(
        4.073906,
        3.306318,
        150,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kIntakeCoral,
        () -> Constants.Conditions.hasCoral());

    private Rotation2d angle;
    private Translation2d xy_coordinates;
    private Supplier<Command> event;
    private BooleanSupplier[] conditions;
    // a list that acts like a buffer accepting pose2ds, sorting them and assigning the closest one
    // to the point to pathfind to. Emptys out after each cyle to accept new arrays of points
    private ArrayList<Pose2d> positionsList = new ArrayList<>();

    // I need a list separate from positionsList because I need to keep track of all consumed pois
    // even after a cycle
    private List<Pose2d> consumedPOIs = new ArrayList<>();
    private boolean consumable;
    private int priority;

    /**
     * constructor stocking all the data we need per poi in variables
     *
     * @param x_coordinates the x position of the poi in (x,y)
     * @param y_coordinates the y position of the poi in (x,y)
     * @param angle the angle the robot should be facing once reaching the poi
     * @param event the sequence we want the robot to perform when arriving on point
     * @param removeCondition the condition to which we remove the poi from a list
     */
    private POI(
        double x_coordinates,
        double y_coordinates,
        double angle,
        Supplier<Command> event,
        int priority,
        BooleanSupplier... removeCondition) {
      this.event = event;
      this.conditions = removeCondition;
      this.angle = new Rotation2d(angle);
      this.xy_coordinates = new Translation2d(x_coordinates, y_coordinates);
      this.priority = priority;
    }

    /**
     * constructor overwritting the other one to allow input of a Translation2d object
     *
     * @param xy_coordinates the x and y position of the poi in (x,y)
     * @param angle the angle the robot should facing once reaching the poi
     * @param event the sequence we want the robot to perform when arriving on point
     * @param priority how bad we want to pathfind to this point
     * @param removeCondition the condition to which we remove the poi from a list
     */
    private POI(
        Translation2d xy_coordinates,
        double angle,
        Supplier<Command> event,
        int priority,
        BooleanSupplier... removeCondition) {
      this.xy_coordinates = xy_coordinates;
      this.angle = new Rotation2d(angle);
      this.conditions = removeCondition;
      this.event = event;
      this.priority = priority;
    }

    /**
     * constructor allowing the input of an array of points, then filters and sort to give the
     * closest one and stores that.
     *
     * @param xyThetacoordinates an array of (x, y, θ) coordinates we want to possibly pathfind to
     * @param event the sequence we want the robot to perform when arriving on point
     * @param priority how bad we want to pathfind to this point
     * @param consumable if we should remove the point we just pathfinded to.
     *     <p>If you have a singular POI that is consumable and thus can't use this param: make a
     *     isConsumed() condition and place it into the boolean supplier of another constructor
     * @param removeCondition the condition to which we remove the poi from a list
     */
    private POI(
        Pose2d[] xyThetacoordinates,
        Supplier<Command> event,
        int priority,
        boolean consumable,
        BooleanSupplier... removeCondition) {
      this.consumable = consumable;

      for (Pose2d coordinate : xyThetacoordinates) {
        this.positionsList.add(coordinate);
      }

      // gets the closest poi from the robot
      Pose2d pose =
          positionsList.stream()
              // filter pois we already went to
              .filter((poseHead) -> !consumedPOIs.contains(poseHead))
              .sorted(
                  (pose1, pose2) -> {
                    if (pose1
                            .getTranslation()
                            .getDistance(RobotContainer.m_swerve.getPose().getTranslation())
                        > pose2
                            .getTranslation()
                            .getDistance(RobotContainer.m_swerve.getPose().getTranslation())) {
                      return 1;
                    } else return 0;
                  })
              .findFirst()
              .get();

      if (this.consumable == true) {
        // removes point we already went to
        consumedPOIs.add(pose);
      }

      this.xy_coordinates = pose.getTranslation();
      this.angle = pose.getRotation();
      this.conditions = removeCondition;
      this.event = event;
      this.priority = priority;
      // clears the postions list so that we can accept new points
      positionsList.clear();
    }

    /**
     * @return the angle we want the robot to face when reaching this point
     */
    private Rotation2d getAngle() {
      return this.angle;
    }

    /**
     * @return the position of the robot when reaching the POI
     */
    private Translation2d getCoordinates() {
      return this.xy_coordinates;
    }

    private Command getEvent() {
      return this.event.get();
    }

    private int getPriority() {
      return this.priority;
    }

    private boolean getConditionStatus() {
      boolean allCondionsTrue = false;
      byte trueConditions = 0;
      for (BooleanSupplier condition : conditions) {
        if (condition.getAsBoolean() == true) {
          trueConditions += 1;
        }
        if (trueConditions == conditions.length) {
          allCondionsTrue = true;
        }
      }

      return allCondionsTrue;
    }
  }

  // #endregion

  enum CustomAuto {
    FIRSTAUTO(POI.FEEDERS, POI.BRANCHES),
    SECONDAUTO(POI.CORAL, POI.BRANCHES),
    THIRDAUTO(POI.CORAL, POI.BRANCHES),
    FOURTHAUTO(POI.CORAL, POI.BRANCHES),
    FIFTHAUTO(POI.PROCESSOR, POI.BRANCHES),
    SIXTHAUTO(POI.CORAL, POI.BRANCHES),
    SEVENTHAUTO(POI.CORAL, POI.BRANCHES),
    EIGHTHAUTO(POI.CORAL, POI.BRANCHES),
    NINTHUTO(POI.CORAL, POI.BRANCHES);

    private POI[] desiredPOIs;

    private CustomAuto(POI... poi) {
      this.desiredPOIs = poi;
    }

    public POI[] getPOIs() {
      return this.desiredPOIs;
    }
  }

  // #region variable declaration
  private static SendableChooser<List<POI>> autoChooser = new SendableChooser<>();
  private static SendableChooser<POI> POIAdder = new SendableChooser<>();
  private static SendableChooser<POI> POIRemover = new SendableChooser<>();
  private static String tabName = "Auto";
  // private static ShuffleboardLayout autoLayout =
  //  Shuffleboard.getTab(tabName).getLayout("AutoChooser", BuiltInLayouts.kList);
  private static String chosenPath = "";
  // private static GenericEntry pathEntry = autoLayout.add("path", chosenPath).getEntry();
  private static GenericEntry pathEntry =
      Shuffleboard.getTab(tabName).add("path", chosenPath).getEntry();

  protected static List<POI> poiList = new ArrayList<>();
  private static POI bestPOI;
  private static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  // #endregion

  /**
   * gives the value of the given POI based on multiple factors
   *
   * @param poi the poi to which we want to calculate the value
   * @return the reward in points per meter or another similar unit which must at leas involve
   *     meters and points
   */
  public static Double POIValue(POI poi) {
    double actionTime = 0.0;

    switch (poi) {
      case CORAL:
        actionTime = Constants.TimeToAction.kIntakeCoral;
        break;
      case BRANCHES:
        actionTime = Constants.TimeToAction.kShootCoralL4;
        break;
      case PROCESSOR:
        actionTime = Constants.TimeToAction.kShootAlgaeProcessor;
        break;
      case NET:
        actionTime = Constants.TimeToAction.kShootNet;
        break;
      case DUMPINGUP:
        break;
      case DUMPINGDOWN:
        break;
      case ALGAE:
        break;
      default:
        break;
    }
    double timeLeft =
        DriverStation.isFMSAttached()
            ? DriverStation.getMatchTime() - 145
            : 15 - DriverStation.getMatchTime(); // gets time left in auto no matter
    double timeDelta = timeLeft - actionTime;
    double distanceRobotToPoint =
        poi.getCoordinates().getDistance(RobotContainer.m_swerve.getPose().getTranslation());
    double pointRatio = (poi.getPriority() * timeDelta) / distanceRobotToPoint;
    if (DriverStation.isTest()) {
      System.out.println(pointRatio + " of " + poi.toString());
    }
    return pointRatio;
  }

  /**
   * filters a raw poi array and returns a pose2d object of the most advantageous point
   *
   * @param raw_poi a list of POIs to filter through
   * @return The {@link Pose2d} of the most advantageous point
   */
  private static List<PathPoint> FilterPOIs(List<POI> raw_poi) {
    List<PathPoint> path = new ArrayList<>();
    // filters and gets the best POI in the raw_poi list
    POI bestPOI =
        raw_poi.stream()
            // removes the POI we are already at
            .filter(poiHead -> Pathfinding.bestPOI != poiHead)
            .filter(poiToCheck -> poiToCheck.getConditionStatus() == true)
            .sorted((p1, p2) -> POIValue(p2).compareTo(POIValue(p1)))
            .findFirst()
            .get();
    // this is to prevent the rechecking of the best POI when trying to get it's event.
    Pathfinding.bestPOI = bestPOI;
    if (DriverStation.isTest()) {
      System.out.println("chosen poi " + bestPOI);
    }

    Pose2d optimisedPos = POICoordinatesOptimisation(bestPOI);
    Pose2d lineupPos = lineupPoint(optimisedPos);
    // spotless:off
    // TODO change the RotationTarget's param to match what we want because I'm not sure what the waypoint does
    // spotless:on
    path.add(
        new PathPoint(
            lineupPos.getTranslation(),
            new RotationTarget(lineupPos.getTranslation().getNorm(), lineupPos.getRotation())));
    path.add(
        new PathPoint(
            optimisedPos.getTranslation(),
            new RotationTarget(
                optimisedPos.getTranslation().getNorm(), optimisedPos.getRotation())));

    // uses the coordinates and angle of the first point
    return path;
  }

  private static List<PathPoint> convertToPathPoints(POI raw_poi) {
    List<PathPoint> path = new ArrayList<>();

    Pose2d optimisedPos = POICoordinatesOptimisation(raw_poi);
    Pose2d lineupPos = lineupPoint(optimisedPos);
    // spotless:off
    // TODO change the RotationTarget's param to match what we want because I'm not sure what the waypoint does
    // spotless:on
    path.add(
        new PathPoint(
            lineupPos.getTranslation(),
            new RotationTarget(lineupPos.getTranslation().getNorm(), lineupPos.getRotation())));
    path.add(
        new PathPoint(
            optimisedPos.getTranslation(),
            new RotationTarget(
                optimisedPos.getTranslation().getNorm(), optimisedPos.getRotation())));

    // uses the coordinates and angle of the first point
    return path;
  }

  private static Pose2d POICoordinatesOptimisation(POI poiToPathfind) {

    double robotLengthPlusBuffer = Constants.Swerve.robotLength * 1.01;
    double robotWidthPlusBuffer = Constants.Swerve.robotWidth * 1.01;
    double robotHyp = Math.hypot(robotLengthPlusBuffer, robotWidthPlusBuffer);
    Rotation2d rotation = poiToPathfind.getAngle();

    // calculates the coordinates to displace the robot actual wanted position relative to the POI
    Translation2d widthToBacktrack =
        new Translation2d(
            poiToPathfind.getCoordinates().getX() - robotHyp * poiToPathfind.getAngle().getCos(),
            poiToPathfind.getCoordinates().getY() - poiToPathfind.getAngle().getSin() * robotHyp);

    // TODO make this code more modular for the future
    if (poiToPathfind.equals(POI.ALGAE) || poiToPathfind.equals(POI.CORAL)) {
      // sets the rotation to the orientation of the vector robot  -> pose2D
      rotation =
          (RobotContainer.m_swerve.getRotation2d().getDegrees() <= 180)
              ? new Rotation2d(
                  180
                      - Math.atan(
                          poiToPathfind.getCoordinates().getX()
                              / poiToPathfind.getCoordinates().getY()))
              : new Rotation2d(
                  180
                      + Math.atan(
                          poiToPathfind.getCoordinates().getX()
                              / poiToPathfind.getCoordinates().getY()));
    }

    return new Pose2d(widthToBacktrack, rotation);
  }

  public static Pose2d lineupPoint(Pose2d poiToLineup) {

    return new Pose2d(
        poiToLineup.getTranslation().getX() - 0.1 * poiToLineup.getRotation().getCos(),
        poiToLineup.getTranslation().getY() - 0.1 * poiToLineup.getRotation().getSin(),
        poiToLineup.getRotation());
  }

  // #region Pathfinding Shuffleboard implementation
  /**
   * creates the chooser widget for the autonomous mode acts like a main shuffleboard method for the
   * Pathfinding class
   */
  public static void makeChooserWidget() {
    // adds the POIs in the enum
    for (POI poi : POI.values()) {
      poiList.add(poi);
    }

    for (CustomAuto autos : CustomAuto.values()) {
      createWidgetList(autos);
    }
    createPOIListWidget();
    autoChooser.setDefaultOption(
        "Full Auto (every coordinates)", poiList); // if no options are chosen put every coordinates
    SmartDashboard.putData(POIAdder);
    SmartDashboard.putData(POIRemover);
    SmartDashboard.putData(autoChooser);
    logicHandler();
  }

  private static void createWidgetList(CustomAuto auto) {
    autoChooser.addOption(
        auto.toString(), // gives the name
        // converts the POIs array into a list
        poiList.stream()
            .filter(
                // removes irrelevant POIs
                (poi) -> {
                  for (int i = 0; i < auto.getPOIs().length; i++) {
                    if (poi.equals(auto.getPOIs()[i])) return true;
                  }
                  return false;
                })
            .collect(Collectors.toList()));
  }

  private static void createPOIListWidget() {
    for (POI poi : POI.values()) {
      POIAdder.addOption(poi.toString(), poi);
      POIRemover.addOption(poi.toString(), poi);
    }
  }

  /**
   * this method reads a string returned by the shuffleboard auto chooser and returns the pois that
   * are contained withing that string
   *
   * @param inputPOI the string returned by the shuffleboard auto chooser
   * @return an array of POIs we want to visit
   */
  private static List<POI> tokenReader(String inputPOI) {

    if (inputPOI.equals("")) {
      // if the inputPOI is empty get default option
      autoChooser.getSelected().forEach((poi) -> chosenPath += " " + poi.toString());
      chosenPath.trim();
    } else {
      // this is so that we only use one tokenizer
      chosenPath = inputPOI;
    }

    ArrayList<POI> readPOIs = new ArrayList<>();
    // gives back the tokens to be read
    StringTokenizer token = new StringTokenizer(chosenPath);
    // verifies if a token matches a value in the enum
    while (token.hasMoreTokens()) {
      boolean foundToken = false;
      String currentString = token.nextToken();
      for (POI poi : POI.values()) {
        if (currentString.equals(poi.toString())) // removes the withespaces in the string
        {
          foundToken = true;
          readPOIs.add(poi);
          break;
        }
      }
      // checks if one of the POIs were not recognized
      try {
        assert foundToken == true;
      } catch (AssertionError e) {
        e.printStackTrace();
        System.out.println("error in POI reading no POI matched the value " + currentString);
      }
    }
    return readPOIs;
  }

  private static void logicHandler() {
    // plays the logic while the driverStation is disabled
    autoChooser.onChange(
        (auto) -> {
          chosenPath = "";
          for (POI poi : auto) {
            chosenPath += " " + poi.toString();
          }
          chosenPath = chosenPath.strip(); // removes the trailing whitespace at the beginning
        });
    POIAdder.onChange(
        (poi) -> {
          // checks if the poi is already contained within the chosenPath
          if (!chosenPath.contains(poi.toString())) {
            chosenPath = chosenPath.concat(" " + poi.toString()); // adds the coordinates we want
            pathEntry.setString(chosenPath);
          }
        });
    POIRemover.onChange(
        (poi) -> {
          chosenPath =
              chosenPath
                  // deletes the coordinate we don't want
                  .replace(poi.toString(), "")
                  .strip();
          pathEntry.setString(chosenPath);
        });
  }

  // adds the logicHandler to the execute loop
  @Override
  public void execute() {
    logicHandler();
  }

  // #endregion

  /**
   * executes the pathfinding command meaning that the robot should go to all chosen POIs
   *
   * @return the command to pathfind to a specified point
   */
  public static Command doPathfinding() {
    // once we have the POIs we want, we replace the old list and add them
    for (POI poiArrayElement : tokenReader(chosenPath)) {
      poiList.add(poiArrayElement);
    }

    return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathPoints(
                FilterPOIs(poiList), constraints, new GoalEndState(0, bestPOI.getAngle())),
            constraints)
        .andThen(bestPOI.getEvent())
        .repeatedly()
        .until(() -> DriverStation.isTeleop());
  }

  public static Command goThere(POI placeToGo) {
    return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathPoints(
                convertToPathPoints(placeToGo),
                constraints,
                new GoalEndState(0, placeToGo.getAngle())),
            constraints)
        .andThen(placeToGo.getEvent());
  }
}
