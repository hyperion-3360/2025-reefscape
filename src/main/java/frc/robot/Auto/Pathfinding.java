package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
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

/** Pathfinding */
public class Pathfinding extends Command {
  // #region POI enum logic
  public enum POI {
    ALGAECORALSTANDS(
        Constants.AlgaeCoralStand.kStands,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kIntakeCoral,
        () -> !Constants.Conditions.hasAlgae() && !Constants.Conditions.hasCoral()),
    BRANCHES(
        Constants.Pegs.kPegs,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kShootCoralL4,
        () -> Constants.Conditions.hasCoral()),
    FEEDERS(
        Constants.Feeders.kFeeders,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kShootCoralL4,
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
        () -> Constants.Conditions.hasAlgae());

    private Rotation2d angle;
    private Translation2d xy_coordinates;
    private Supplier<Command> event;
    private BooleanSupplier[] conditions;
    private ArrayList<Pose2d> positionsList = new ArrayList<>();
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
     * @param angle the angel the robot should facing once reaching the poi
     * @param event the sequence we want the robot to perform when arriving on point
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

    private POI(
        Pose2d[] xyThetacoordinates,
        Supplier<Command> event,
        int priority,
        BooleanSupplier... removeCondition) {
      for (Pose2d coordinate : xyThetacoordinates) {
        this.positionsList.add(coordinate);
      }
      // gets the closest peg from the robot
      Pose2d pose =
          positionsList.parallelStream()
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

      this.xy_coordinates = pose.getTranslation();
      this.angle = pose.getRotation();
      this.conditions = removeCondition;
      this.event = event;
      this.priority = priority;
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

    /**
     * generic function to override. If not overriden will always return 0.
     *
     * @param poi the poi to which we want to estimate the reward
     * @return the reward in points per meter or another similar unit which must involve points
     */
    public Double rewardFunction(POI poi) {
      double actionTime = 0.0;

      switch (poi) {
        case ALGAECORALSTANDS:
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
      return pointRatio;
    }
  }

  // #endregion

  enum CustomAuto {
    FIRSTAUTO(POI.FEEDERS, POI.BRANCHES),
    SECONDAUTO(POI.ALGAECORALSTANDS, POI.BRANCHES),
    THIRDAUTO(POI.ALGAECORALSTANDS, POI.BRANCHES),
    FOURTHAUTO(POI.ALGAECORALSTANDS, POI.BRANCHES),
    FIFTHAUTO(POI.PROCESSOR, POI.BRANCHES),
    SIXTHAUTO(POI.ALGAECORALSTANDS, POI.BRANCHES),
    SEVENTHAUTO(POI.ALGAECORALSTANDS, POI.BRANCHES),
    EIGHTHAUTO(POI.ALGAECORALSTANDS, POI.BRANCHES),
    NINTHUTO(POI.ALGAECORALSTANDS, POI.BRANCHES);

    private POI[] desiredPOIs;
    private List<POI> POIlist = new ArrayList<>();

    private CustomAuto(POI... poi) {
      this.desiredPOIs = poi;
    }

    public List<POI> getPOIs() {
      // clears the old POI list to accept new ones
      POIlist.clear();
      for (POI poi : desiredPOIs) {
        POIlist.add(poi);
      }
      return POIlist;
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
  private static List<POI> filtered_pois; // this is a collection of sorted poilist POI
  private static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  // #endregion
  /**
   * filters a raw poi array and returns a pose2d object of the most advantageous point
   *
   * @param raw_poi a list of POIs to filter through
   * @return The {@link Pose2d} of the most advantageous point
   */
  private static Pose2d FilterPOIs(List<POI> raw_poi) {

    // filters raw poi data and collects the result into a list of pois sorted from most profitable
    // point to least
    List<POI> filtered_pois =
        raw_poi.stream()
            .filter(offending_poi -> offending_poi.getConditionStatus() == true)
            .sorted((p1, p2) -> p1.rewardFunction(p1).compareTo(p2.rewardFunction(p2)))
            .collect(Collectors.toList());
    Pathfinding.filtered_pois = filtered_pois;
    // uses the coordinates and angle of the first point
    return new Pose2d(filtered_pois.get(0).getCoordinates(), filtered_pois.get(0).getAngle());
  }

  // #region Pathfinding Shuffleboard implementation
  /** creates the chooser widget for the autonomous mode acts for the Pathfinding class */
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
    // autoLayout.add(autoChooser);
    // autoLayout.add(POIAdder); // adder
    // autoLayout.add(POIRemover); // remover
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
            .filter((poi) -> auto.getPOIs().contains(poi)) // removes irrelevant POIs
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
  private static POI[] tokenReader(String inputPOI) {
    ArrayList<POI> readPOIs = new ArrayList<>();
    // gives back the tokens to be read
    StringTokenizer token = new StringTokenizer(inputPOI);
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
        System.out.println("error in POI reading no POI matched the value" + currentString);
      }
    }
    return (POI[]) readPOIs.toArray();
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
                  .replace(poi.toString(), "")
                  .strip(); // deletes the coordinate we don't want
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
    poiList.clear();
    for (POI poiArrayElement : tokenReader(chosenPath)) {
      poiList.add(poiArrayElement);
    }

    return AutoBuilder.pathfindToPose(FilterPOIs(poiList), constraints)
        .andThen(filtered_pois.get(0).getEvent())
        .repeatedly()
        .until(() -> DriverStation.isTeleop());
  }
}
