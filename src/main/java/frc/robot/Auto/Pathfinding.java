package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoCmd.AutoBranchShooting;
import frc.robot.commands.AutoCmd.AutoDump;
import frc.robot.commands.AutoCmd.AutoFeed;
import frc.robot.commands.AutoCmd.AutoProcessor;
import frc.robot.commands.AutoCmd.AutoShootBranch;
import frc.robot.commands.AutoCmd.ShootAuto;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.StringTokenizer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.json.simple.parser.ParseException;

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
        false,
        () -> Commands.print("lol you are not used xaxaxa"),
        Constants.Priorities.kIntakeCoral,
        true,
        () -> !RobotContainer.m_algaeIntake.sensorTriggered()),
    BRANCHES(
        Constants.Pegs.kPegs,
        false,
        () -> m_autoShoot,
        Constants.Priorities.kShootCoralL4,
        true,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_A(
        Constants.Pegs.kPegs[0].getTranslation(),
        Constants.Pegs.kPegs[0].getRotation().getDegrees(),
        false,
        () -> m_autoShootA,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_B(
        Constants.Pegs.kPegs[1].getTranslation(),
        Constants.Pegs.kPegs[1].getRotation().getDegrees(),
        false,
        () -> m_autoShootB,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_C(
        Constants.Pegs.kPegs[2].getTranslation(),
        Constants.Pegs.kPegs[2].getRotation().getDegrees(),
        false,
        () -> m_autoShootC,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_D(
        Constants.Pegs.kPegs[3].getTranslation(),
        Constants.Pegs.kPegs[3].getRotation().getDegrees(),
        false,
        () -> m_autoShootD,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_E(
        Constants.Pegs.kPegs[4].getTranslation(),
        Constants.Pegs.kPegs[4].getRotation().getDegrees(),
        false,
        () -> m_autoShootE,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_F(
        Constants.Pegs.kPegs[5].getTranslation(),
        Constants.Pegs.kPegs[5].getRotation().getDegrees(),
        false,
        () -> m_autoShootF,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_G(
        Constants.Pegs.kPegs[6].getTranslation(),
        Constants.Pegs.kPegs[6].getRotation().getDegrees(),
        false,
        () -> m_autoShootG,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_H(
        Constants.Pegs.kPegs[7].getTranslation(),
        Constants.Pegs.kPegs[7].getRotation().getDegrees(),
        false,
        () -> m_autoShootH,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_I(
        Constants.Pegs.kPegs[8].getTranslation(),
        Constants.Pegs.kPegs[8].getRotation().getDegrees(),
        false,
        () -> m_autoShootI,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_J(
        Constants.Pegs.kPegs[9].getTranslation(),
        Constants.Pegs.kPegs[9].getRotation().getDegrees(),
        false,
        () -> m_autoShootJ,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_K(
        Constants.Pegs.kPegs[10].getTranslation(),
        Constants.Pegs.kPegs[10].getRotation().getDegrees(),
        false,
        () -> m_autoShootK,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    BRANCH_L(
        Constants.Pegs.kPegs[11].getTranslation(),
        Constants.Pegs.kPegs[11].getRotation().getDegrees(),
        false,
        () -> m_autoShootL,
        Constants.Priorities.kShootCoralL4,
        () -> RobotContainer.m_shooter.isCoralIn()),
    STARTINGBRANCH(
        Constants.Pegs.kPegs,
        false,
        () -> m_startingAutoShoot,
        Constants.Priorities.kShootCoralL4,
        true,
        () -> RobotContainer.m_shooter.isCoralIn()),
    FEEDERS(
        Constants.Feeders.kFeeders,
        true,
        () -> m_feeder,
        Constants.Priorities.kShootCoralL4,
        false,
        () -> !RobotContainer.m_shooter.isCoralIn()),
    PROCESSOR(
        5.9875,
        0,
        270.0,
        false,
        () -> processAlgae,
        Constants.Priorities.kShootingProcessor,
        () -> RobotContainer.m_algaeIntake.sensorTriggered()),
    NET(
        8.2722,
        6.1376,
        0.0,
        false,
        () -> Commands.runOnce(() -> System.out.println("Hello net")),
        Constants.Priorities.kShootNet,
        () -> RobotContainer.m_algaeIntake.sensorTriggered()),
    DUMPINGUP(
        7.0,
        3,
        180.0,
        true,
        () -> m_dump,
        Constants.Priorities.kIntakeCoral,
        () -> RobotContainer.m_shooter.isCoralIn()),
    DUMPINGDOWN(
        7.0,
        3,
        180.0,
        true,
        () -> m_dump,
        Constants.Priorities.kIntakeCoral,
        () -> RobotContainer.m_shooter.isCoralIn());

    private Supplier<Command> event;
    // a list that acts like a buffer accepting pose2ds, sorting them and assigning the closest one

    // I need a list separate from positionsList because I need to keep track of all consumed pois
    // even after a cycle
    private List<Pose2d> consumedPOIs = new ArrayList<>();
    private boolean consumable;
    private int priority;
    private Pose2d[] poseArray;
    private Pose2d pose;
    private boolean shouldFlipRobot;

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
        boolean shouldFlipRobot,
        Supplier<Command> event,
        int priority,
        BooleanSupplier... removeCondition) {
      Pose2d[] poseArray = {
        new Pose2d(x_coordinates, y_coordinates, Rotation2d.fromDegrees(angle))
      };
      this.event = event;
      this.shouldFlipRobot = shouldFlipRobot;
      this.poseArray = poseArray;
      this.priority = priority;
      changePose2d();
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
        boolean shouldFlipRobot,
        Supplier<Command> event,
        int priority,
        BooleanSupplier... removeCondition) {
      Pose2d[] poseArray = {new Pose2d(xy_coordinates, Rotation2d.fromDegrees(angle))};
      this.poseArray = poseArray;
      this.shouldFlipRobot = shouldFlipRobot;
      this.event = event;
      this.priority = priority;
      changePose2d();
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
        boolean shouldFlipRobot,
        Supplier<Command> event,
        int priority,
        boolean consumable,
        BooleanSupplier... removeCondition) {
      this.poseArray = xyThetacoordinates;
      this.shouldFlipRobot = shouldFlipRobot;
      this.consumable = consumable;
      this.event = event;
      this.priority = priority;
      changePose2d();
    }

    private Command getEvent() {
      return this.event.get();
    }

    private int getPriority() {
      return this.priority;
    }

    private Pose2d getPose2d() {
      return this.pose;
    }

    private void changePose2d() {

      ArrayList<Pose2d> positionsList = new ArrayList<>();

      for (Pose2d coordinate : poseArray) {
        positionsList.add(coordinate);
      }

      // gets the closest poi from the robot
      Pose2d pose =
          RobotContainer.m_swerve
              .getPose()
              .nearest(
                  positionsList.stream()
                      .filter(poseHead -> !consumedPOIs.contains(poseHead))
                      .toList());

      if (this.consumable == true) {
        // removes point we already went to
        consumedPOIs.add(pose);
      }
      this.pose = pose;
    }

    private boolean isRobotFlipped() {
      return this.shouldFlipRobot;
    }
  }

  // #endregion

  enum CustomAuto {
    FIVECORALAUTO(POI.STARTINGBRANCH, POI.FEEDERS, POI.BRANCH_A),
    ALGAE(POI.DUMPINGDOWN, POI.FEEDERS);

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
  private static PathConstraints constraints =
      new PathConstraints(4.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(360));

  private static AutoDump m_dump = new AutoDump(RobotContainer.m_dumper);
  private static AutoFeed m_feeder =
      new AutoFeed(RobotContainer.m_elevator, RobotContainer.m_shooter, RobotContainer.m_leds);
  private static AutoProcessor processAlgae =
      new AutoProcessor(
          RobotContainer.m_elevator, RobotContainer.m_algaeIntake, RobotContainer.m_leds);
  private static ShootAuto m_startingAutoShoot =
      new ShootAuto(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve);

  private static AutoBranchShooting m_autoShoot =
      new AutoBranchShooting(
          RobotContainer.m_swerve,
          RobotContainer.m_elevator,
          RobotContainer.m_shooter,
          RobotContainer.m_leds);

  private static AutoShootBranch m_autoShootA =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_A);
  private static AutoShootBranch m_autoShootB =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_B);
  private static AutoShootBranch m_autoShootC =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_C);
  private static AutoShootBranch m_autoShootD =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_D);
  private static AutoShootBranch m_autoShootE =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_E);
  private static AutoShootBranch m_autoShootF =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_F);
  private static AutoShootBranch m_autoShootG =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_G);
  private static AutoShootBranch m_autoShootH =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_H);
  private static AutoShootBranch m_autoShootI =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_I);
  private static AutoShootBranch m_autoShootJ =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_J);
  private static AutoShootBranch m_autoShootK =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_K);
  private static AutoShootBranch m_autoShootL =
      new AutoShootBranch(
          RobotContainer.m_shooter,
          RobotContainer.m_elevator,
          RobotContainer.m_leds,
          RobotContainer.m_swerve,
          POI.BRANCH_L);

  // #endregion

  private static Pose2d POICoordinatesOptimisation(POI poiToPathfind) {
    // adds a 1 percent buffer to the robot size
    double robotLengthPlusBuffer = (Constants.Swerve.robotLength / 2.0) * 1.01;
    double robotWidthPlusBuffer = (Constants.Swerve.robotWidth / 2.0) * 1.01;
    Rotation2d rotation =
        poiToPathfind.isRobotFlipped()
            ? Rotation2d.fromDegrees(poiToPathfind.getPose2d().getRotation().getDegrees())
            : Rotation2d.fromDegrees(poiToPathfind.getPose2d().getRotation().getDegrees() + 180);

    // calculates the coordinates to displace the robot actual wanted position relative to the POI
    Translation2d widthToBacktrack =
        new Translation2d(
            poiToPathfind.getPose2d().getX() + robotLengthPlusBuffer * rotation.getCos(),
            poiToPathfind.getPose2d().getY() + robotWidthPlusBuffer * rotation.getSin());

    return new Pose2d(
        widthToBacktrack,
        poiToPathfind.isRobotFlipped()
            ? poiToPathfind.getPose2d().getRotation().minus(Rotation2d.k180deg)
            : poiToPathfind.getPose2d().getRotation());
  }

  // #region Shuffleboard implementation
  /**
   * creates the chooser widget for the autonomous mode acts like a main shuffleboard method for the
   * Pathfinding class
   */
  public static void makeChooserWidget() {
    List<POI> poiListClone = poiList;
    // adds the POIs in the enum
    for (POI poi : POI.values()) {
      poiListClone.add(poi);
    }

    for (CustomAuto autos : CustomAuto.values()) {
      createWidgetList(autos);
    }
    createPOIListWidget();
    autoChooser.setDefaultOption(
        "Full Auto (every coordinates)",
        poiListClone); // if no options are chosen put every coordinates
    SmartDashboard.putData(POIAdder);
    SmartDashboard.putData(POIRemover);
    SmartDashboard.putData(autoChooser);
    logicHandler();
    poiList.clear();
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
      for (POI poi : POI.values()) {
        chosenPath += " " + poi.toString();
      }
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
    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
     readPOIs.forEach(poi -> {FlippingUtil.flipFieldPose(poi.getPose2d());}); 
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
    SmartDashboard.putNumber("target x", POICoordinatesOptimisation(poiList.remove(0)).getX());
    SmartDashboard.putNumber("target y", POICoordinatesOptimisation(poiList.remove(0)).getY());
  }

  // #endregion

  /**
   * A simple command to go to a specified POI in order to execute a command. This should only be
   * used during teleop
   *
   * @param placeToGo The poi to go to
   * @return A command to pathfind and execute the event
   */
  public static Command goThere(Supplier<POI> placeToGo) {
    return new DeferredCommand(
        () ->
            Commands.runOnce(() -> placeToGo.get().changePose2d())
                .andThen(
                    AutoBuilder.pathfindToPose(
                        POICoordinatesOptimisation(placeToGo.get()), constraints)),
        Set.of());
  }

  public static Command goThere(String pathName) {

    try {
      return AutoBuilder.pathfindThenFollowPath(
          PathPlannerPath.fromPathFile(pathName), constraints);
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
    System.out.println("no");
    return Commands.none();
  }

  /**
   * A simple command to go to a specified POI in order to execute a command. This should only be
   * used during teleop
   *
   * @param placeToGo The pose2d we want to go to
   * @return A command to pathfind to a specified point
   */
  public static Command goThere(Pose2d placeToGo) {
    Double[] pose2d = {placeToGo.getX(), placeToGo.getY(), placeToGo.getRotation().getDegrees()};
    SmartDashboard.putNumber("swerve pathfinding target x", pose2d[0]);
    SmartDashboard.putNumber("swerve pathfinding target y", pose2d[1]);
    SmartDashboard.putNumber("swerve pathfinding target theta", pose2d[2]);
    return AutoBuilder.pathfindToPose(placeToGo, constraints);
  }

  public static Command fullControl() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());
    // once we have the POIs we want, we replace the old list and add them
    for (POI poiArrayElement : tokenReader(chosenPath)) {
      poiList.add(poiArrayElement);
      pathfindingSequence.addCommands(poiArrayElement.getEvent());
    }

    return pathfindingSequence;
  }

  public static boolean isCloseToPOI(POI currentPOI) {
    return RobotContainer.m_swerve
            .getPose()
            .getTranslation()
            .getDistance(currentPOI.getPose2d().getTranslation())
        <= 0.4;
  }
}
