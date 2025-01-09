package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/** Pathfinding */
public class Pathfinding {
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
    PROCESSOR(
        10.0,
        5.3,
        180.0,
        () -> Commands.runOnce(() -> System.out.println("Hello World")),
        Constants.Priorities.kShootingProcessor,
        () -> Constants.Conditions.hasAlgae()),
    NET(
        10.0,
        5.3,
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

  private static LinkedList<POI> poiList = new LinkedList<>();
  private static List<POI> filtered_pois;
  private static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

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

  /**
   * executes the pathfinding command meaning that the robot should go to all chosen POIs
   *
   * @param poi a list of POIs that the robot should go through if conditions apply this param is
   *     there in case we want to prefilter pois we don't want
   * @return the command to pathfind to a specified point
   */
  public static Command doPathfinding(POI[] poi) {
    if (poiList.isEmpty()) {
      for (POI poiArrayElement : poi) {
        poiList.add(poiArrayElement);
      }
    }
    return AutoBuilder.pathfindToPose(FilterPOIs(poiList), constraints)
        .andThen(filtered_pois.get(0).getEvent())
        .repeatedly()
        .until(() -> DriverStation.isTeleop());
  }

  /**
   * executes the pathfinding command meaning that the robot should go to all chosen POIs
   *
   * @return the command to pathfind to a specified point
   */
  public static Command doPathfinding() {
    if (poiList.isEmpty()) {
      for (POI poiArrayElement : POI.values()) {
        poiList.add(poiArrayElement);
      }
    }
    return AutoBuilder.pathfindToPose(FilterPOIs(poiList), constraints)
        .andThen(filtered_pois.get(0).getEvent())
        .repeatedly()
        .until(() -> DriverStation.isTeleop());
  }
}
