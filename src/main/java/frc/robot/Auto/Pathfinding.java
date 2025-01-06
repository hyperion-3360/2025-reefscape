package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/** Pathfinding */
public class Pathfinding {
  public enum POI {
    ALGAECORALSTAND(10.0,5.3, 180.0, null ,() -> true),
    BRANCHES(10.0,5.3, 180.0, null ,() -> true), //will add coordinate array later
    PROCESSOR(10.0,5.3, 180.0, null ,() -> true),
    NET(10.0,5.3, 180.0, null ,() -> true);

    private double x_coordinates;
    private double y_coordinates;
    private Rotation2d angle;
    private Translation2d xy_coordinates;
    private Supplier<Commands> events;
    private BooleanSupplier[] conditions;

    /**
     * constructor stocking all the data we need per poi in variables
     *
     * @param x_coordinates the x position of the poi in (x,y)
     * @param y_coordinates the y position of the poi in (x,y)
     * @param angle the angle the robot should be facing once reaching the poi
     * @param events the events we want to execute when reaching the point
     * @param removeCondition the condition to which we remove the poi from a list
     */
    private POI(
        double x_coordinates,
        double y_coordinates,
        double angle,
        Supplier<Commands> events,
        BooleanSupplier... removeCondition) {
      this.conditions = removeCondition;
      this.x_coordinates = x_coordinates;
      this.y_coordinates = y_coordinates;
      this.angle = new Rotation2d(angle);
      this.xy_coordinates = new Translation2d(this.x_coordinates, this.y_coordinates);
    }

    /**
     * constructor overwritting the other one to allow input of a Translation2d object
     *
     * @param xy_coordinates the x and y position of the poi in (x,y)
     * @param angle the angel the robot should facing once reaching the poi
     * @param events
     * @param removeCondition the condition to which we remove the poi from a list
     */
    private POI(Translation2d xy_coordinates, double angle, Supplier<Commands> event, BooleanSupplier... removeCondition) {
      this.conditions = removeCondition;
      this.angle = new Rotation2d(angle);
      this.xy_coordinates = xy_coordinates;
    }
/**
     * constructor overwritting the other one to allow input of a Translation2d object
     *
     * @param xy_coordinates the x and y position of the poi in (x,y)
     * @param angle the angel the robot should facing once reaching the poi
     * @param events
     * @param removeCondition the condition to which we remove the poi from a list
     */
    private POI(Translation2d[] xy_coordinates, double angle, Supplier<Commands> event, BooleanSupplier... removeCondition) {
    for (Translation2d coordinateInstance : xy_coordinates) {
      
    }
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
      return 0.0;
    }
  }

  private static LinkedList<POI> poiList = new LinkedList<>();

  private static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /**
   * filters a raw poi array and returns a pose2d object of the most advantageous point
   *
   * @param raw_poi a list of POIs to filter through
   * @return The {@link Pose2d} of the most advantageous point
   */
  private static Pose2d FilterPOIs(LinkedList<POI> raw_poi) {
    // filters raw poi data and collects the result into a list of pois sorted from most profitable
    // point to least
    List<POI> filtered_pois =
        raw_poi.stream()
            .filter(offending_poi -> offending_poi.getConditionStatus() == true)
            .sorted((p1, p2) -> p1.rewardFunction(p1).compareTo(p2.rewardFunction(p2)))
            .collect(Collectors.toList());

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
    Command commandUsed =
        AutoBuilder.pathfindToPose(FilterPOIs(poiList), constraints)
            .until(() -> DriverStation.isTeleop());

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      commandUsed =
          AutoBuilder.pathfindToPoseFlipped(FilterPOIs(poiList), constraints)
              .until(() -> DriverStation.isTeleop());
    }

    return commandUsed;
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
    Command commandUsed =
        AutoBuilder.pathfindToPose(FilterPOIs(poiList), constraints)
            .until(() -> DriverStation.isTeleop());

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      commandUsed =
          AutoBuilder.pathfindToPoseFlipped(FilterPOIs(poiList), constraints)
              .until(() -> DriverStation.isTeleop());
    }

    return commandUsed;
  }

  public static Pose2d getPose2d() {
    if (poiList.isEmpty()) {
      for (POI poiArrayElement : POI.values()) {
        poiList.add(poiArrayElement);
      }
    }

    return FilterPOIs(poiList);
  }
}

// this class could be in constants
final class ConditionBuilder {
  // example code to demonstrate how we could use functions to make our conditions
  public static boolean exampleCondition() {
    boolean conditionFullfilled = false;
    boolean somecondition = true;
    if (somecondition == true) {
      conditionFullfilled = true;
    }
    return conditionFullfilled;
  }
}