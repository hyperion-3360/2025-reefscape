package frc.robot.Auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.List;
import java.util.stream.Collectors;

public class PathfindingShuffleBoard extends Pathfinding {
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

    private CustomAuto(POI... poi) {
      this.desiredPOIs = poi;
    }

    public POI[] getPOIs() {
      return desiredPOIs;
    }
  }

  private static SendableChooser<List<POI>> autoChooser = new SendableChooser<>();
  private static SendableChooser<POI> POIAdder = new SendableChooser<>();
  private static SendableChooser<POI> POIRemover = new SendableChooser<>();
  private static String tabName = "Auto";
  private static ShuffleboardLayout autoLayout;
  private static String chosenPath = "";

  /**
   * calls the {@code createWidgetList} method that creates a list of possible predictable paths the
   * robot can take
   */
  public static void makeChooserWidget() {
    for (CustomAuto autos : CustomAuto.values()) {
      createWidgetList(autos);
    }
    Shuffleboard.getTab(tabName).getLayout("AutoBuilder");
    autoChooser.setDefaultOption(
        "Full Auto (every coordinates)", poiList); // if no options are chosen put every paths
    autoLayout.add(autoChooser);
    autoLayout.add(POIAdder); // adder
    autoLayout.add(POIRemover); // remover
    autoLayout.add("current path", chosenPath);
    logicHandler();
  }

  private static void createWidgetList(CustomAuto auto) {
    POI[] POIsToAdd = auto.getPOIs();
    autoChooser.addOption(
        auto.toString(), // gives the name
        poiList.stream() // poilist comes from Pathfinding
            .filter((poi) -> poi.toString() != POIsToAdd.toString()) // removes irrelevant POIs
            .collect(Collectors.toList()));
  }

  public static void createPOIListWidget() {
    for (POI poi : POI.values()) {
      POIAdder.addOption(poi.toString(), poi);
      POIRemover.addOption(poi.toString(), poi);
    }
  }

  private static void logicHandler() {
    // check at least once because we want to add the layout
    while (!DriverStation.isAutonomous() || !DriverStation.isTeleop()) {
      POIAdder.onChange(
          (poi) -> {
            chosenPath.concat(poi.toString());
            autoLayout.add(POIAdder);
          });
      POIRemover.onChange(
          (poi) -> {
            chosenPath.replace(poi.toString(), ""); // deletes the coordinate we don't want
            autoLayout.add(POIRemover);
          });
      autoLayout.add("current path", chosenPath);
    }
  }
}
