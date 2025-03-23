package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.AlignPeg;
import frc.robot.commands.AlignPeg.Direction;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.AlgaeIntake.shooting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.PegDetect;
import frc.robot.vision.Vision;
import java.util.NoSuchElementException;
import java.util.function.Supplier;

public class PathfindingV2 extends Command {

  enum Alliance {
    Blue,
    Red
  }

  Shooter m_shooter;
  Elevator m_elevator;
  LEDs m_leds;
  Swerve m_swerve;
  AlgaeIntake m_algaeIntake;
  PegDetect m_pegDetection;
  Vision m_vision;
  public static final double robotLength = Units.inchesToMeters(35.0); // with bumper: 32.5
  public static final double robotWidth = Units.inchesToMeters(35.0); // with bumper: 32.5
  private final double distanceToBackTrack = 1.2;

  private Alliance currentAlliance = Alliance.Blue;

  PathConstraints constraints = new PathConstraints(6.0, 7.0, 1.5, 2.5);

  public PathfindingV2(
      Shooter shooter,
      Elevator elevator,
      LEDs leds,
      Swerve swerve,
      AlgaeIntake algaeIntake,
      PegDetect PegDetect,
      Vision vision) {
    m_shooter = shooter;
    m_elevator = elevator;
    m_leds = leds;
    m_swerve = swerve;
    m_algaeIntake = algaeIntake;
    m_pegDetection = PegDetect;
    m_vision = vision;

    addRequirements(m_shooter, m_elevator, m_leds, m_swerve, m_algaeIntake);

    try {
      if (DriverStation.getAlliance().get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
        currentAlliance = Alliance.Blue;
      } else if (DriverStation.getAlliance().get()
          == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
        currentAlliance = Alliance.Red;
      }
    } catch (NoSuchElementException e) {
      currentAlliance = Alliance.Blue;
    }
  }

  public Command goThere(Pose2d pose) {
    return goThere(pose, 0);
  }

  public Command goThere(Supplier<Pose2d> pose) {
    return goThere(pose.get(), 0);
  }

  public Command goThere(Pose2d pose, double goalEndVelocity) {
    return AutoBuilder.pathfindToPose(pose, constraints, goalEndVelocity);
  }

  /**
   * @param originalPose
   * @param offset
   * @param angleToSubtract angle to subtract in degrees
   * @return
   */
  public Pose2d offsetPose(Pose2d originalPose, double offset, double angleToSubtract) {
    // Offset by 0.05 meters in the direction of the current angle
    double offsetX =
        originalPose.getTranslation().getX()
            + offset * Math.cos(originalPose.getRotation().getRadians());
    double offsetY =
        originalPose.getTranslation().getY()
            + offset * Math.sin(originalPose.getRotation().getRadians());

    // Create a new Pose2d with the offset translation and the same rotation
    Translation2d offsetTranslation = new Translation2d(offsetX, offsetY);
    Rotation2d sameRotation = originalPose.getRotation();

    return new Pose2d(
        offsetTranslation,
        sameRotation.minus(new Rotation2d(Units.degreesToRadians((angleToSubtract)))));
  }

  public boolean isCloseTo(Pose2d pose, double distance) {
    System.out.println(
        "swerve pose: " + m_swerve.getPose().getX() + " " + m_swerve.getPose().getY());
    System.out.println("goal pose: " + pose.getX() + " " + pose.getY());
    return m_swerve.getPose().getTranslation().getDistance(pose.getTranslation()) < distance;
  }

  // private Command driveWaitAndShoot(Pose2d targetPos, double waitTime) {
  //   var approachPose = offsetPose(targetPos, -1);
  //   SequentialCommandGroup shootSequence = new SequentialCommandGroup(Commands.none());
  //   shootSequence.addCommands(
  //       new InstantCommand(() -> m_swerve.drivetoTarget(approachPose)),
  //       new WaitUntilCommand(() -> m_swerve.targetReached()),
  //       new InstantCommand(() -> m_swerve.disableDriveToTarget()),
  //       new WaitCommand(waitTime),
  //       new InstantCommand(() -> m_swerve.drivetoTarget(targetPos)),
  //       new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.L4)),
  //       new InstantCommand(() -> m_shooter.openBlocker()),
  //       new ParallelDeadlineGroup(
  //           new WaitCommand(1.0), // will be elevatecmd(L4) later
  //           new WaitUntilCommand(() -> m_swerve.targetReached())),
  //       new WaitCommand(1.0), // will be elevatecmd(L4) later
  //       new InstantCommand(() -> m_swerve.disableDriveToTarget()),
  //       new InstantCommand(() -> m_shooter.setShoot(shootSpeed.L4)),
  //       new WaitCommand(0.4),
  //       new InstantCommand(() -> m_shooter.stop()),
  //       new InstantCommand(() -> m_shooter.closeBlocker()),
  //       Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.FEEDER)));
  //   return shootSequence;
  // }

  private Command driveAndIntakeAlgaeAndSmoothNet(
      Pose3d pose,
      double elevatorRaiseDistance,
      desiredHeight algaeHeight,
      double reachBackTrackWait,
      double reachAlgaeWait,
      Alliance alliance) {
    // adjusts tag pose for robot length so that robot doesn't smash into the reef
    Pose2d targetPos =
        new Pose2d(
            pose.getX() + ((robotLength - 0.05) / 2 * Math.cos(pose.getRotation().getAngle())),
            pose.getY() + ((robotLength - 0.05) / 2 * Math.sin(pose.getRotation().getAngle())),
            Rotation2d.fromDegrees(pose.getRotation().toRotation2d().getDegrees() - 180));

    SequentialCommandGroup algaeIntakeSequence = new SequentialCommandGroup(Commands.none());

    Pose2d backTrackedPose =
        new Pose2d(
            targetPos.getX() + (0.45 * Math.cos(pose.getRotation().getAngle())),
            targetPos.getY() + (0.45 * Math.sin(pose.getRotation().getAngle())),
            Rotation2d.fromDegrees(pose.getRotation().toRotation2d().getDegrees() - 180));

    algaeIntakeSequence.addCommands(
        // sets back the target pose so that we don't break the algae intake into a thousand pieces
        new InstantCommand(() -> m_swerve.lessenedConstraints()),
        new InstantCommand(() -> m_swerve.drivetoTarget(backTrackedPose)),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.INTAKE)),
        new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.FLOOR)),
        new ParallelDeadlineGroup(
            new WaitCommand(reachBackTrackWait),
            new WaitUntilCommand(() -> isCloseTo(targetPos, elevatorRaiseDistance))),
        new InstantCommand(() -> m_elevator.SetHeight(algaeHeight)),
        new ParallelDeadlineGroup(
            new WaitCommand(reachAlgaeWait),
            new WaitUntilCommand(() -> m_swerve.AutoTargetReached())),
        new InstantCommand(() -> m_swerve.regularConstraints()),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new InstantCommand(() -> m_swerve.drivetoTarget(targetPos)),
        new WaitUntilCommand(() -> m_algaeIntake.sensorTriggered()),
        new InstantCommand(
            () ->
                m_swerve.drivetoTarget(
                    alliance.equals(Alliance.Red)
                        ? AutoWaypoints.RedAlliance.LeftSide.NetWaypoint.netSecondAlgae
                        : AutoWaypoints.BlueAlliance.LeftSide.NetWaypoint.netSecondAlgae)),
        new ParallelCommandGroup(
            new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.STORING)),
            new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.NET))),
        new ParallelDeadlineGroup(
            new WaitCommand(0.4), new WaitUntilCommand(() -> m_swerve.targetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()));

    return algaeIntakeSequence;
  }

  private Command driveAndIntakeAlgae(
      Pose3d pose,
      double elevatorRaiseDistance,
      desiredHeight algaeHeight,
      double reachBackTrackWait,
      double reachAlgaeWait) {
    // adjusts tag pose for robot length so that robot doesn't smash into the reef
    Pose2d targetPos =
        new Pose2d(
            pose.getX() + ((robotLength - 0.05) / 2 * Math.cos(pose.getRotation().getAngle())),
            pose.getY() + ((robotLength - 0.05) / 2 * Math.sin(pose.getRotation().getAngle())),
            Rotation2d.fromDegrees(pose.getRotation().toRotation2d().getDegrees() - 180));

    SequentialCommandGroup algaeIntakeSequence = new SequentialCommandGroup(Commands.none());

    Pose2d backTrackedPose =
        new Pose2d(
            targetPos.getX() + (0.45 * Math.cos(pose.getRotation().getAngle())),
            targetPos.getY() + (0.45 * Math.sin(pose.getRotation().getAngle())),
            Rotation2d.fromDegrees(pose.getRotation().toRotation2d().getDegrees() - 180));

    algaeIntakeSequence.addCommands(
        // sets back the target pose so that we don't break the algae intake into a thousand pieces
        new InstantCommand(() -> m_swerve.drivetoTarget(backTrackedPose)),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.INTAKE)),
        new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.FLOOR)),
        new ParallelDeadlineGroup(
            new WaitCommand(reachBackTrackWait),
            new WaitUntilCommand(() -> isCloseTo(targetPos, elevatorRaiseDistance))),
        new InstantCommand(() -> m_elevator.SetHeight(algaeHeight)),
        new ParallelDeadlineGroup(
            new WaitCommand(reachAlgaeWait),
            new WaitUntilCommand(() -> m_swerve.AutoTargetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new InstantCommand(() -> m_swerve.drivetoTarget(targetPos)),
        new WaitUntilCommand(() -> m_algaeIntake.sensorTriggered()),
        new InstantCommand(() -> m_swerve.drivetoTarget(backTrackedPose)),
        new ParallelCommandGroup(
            new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.STORING)),
            new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.NET))),
        new ParallelDeadlineGroup(
            new WaitCommand(0.4), new WaitUntilCommand(() -> m_swerve.targetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()));

    return algaeIntakeSequence;
  }

  private Command driveAndShootNet(
      Pose2d targetPos, double elevatorRaiseDistance, desiredHeight nextAlgaeHeight) {
    SequentialCommandGroup shootNetSequence = new SequentialCommandGroup(Commands.none());
    shootNetSequence.addCommands(
        new InstantCommand(() -> m_swerve.drivetoTarget(targetPos)),
        new WaitUntilCommand(() -> isCloseTo(targetPos, elevatorRaiseDistance)),
        new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.NET)),
        new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.NET)),
        new ParallelDeadlineGroup(
            new WaitCommand(0.6), new WaitUntilCommand(() -> m_swerve.targetReached())),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.INTAKE)),
        new WaitCommand(0.15),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.AUTONET)),
        new WaitUntilCommand(() -> !m_algaeIntake.sensorTriggered()),
        new WaitCommand(0.05),
        new InstantCommand(() -> m_elevator.SetHeight(nextAlgaeHeight)),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.STORED)),
        new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.STORED)));
    return shootNetSequence;
  }

  private Command driveAndShootCycle(
      Pose2d targetPos,
      double elevatorRaiseDistance,
      desiredHeight elevatorHeight,
      double waitTimeToReach,
      Direction direction) {
    SequentialCommandGroup shootSequence = new SequentialCommandGroup(Commands.none());

    shootSequence.addCommands(
        new InstantCommand(() -> m_swerve.drivetoTarget(offsetPose(targetPos, 1.5))),
        new WaitUntilCommand(() -> isCloseTo(targetPos, elevatorRaiseDistance)),
        new InstantCommand(() -> m_elevator.SetHeight(elevatorHeight)),
        new InstantCommand(() -> m_shooter.openBlocker()),
        new ParallelDeadlineGroup(
            new WaitCommand(waitTimeToReach), // will be elevatecmd(L4) later
            new WaitUntilCommand(() -> m_swerve.targetReached())),
        // new WaitCommand(1.0), // will be elevatecmd(L4) later
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new AlignPeg(
            m_swerve, m_elevator, m_shooter, m_algaeIntake, m_pegDetection, m_vision, direction),
        new WaitCommand(5),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.L4AUTO)),
        new WaitCommand(0.4),
        new InstantCommand(() -> m_shooter.stop()),
        new InstantCommand(() -> m_shooter.closeBlocker()),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.FEEDER)));
    return shootSequence;
  }

  private Command driveAndShootAndAlgae(
      Pose2d targetPos,
      double elevatorRaiseDistance,
      desiredHeight elevatorHeight,
      double waitTimeToReach) {
    SequentialCommandGroup shootSequence = new SequentialCommandGroup(Commands.none());

    shootSequence.addCommands(
        new InstantCommand(() -> m_swerve.drivetoTarget(targetPos)),
        new WaitUntilCommand(() -> isCloseTo(targetPos, elevatorRaiseDistance)),
        new InstantCommand(() -> m_swerve.regularConstraints()),
        new InstantCommand(() -> m_elevator.SetHeight(elevatorHeight)),
        new InstantCommand(() -> m_shooter.openBlocker()),
        new ParallelDeadlineGroup(
            new WaitCommand(waitTimeToReach), // will be elevatecmd(L4) later
            new WaitUntilCommand(() -> m_swerve.targetReached())),
        new ConditionalCommand(
            new DeferredCommand(
                () ->
                    new MinuteMoveCmd(
                        m_swerve,
                        0.8,
                        Math.abs(m_pegDetect.getOffset()),
                        (m_pegDetect.getOffset() < 0) ? OffsetDir.LEFT : OffsetDir.RIGHT),
                getRequirements()),
            new PrintCommand("Can't locate peg!!! "),
            () -> {
              if (m_pegDetect.processImage()
                  && m_elevator.getTargetHeight() == desiredHeight.L4
                  && !m_algaeIntake.pegBeamBreak()) return true;
              else return false;
            }),
        // new WaitCommand(1.0), // will be elevatecmd(L4) later
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.L4AUTO)),
        new WaitUntilCommand(() -> !m_shooter.isCoralIn()),
        new WaitCommand(0.1),
        new InstantCommand(() -> m_shooter.stop()),
        new InstantCommand(() -> m_swerve.regularConstraints()),
        new InstantCommand(() -> m_shooter.closeBlocker()),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.ALGAEL2)));
    return shootSequence;
  }

  private Command driveAndIntakeCycle(Pose2d targetPos) {
    var approachPose = offsetPose(targetPos, (robotLength / 2) - 0.12, 0);
    SequentialCommandGroup intakeSequence = new SequentialCommandGroup(Commands.none());
    intakeSequence.addCommands(
        new InstantCommand(() -> m_swerve.drivetoTarget(approachPose)),
        new WaitUntilCommand(() -> isCloseTo(approachPose, 0.5)),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.INTAKE)),
        new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> m_shooter.isCoralIn()),
            new WaitUntilCommand(() -> m_swerve.targetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()));

    return intakeSequence;
  }

  public Command ThreeCoralRight() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());

    switch (currentAlliance) {
      case Blue:
        pathfindingSequence.addCommands(
            new InstantCommand(() -> m_swerve.lessenedConstraints()),
            driveAndShootCycle(
                AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchE,
                1.0,
                desiredHeight.L4,
                1.2,
                Direction.left),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(12).get().toPose2d()),
            new InstantCommand(() -> m_swerve.boostedConstraints()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                // TODO: tweak time (last param)
                driveAndShootCycle(
                    AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchD,
                    0.2,
                    desiredHeight.L4,
                    1.2,
                    Direction.right)),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(12).get().toPose2d()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                // TODO: tweak time (last param)
                driveAndShootCycle(
                    AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchC,
                    0.2,
                    desiredHeight.L4,
                    1.2,
                    Direction.left)),
            new InstantCommand(() -> m_swerve.regularConstraints()));
        break;
      case Red:
        pathfindingSequence.addCommands(
            new InstantCommand(() -> m_swerve.regularConstraints()),
            driveAndShootCycle(
                AutoWaypoints.RedAlliance.LeftSide.pegWaypoints.branchJ,
                1.5,
                desiredHeight.L4,
                1.2,
                Direction.left),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(2).get().toPose2d()),
            new InstantCommand(() -> m_swerve.boostedConstraints()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.RedAlliance.LeftSide.pegWaypoints.branchK,
                    1.5,
                    desiredHeight.L4,
                    1.2,
                    Direction.right)),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(2).get().toPose2d()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.RedAlliance.LeftSide.pegWaypoints.branchL,
                    1.5,
                    desiredHeight.L4,
                    1.2,
                    Direction.left)),
            new InstantCommand(() -> m_swerve.regularConstraints()));
        break;
      default:
        break;
    }

    return pathfindingSequence;
  }

  public Command ThreeCoralLeft() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());

    switch (currentAlliance) {
      case Blue:
        pathfindingSequence.addCommands(
            new InstantCommand(() -> m_swerve.lessenedConstraints()),
            driveAndShootCycle(
                AutoWaypoints.BlueAlliance.LeftSide.pegWaypoints.branchJ,
                1.5,
                desiredHeight.L4,
                1.2,
                Direction.left),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(13).get().toPose2d()),
            new InstantCommand(() -> m_swerve.lessenedConstraints()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.BlueAlliance.LeftSide.pegWaypoints.branchK,
                    1.5,
                    desiredHeight.L4,
                    1.2,
                    Direction.right)),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(13).get().toPose2d()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.BlueAlliance.LeftSide.pegWaypoints.branchL,
                    1.5,
                    desiredHeight.L4,
                    1.2,
                    Direction.left)),
            new InstantCommand(() -> m_swerve.regularConstraints()));

        break;
      case Red:
        pathfindingSequence.addCommands(
            new InstantCommand(() -> m_swerve.lessenedConstraints()),
            driveAndShootCycle(
                AutoWaypoints.RedAlliance.RightSide.pegWaypoints.branchE,
                1.5,
                desiredHeight.L4,
                1.2,
                Direction.left),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(1).get().toPose2d()),
            new InstantCommand(() -> m_swerve.boostedConstraints()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.RedAlliance.RightSide.pegWaypoints.branchD,
                    1.5,
                    desiredHeight.L4,
                    1.2,
                    Direction.right)),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(1).get().toPose2d()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.RedAlliance.RightSide.pegWaypoints.branchC,
                    1.5,
                    desiredHeight.L4,
                    1.2,
                    Direction.left)),
            new InstantCommand(() -> m_swerve.regularConstraints()));
        break;
      default:
        break;
    }

    return pathfindingSequence;
  }

  public Command coralAndAlgae() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());
    Pose2d offsetedPositionBlue =
        offsetPose(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
                .getTagPose(22)
                .get()
                .toPose2d(),
            1.0,
            180);
    Pose2d offsetedPositionRed =
        offsetPose(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
                .getTagPose(9)
                .get()
                .toPose2d(),
            1.0,
            180);
    MinuteMoveCmd shuffleLeft = new MinuteMoveCmd(m_swerve, 0.5, 2.5, OffsetDir.LEFT);
    switch (currentAlliance) {
      case Blue:
        pathfindingSequence.addCommands(
            new InstantCommand(() -> m_swerve.slightlyBoostedConstraints()),
            driveAndShootAndAlgae(
                AutoWaypoints.BlueAlliance.LeftSide.pegWaypoints.branchH,
                2.5,
                desiredHeight.L4,
                1.2,
                Direction.left),
            new InstantCommand(() -> m_swerve.regularConstraints()),
            driveAndIntakeAlgae(
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
                    .getTagPose(21)
                    .get(),
                0.9,
                desiredHeight.ALGAEL2,
                0.5,
                0.8),
            new InstantCommand(() -> m_swerve.regularConstraints()),
            driveAndShootNet(
                AutoWaypoints.BlueAlliance.LeftSide.NetWaypoint.net, 0.4, desiredHeight.ALGAEL2),
            driveAndIntakeAlgaeAndSmoothNet(
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
                    .getTagPose(20)
                    .get(),
                0.01,
                desiredHeight.ALGAEL3,
                1.0,
                0.8,
                Alliance.Red),
            driveAndShootNet(
                AutoWaypoints.BlueAlliance.LeftSide.NetWaypoint.netSecondAlgae,
                0.3,
                desiredHeight.LOW),
            new InstantCommand(() -> m_swerve.boostedConstraints()),
            new InstantCommand(
                () ->
                    m_swerve.drivetoTarget(
                        new Pose2d(
                            offsetedPositionBlue.getX() + 1.8,
                            offsetedPositionBlue.getY() + 3.0,
                            offsetedPositionBlue.getRotation()))),
            new WaitUntilCommand(() -> m_swerve.targetReached()),
            new InstantCommand(() -> m_swerve.regularConstraints()),
            shuffleLeft
            // driveAndIntakeAlgae(
            //     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
            //         .getTagPose(22)
            //         .get(),
            //     0.1,
            //     desiredHeight.ALGAEL3,
            //     1.0,
            //     0.8),
            // shuffleRight,
            // driveAndShootNet(
            //     AutoWaypoints.BlueAlliance.LeftSide.NetWaypoint.net, 0.4, desiredHeight.LOW)
            // driveAndIntakeAlgae(
            //     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
            //         .getTagPose(22)
            //         .get(),
            //     0.7,
            //     desiredHeight.ALGAEL3),
            // driveAndShootNet(
            //     AutoWaypoints.BlueAlliance.LeftSide.NetWaypoint.net, 0.4, desiredHeight.LOW)
            );
        break;

      case Red:
        pathfindingSequence.addCommands(
            new InstantCommand(() -> m_swerve.slightlyBoostedConstraints()),
            driveAndShootAndAlgae(
                AutoWaypoints.RedAlliance.LeftSide.pegWaypoints.branchH,
                2.5,
                desiredHeight.L4,
                1.2,
                Direction.left),
            new InstantCommand(() -> m_swerve.regularConstraints()),
            driveAndShootNet(
                AutoWaypoints.RedAlliance.LeftSide.NetWaypoint.net, 0.4, desiredHeight.L2),
            driveAndIntakeAlgaeAndSmoothNet(
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
                    .getTagPose(11)
                    .get(),
                0.01,
                desiredHeight.ALGAEL3,
                1.2,
                0.8,
                Alliance.Red),
            driveAndShootNet(
                AutoWaypoints.RedAlliance.LeftSide.NetWaypoint.netSecondAlgae,
                0.3,
                desiredHeight.LOW),
            new InstantCommand(() -> m_swerve.boostedConstraints()),
            new InstantCommand(
                () ->
                    m_swerve.drivetoTarget(
                        new Pose2d(
                            offsetedPositionRed.getX() - 1.0,
                            offsetedPositionRed.getY() - 2.0,
                            offsetedPositionRed.getRotation()))),
            new WaitUntilCommand(() -> m_swerve.AutoTargetReached()),
            new InstantCommand(() -> m_swerve.regularConstraints()),
            shuffleLeft
            // driveAndIntakeAlgae(
            //     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
            //         .getTagPose(9)
            //         .get(),
            //     0.4,
            //     desiredHeight.ALGAEL3,
            //     0.5,
            //     0.8),
            // new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.ALGAEL2)),
            // new WaitCommand(0.2),
            // shuffleRight,
            // new ParallelCommandGroup(
            //     driveAndShootNet(
            //         AutoWaypoints.RedAlliance.LeftSide.NetWaypoint.netAngled,
            //         0.4,
            //         desiredHeight.LOW),
            //     new SequentialCommandGroup(
            //         new InstantCommand(() -> m_swerve.boostedConstraints()),
            //         new WaitCommand(0.6),
            //         new InstantCommand(() -> m_swerve.regularConstraints()),
            //         new WaitCommand(0.3),
            //         new InstantCommand(() -> m_swerve.lessenedConstraints())))
            // driveAndIntakeAlgae(
            //         AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).
            //             .getTagPose(9)
            //             .get(),
            //         0.7,
            //         desiredHeight.ALGAEL3),
            //     driveAndShootNet(AutoWaypoints.BlueAlliance.LeftSide.NetWaypoint.net, 0.4));
            );
        break;

      default:
        break;
    }
    return pathfindingSequence;
  }

  public Command straightLine() {
    var line = Commands.none();
    switch (currentAlliance) {
      case Blue:
        line =
            Commands.sequence(
                Commands.run(() -> m_swerve.drive(new Translation2d(-1, 0), 0, false, false)),
                new WaitCommand(1),
                Commands.run(() -> m_swerve.drive(new Translation2d(0, 0), 0, false, false)));
        break;
      case Red:
        line =
            Commands.sequence(
                Commands.run(() -> m_swerve.drive(new Translation2d(1, 0), 0, false, false)),
                new WaitCommand(1),
                Commands.run(() -> m_swerve.drive(new Translation2d(0, 0), 0, false, false)));

        break;
      default:
        break;
    }

    return line;
  }
}
