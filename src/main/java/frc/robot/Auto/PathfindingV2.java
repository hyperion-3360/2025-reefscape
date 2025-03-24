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
import frc.robot.commands.MinuteMoveCmd;
import frc.robot.commands.MinuteMoveCmd.*;
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

  private Alliance currentAlliance = Alliance.Blue;

  PathConstraints constraints = new PathConstraints(5.0, 4.0, 1, 2);

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

  public Pose2d offsetPose(Pose2d originalPose, double offset) {
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

    return new Pose2d(offsetTranslation, sameRotation);
  }

  public Pose2d offsetPose(Pose2d originalPose, double offset, boolean flip) {
    // Offset by 0.05 meters in the direction of the current angle
    double offsetX =
        originalPose.getTranslation().getX()
            + (offset * Math.cos(originalPose.getRotation().getRadians()));
    double offsetY =
        originalPose.getTranslation().getY()
            + (offset * Math.sin(originalPose.getRotation().getRadians()));
    var offsetRot = 0.0;
    if (flip) {
      offsetRot = originalPose.getRotation().getDegrees() - 180;
    } else {
      offsetRot = originalPose.getRotation().getDegrees();
    }

    // Create a new Pose2d with the offset translation and the same rotation
    Translation2d offsetTranslation = new Translation2d(offsetX, offsetY);
    Rotation2d rotation = new Rotation2d(Math.toRadians(offsetRot));

    return new Pose2d(offsetTranslation, rotation);
  }

  public boolean isCloseTo(Pose2d pose, double distance) {
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

  private Command driveAndIntakeAlgae(
      Pose2d targetPos, double elevatorRaiseDistance, desiredHeight algaeHeight) {
    SequentialCommandGroup algaeIntakeSequence = new SequentialCommandGroup(Commands.none());
    Pose2d backTrackedPose =
        new Pose2d(
            targetPos
                .getTranslation()
                .plus(new Translation2d(currentAlliance.equals(Alliance.Blue) ? 1 : -1, 0)),
            targetPos.getRotation());

    // sets the backtracked position depending on the alliance
    // Pose2d backTrackedPose =
    //     switch (currentAlliance) {
    //       case Blue:
    //         yield backTrackedPose =
    //             new Pose2d(
    //                 targetPos.getTranslation().plus(new Translation2d(1, 0)),
    //                 targetPos.getRotation());
    //       case Red:
    //         yield backTrackedPose =
    //             new Pose2d(
    //                 targetPos.getTranslation().minus(new Translation2d(1, 0)),
    //                 targetPos.getRotation());
    //               };

    algaeIntakeSequence.addCommands(
        // sets back the target pose so that we don't break the algae intake into a thousand pieces
        new InstantCommand(() -> m_swerve.drivetoTarget(backTrackedPose)),
        new WaitUntilCommand(() -> isCloseTo(targetPos, elevatorRaiseDistance)),
        new InstantCommand(() -> m_elevator.SetHeight(algaeHeight)),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.INTAKE)),
        new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.FLOOR)),
        new ParallelDeadlineGroup(
            new WaitCommand(0.5), new WaitUntilCommand(() -> m_swerve.targetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new InstantCommand(() -> m_swerve.drivetoTarget(targetPos)),
        new WaitUntilCommand(() -> m_algaeIntake.sensorTriggered()),
        new InstantCommand(() -> m_swerve.drivetoTarget(backTrackedPose)),
        new ParallelCommandGroup(
            new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.STORING)),
            new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.NET))),
        new ParallelDeadlineGroup(
            new WaitCommand(0.5), new WaitUntilCommand(() -> m_swerve.targetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()));

    return algaeIntakeSequence;
  }

  private Command driveAndShootNet(Pose2d targetPos, double elevatorRaiseDistance) {
    SequentialCommandGroup shootNetSequence = new SequentialCommandGroup(Commands.none());
    shootNetSequence.addCommands(
        new InstantCommand(() -> m_swerve.drivetoTarget(targetPos)),
        new WaitUntilCommand(() -> isCloseTo(targetPos, elevatorRaiseDistance)),
        new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.NET)),
        new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.NET)),
        new ParallelDeadlineGroup(
            new WaitCommand(0.5), new WaitUntilCommand(() -> m_swerve.targetReached())),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.INTAKE)),
        new WaitCommand(0.9),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.NET)),
        new WaitUntilCommand(() -> !m_algaeIntake.sensorTriggered()),
        new WaitCommand(0.4),
        new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.STORED)),
        new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.STORED)));
    return shootNetSequence;
  }

  //   private Command driveAndShootCycle(
  //       Pose2d targetPos,
  //       double elevatorRaiseDistance,
  //       desiredHeight elevatorHeight,
  //       double waitTimeToReach) {
  //     SequentialCommandGroup shootSequence = new SequentialCommandGroup(Commands.none());

  //     shootSequence.addCommands(
  //         // new InstantCommand(() -> m_swerve.drivetoTarget(offsetPose(targetPos, 1.5, false))),
  //         // new WaitUntilCommand(() -> isCloseTo(targetPos, elevatorRaiseDistance)),
  //         // new InstantCommand(() -> m_elevator.SetHeight(elevatorHeight)),
  //         // new InstantCommand(() -> m_shooter.openBlocker()),
  //         // new ParallelDeadlineGroup(
  //         //     new WaitCommand(waitTimeToReach), // will be elevatecmd(L4) later
  //         //     new WaitUntilCommand(() -> m_swerve.targetReached())),
  //         // // new WaitCommand(1.0), // will be elevatecmd(L4) later
  //         // new InstantCommand(() -> m_swerve.disableDriveToTarget()),
  //         new AlignPeg(
  //             m_swerve,
  //             m_elevator,
  //             m_shooter,
  //             m_algaeIntake,
  //             m_pegDetection,
  //             m_vision,
  //             targetPos,
  //             elevatorHeight),
  //         new WaitCommand(5),
  //         new InstantCommand(() -> m_shooter.setShoot(shootSpeed.L4AUTO)),
  //         new WaitCommand(0.4),
  //         new InstantCommand(() -> m_shooter.stop()),
  //         new InstantCommand(() -> m_shooter.closeBlocker()),
  //         Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.FEEDER)));
  //     return shootSequence;
  //   }

  private Command driveAndShootCycleFirst(Pose2d desiredpPose, desiredHeight elevatorHeight) {
    SequentialCommandGroup shootSequence = new SequentialCommandGroup(Commands.none());
    shootSequence.addCommands(
        new InstantCommand(() -> m_swerve.regularConstraints()),
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(2.7),
                    new SequentialCommandGroup(
                        Commands.runOnce(() -> m_swerve.drivetoTarget(desiredpPose)),
                        new WaitCommand(1),
                        Commands.runOnce(() -> m_elevator.SetHeight(elevatorHeight)),
                        Commands.runOnce(() -> m_shooter.openBlocker()),
                        new WaitUntilCommand(() -> m_swerve.targetReached()))),
                new InstantCommand(() -> m_swerve.disableDriveToTarget()),
                new ConditionalCommand(
                    new DeferredCommand(
                        () ->
                            new MinuteMoveCmd(
                                m_swerve,
                                0.5,
                                Math.abs(m_pegDetection.getOffset()),
                                (m_pegDetection.getOffset() < 0)
                                    ? OffsetDir.LEFT
                                    : OffsetDir.RIGHT),
                        getRequirements()),
                    new PrintCommand("Can't locate peg!!! "),
                    () -> {
                      if (m_pegDetection.processImage()) return true;
                      else return false;
                    })),
            new PrintCommand("hehe"),
            () -> m_vision.getLockID() != 0),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.L4AUTO)),
        new WaitUntilCommand(() -> !m_shooter.isCoralIn()),
        new WaitCommand(0.3),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.STOP)),
        new InstantCommand(() -> m_shooter.closeBlocker()),
        new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.FEEDER)));

    // ends command for peg correction

    return shootSequence;
  }

  private Command driveAndShootCycle(Pose2d desiredpPose, desiredHeight elevatorHeight) {
    SequentialCommandGroup shootSequence = new SequentialCommandGroup(Commands.none());
    shootSequence.addCommands(
        new InstantCommand(() -> m_swerve.boostedConstraints()),
        new InstantCommand(() -> m_swerve.drivetoTarget(offsetPose(desiredpPose, -0.5, false))),
        new WaitUntilCommand(() -> m_swerve.AutoTargetReached()),
        new InstantCommand(() -> m_swerve.lessenedConstraints()),
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(1.3),
                    Commands.runOnce(() -> m_swerve.drivetoTarget(desiredpPose)),
                    Commands.runOnce(() -> m_elevator.SetHeight(elevatorHeight)),
                    Commands.runOnce(() -> m_shooter.openBlocker()),
                    new WaitUntilCommand(() -> m_swerve.targetReached())),
                new InstantCommand(() -> m_swerve.disableDriveToTarget()),
                new ConditionalCommand(
                    new DeferredCommand(
                        () ->
                            new MinuteMoveCmd(
                                m_swerve,
                                0.5,
                                Math.abs(m_pegDetection.getOffset()),
                                (m_pegDetection.getOffset() < 0)
                                    ? OffsetDir.LEFT
                                    : OffsetDir.RIGHT),
                        getRequirements()),
                    new PrintCommand("Can't locate peg!!! "),
                    () -> {
                      if (m_pegDetection.processImage()) return true;
                      else return false;
                    })),
            new PrintCommand("hehe"),
            () -> m_vision.getLockID() != 0),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.L4AUTO)),
        new WaitUntilCommand(() -> !m_shooter.isCoralIn()),
        new WaitCommand(0.2),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.STOP)),
        new InstantCommand(() -> m_shooter.closeBlocker()),
        new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.FEEDER)));

    // ends command for peg correction

    return shootSequence;
  }

  private Command driveAndIntakeCycle(Pose2d targetPos) {
    var approachPose = offsetPose(targetPos, (robotLength / 2));
    SequentialCommandGroup intakeSequence = new SequentialCommandGroup(Commands.none());
    intakeSequence.addCommands(
        new InstantCommand(() -> m_swerve.drivetoTarget(approachPose)),
        new WaitUntilCommand(() -> isCloseTo(approachPose, 0.5)),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.INTAKE)),
        new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> m_shooter.isCoralIn()),
            new WaitUntilCommand(() -> m_swerve.targetReached())),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.STOP)));

    return intakeSequence;
  }

  public Command ThreeCoralRight() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());

    switch (currentAlliance) {
      case Blue:
        pathfindingSequence.addCommands(
            driveAndShootCycleFirst(
                AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchE, desiredHeight.L4),
            new InstantCommand(() -> m_swerve.ExtraBoostedConstraints()),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(12).get().toPose2d()),
            new InstantCommand(() -> m_swerve.ExtraBoostedConstraints()),
            driveAndShootCycle(
                AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchD, desiredHeight.L4),
            new InstantCommand(() -> m_swerve.ExtraBoostedConstraints()),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(12).get().toPose2d()),
            new InstantCommand(() -> m_swerve.ExtraBoostedConstraints()),
            driveAndShootCycle(
                AutoWaypoints.BlueAlliance.RightSide.pegWaypoints.branchC, desiredHeight.L4));
        break;
      case Red:
        pathfindingSequence.addCommands(
            new InstantCommand(() -> m_swerve.regularConstraints()),
            driveAndShootCycle(
                AutoWaypoints.RedAlliance.LeftSide.pegWaypoints.branchJ, desiredHeight.L4),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(2).get().toPose2d()),
            new InstantCommand(() -> m_swerve.boostedConstraints()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.RedAlliance.LeftSide.pegWaypoints.branchK, desiredHeight.L4)),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(2).get().toPose2d()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.RedAlliance.LeftSide.pegWaypoints.branchL, desiredHeight.L4)),
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
                AutoWaypoints.BlueAlliance.LeftSide.pegWaypoints.branchJ, desiredHeight.L4),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(13).get().toPose2d()),
            new InstantCommand(() -> m_swerve.lessenedConstraints()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.BlueAlliance.LeftSide.pegWaypoints.branchK, desiredHeight.L4)),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(13).get().toPose2d()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.BlueAlliance.LeftSide.pegWaypoints.branchL, desiredHeight.L4)),
            new InstantCommand(() -> m_swerve.regularConstraints()));

        break;
      case Red:
        pathfindingSequence.addCommands(
            new InstantCommand(() -> m_swerve.lessenedConstraints()),
            driveAndShootCycle(
                AutoWaypoints.RedAlliance.RightSide.pegWaypoints.branchE, desiredHeight.L4),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(1).get().toPose2d()),
            new InstantCommand(() -> m_swerve.boostedConstraints()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.RedAlliance.RightSide.pegWaypoints.branchD, desiredHeight.L4)),
            driveAndIntakeCycle(Constants.tagLayout.getTagPose(1).get().toPose2d()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // will be elevatecmd(L4) later
                    new InstantCommand(() -> m_shooter.stop())),
                driveAndShootCycle(
                    AutoWaypoints.RedAlliance.RightSide.pegWaypoints.branchC, desiredHeight.L4)),
            new InstantCommand(() -> m_swerve.regularConstraints()));
        break;
      default:
        break;
    }

    return pathfindingSequence;
  }

  public Command coralAndAlgae() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());
    switch (currentAlliance) {
      case Blue:
        pathfindingSequence.addCommands(
            new InstantCommand(() -> m_swerve.lessenedConstraints()),
            driveAndShootCycle(
                AutoWaypoints.BlueAlliance.LeftSide.pegWaypoints.branchH, desiredHeight.L4),
            new InstantCommand(() -> m_swerve.regularConstraints()),
            driveAndIntakeAlgae(
                AutoWaypoints.BlueAlliance.LeftSide.AlgaeWaypoint.AlgaeHG,
                0.7,
                desiredHeight.ALGAEL2),
            driveAndShootNet(AutoWaypoints.BlueAlliance.LeftSide.NetWaypoint.net, 0.4));

        break;

      case Red:
        pathfindingSequence.addCommands(
            new InstantCommand(() -> m_swerve.lessenedConstraints()),
            driveAndShootCycle(
                AutoWaypoints.RedAlliance.LeftSide.pegWaypoints.branchH, desiredHeight.L4),
            new InstantCommand(() -> m_swerve.regularConstraints()),
            driveAndIntakeAlgae(
                AutoWaypoints.RedAlliance.LeftSide.AlgaeWaypoint.AlgaeHG,
                0.7,
                desiredHeight.ALGAEL2),
            driveAndShootNet(AutoWaypoints.RedAlliance.LeftSide.NetWaypoint.net, 0.4));
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
