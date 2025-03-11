package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.SwerveElevatorSlowDownFunc;
import frc.lib.util.TestBindings;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.vision.Vision;
import java.io.File;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Swerve extends SubsystemBase implements TestBindings {
  public SwerveModule[] mSwerveMods;
  public SwerveModulePosition[] positions;
  private final Pigeon2 m_gyro;
  public final Field2d m_field2d = new Field2d();
  // public SwerveDriveOdometry m_odometry;
  private Vision vision;
  private final SwerveDrivePoseEstimator poseEstimator;
  Thread thread = new Thread();
  private boolean hasStartedEstimation = false;
  private Orchestra m_orchestra = new Orchestra();
  private boolean m_targetModeEnabled = false;
  private ProfiledPIDController m_xController;
  TrapezoidProfile.Constraints m_xConstraints;
  private ProfiledPIDController m_yController;
  TrapezoidProfile.Constraints m_yConstraints;
  private ProfiledPIDController m_rotController;
  TrapezoidProfile.Constraints m_rotConstraints;
  private final double kMaxSpeedMetersPerSecondX = 3.5;
  private final double kMaxAccelerationMetersPerSecondSquaredX = 2.0;
  private final double kMaxSpeedMetersPerSecondY = 3.5;
  private final double kMaxAccelerationMetersPerSecondSquaredY = 2.0;
  private final double kMaxSpeedRadiansPerSecond = 5.5;
  private final double kMaxAccelerationRadiansPerSecondSquared = 5.5;
  private final double kPTranslation = 6.0;
  private final double kPRot = 6.2;
  private Elevator m_elevator;

  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(Math.PI, Math.PI);

  // vision estimation of robot pose
  Optional<EstimatedRobotPose> visionEstLml3;
  Optional<EstimatedRobotPose> visionEstLml2R;
  Optional<EstimatedRobotPose> visionEstLml2L;

  public Swerve(Vision vision, Elevator elevator) {
    m_elevator = elevator;
    m_gyro = new Pigeon2(Constants.Swerve.kGyroCanId);

    m_gyro.reset();
    m_gyro.getAccumGyroZ(true);
    this.vision = vision;
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    for (SwerveModule mod : mSwerveMods) {
      m_orchestra.addInstrument(mod.getDriveMotor());
      m_orchestra.addInstrument(mod.getRotationMotor());
    }

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, getRotation2d(), getModulePositions(), new Pose2d());

    m_xController = new ProfiledPIDController(kPTranslation, 0, 0, m_xConstraints);
    m_yController = new ProfiledPIDController(kPTranslation, 0, 0, m_yConstraints);
    m_rotController = new ProfiledPIDController(kPRot, 0, 0, m_rotConstraints);
    m_rotController.enableContinuousInput(-Math.PI, Math.PI);

    configurePathPlanner();
  }

  public void boostedConstraints() {
    m_rotConstraints =
        new TrapezoidProfile.Constraints(
            kMaxSpeedRadiansPerSecond, kMaxAccelerationRadiansPerSecondSquared);
    m_yConstraints =
        new TrapezoidProfile.Constraints(
            kMaxSpeedMetersPerSecondY, kMaxAccelerationMetersPerSecondSquaredY);
    m_xConstraints =
        new TrapezoidProfile.Constraints(
            kMaxSpeedMetersPerSecondX, kMaxAccelerationMetersPerSecondSquaredX);

    m_xController.setConstraints(m_xConstraints);
    m_yController.setConstraints(m_yConstraints);
    m_rotController.setConstraints(m_rotConstraints);
  }

  public void regularConstraints() {
    m_rotConstraints =
        new TrapezoidProfile.Constraints(
            kMaxSpeedRadiansPerSecond, kMaxAccelerationRadiansPerSecondSquared);
    m_yConstraints =
        new TrapezoidProfile.Constraints(
            kMaxSpeedMetersPerSecondY + 0.5, kMaxAccelerationMetersPerSecondSquaredY + 1);
    m_xConstraints =
        new TrapezoidProfile.Constraints(
            kMaxSpeedMetersPerSecondX + 0.5, kMaxAccelerationMetersPerSecondSquaredX + 1);

    m_xController.setConstraints(m_xConstraints);
    m_yController.setConstraints(m_yConstraints);
    m_rotController.setConstraints(m_rotConstraints);
  }

  public void lessenedConstraints() {
    m_rotConstraints =
        new TrapezoidProfile.Constraints(
            kMaxSpeedRadiansPerSecond - 1.0, kMaxAccelerationRadiansPerSecondSquared - 1.5);
    m_yConstraints =
        new TrapezoidProfile.Constraints(
            kMaxSpeedMetersPerSecondY - 1.0, kMaxAccelerationMetersPerSecondSquaredY - 1.5);
    m_xConstraints =
        new TrapezoidProfile.Constraints(
            kMaxSpeedMetersPerSecondX - 1.0, kMaxAccelerationMetersPerSecondSquaredX - 1.0);

    m_xController.setConstraints(m_xConstraints);
    m_yController.setConstraints(m_yConstraints);
    m_rotController.setConstraints(m_rotConstraints);
  }

  public double getGyroZ() {
    return m_gyro.getAccumGyroZ().getValueAsDouble();
  }

  /* periodic */

  @Override
  public void periodic() {

    // updates the odometry positon
    poseEstimator.update(m_gyro.getRotation2d(), getModulePositions());

    visionEstLml3 = vision.getEstimatedGlobalPoseLml3();
    visionEstLml2L = vision.getEstimatedGlobalPoseLml2Left();
    visionEstLml2R = vision.getEstimatedGlobalPoseLml2Right();

    SmartDashboard.putNumber("gyro z", getGyroZ());

    if (visionEstLml3.isPresent()
        || visionEstLml2R.isPresent()
        || visionEstLml2L.isPresent() && !hasStartedEstimation) {
      hasStartedEstimation = true;
      estimatePose();
    }

    m_field2d.setRobotPose(poseEstimator.getEstimatedPosition());

    if (DriverStation.isDisabled()) {
      m_targetModeEnabled = false;
      drive(0);
    }

    if (m_targetModeEnabled) {

      if (!DriverStation.isAutonomous()) {
        m_xController.setConstraints(
            new Constraints(
                kMaxSpeedMetersPerSecondX
                    * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos()),
                kMaxAccelerationMetersPerSecondSquaredX
                    * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos())));
        m_yController.setConstraints(
            new Constraints(
                kMaxSpeedMetersPerSecondY
                    * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos()),
                kMaxAccelerationMetersPerSecondSquaredY
                    * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos())));
        m_rotController.setConstraints(
            new Constraints(
                kMaxSpeedRadiansPerSecond
                    * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos()),
                kMaxAccelerationRadiansPerSecondSquared
                    * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos())));

        SmartDashboard.putNumber(
            "max speed x",
            kMaxSpeedMetersPerSecondX
                * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos()));
        SmartDashboard.putNumber(
            "max acceleration x",
            kMaxAccelerationMetersPerSecondSquaredX
                * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos()));
        SmartDashboard.putNumber(
            "max speed y",
            kMaxSpeedMetersPerSecondY
                * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos()));
        SmartDashboard.putNumber(
            "max acceleration y",
            kMaxAccelerationMetersPerSecondSquaredY
                * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos()));
        SmartDashboard.putNumber(
            "max speed rotation",
            kMaxSpeedRadiansPerSecond
                * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos()));
        SmartDashboard.putNumber(
            "max acceleration rotation",
            kMaxAccelerationRadiansPerSecondSquared
                * SwerveElevatorSlowDownFunc.calculate(() -> m_elevator.getEncoderPos()));
      }
      var x = m_xController.calculate(poseEstimator.getEstimatedPosition().getX());
      var y = m_yController.calculate(poseEstimator.getEstimatedPosition().getY());
      var rot = 0.0;
      rot =
          m_rotController.calculate(
              poseEstimator.getEstimatedPosition().getRotation().getRadians());

      _drive(new Translation2d(x, y), rot, true, true);
    }

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CTRE Mag encoder", mod.getMagEncoderPos().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

  /* thread */

  public void estimatePose() {

    // if vision estimation is present, create method est to add vision measurment
    // to
    // pose estimator with estimated pose, estimated timestamp and estimated stdDevs

    visionEstLml3.ifPresent(
        est -> {
          var estStdDevs = vision.getEstimationStdDevsLml3();
          poseEstimator.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });

    visionEstLml2R.ifPresent(
        est -> {
          var estStdDevs = vision.getEstimationStdDevsLml2();
          poseEstimator.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });

    visionEstLml2L.ifPresent(
        est -> {
          var estStdDevs = vision.getEstimationStdDevsLml2();
          poseEstimator.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });

    hasStartedEstimation = false;
  }

  /* drive related things */

  private boolean isAlmostEqual(double a, double b, double epsilon) {
    return Math.abs(a - b) < epsilon;
  }

  public boolean targetReached() {
    var posX = getPose().getX();
    var posY = getPose().getY();
    var rot = getPose().getRotation().getRadians();
    var goalX = m_xController.getGoal().position;
    var goalY = m_yController.getGoal().position;
    var goalRot = m_rotController.getGoal().position;

    return isAlmostEqual(posX, goalX, 0.01)
        && isAlmostEqual(posY, goalY, 0.01)
        && isAlmostEqual(rot, goalRot, Units.degreesToRadians(2));
  }

  public void disableDriveToTarget() {
    m_targetModeEnabled = false;
  }

  public boolean targetDriveDisabled() {
    return m_targetModeEnabled == false;
  }

  public void drivetoTarget(Pose2d target) {
    if (target == Pose2d.kZero) {
      m_targetModeEnabled = false;
    } else {
      m_targetModeEnabled = true;
      m_xController.reset(getPose().getX());
      m_xController.setGoal(target.getX());
      m_yController.reset(getPose().getY());
      m_yController.setGoal(target.getY());
      m_rotController.reset(getPose().getRotation().getRadians());
      m_rotController.setGoal(target.getRotation().getRadians());
    }
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    if (!m_targetModeEnabled) _drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  private void _drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getHeading())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    setStates(swerveModuleStates, isOpenLoop);
  }

  public void setStates(SwerveModuleState[] targetStates, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxSpeed);

    for (int i = 0; i < mSwerveMods.length; i++) {
      mSwerveMods[i].setDesiredState(targetStates[i], isOpenLoop);
    }
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

    // ChassisSpeeds fieldRelativeSpeed =
    // ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getHeading());
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates, true);
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void drive(double voltage) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDriveVoltage(voltage);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    var curPos = poseEstimator.getEstimatedPosition();
    // System.out.println(String.format("x: %f y:%f", curPos.getX(),
    // curPos.getY()));
    return curPos;
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    poseEstimator.resetPosition(
        getRotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    poseEstimator.resetPosition(
        getRotation2d(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees());
  }

  /**
   * WARNING: This method takes some time (based on the number of calibration samples to collect) to
   * execute! It performs frequency calibration of all the absolute magnetic encoder to improve
   * their accuracy and stability
   */
  public void resetModulesToAbsolute() {
    // perform absolute encoders frequency calibration
    for (int i = 0; i < Constants.Swerve.calibrationFreqSamples; i++) {
      for (SwerveModule mod : mSwerveMods) mod.calibrateMagEncoder();
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    for (SwerveModule mod : mSwerveMods) mod.resetToAbsolute();
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  /* pathplanner config */

  private void configurePathPlanner() {

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getSpeeds,
        this::driveRobotRelative,
        new PPHolonomicDriveController(new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0)),
        Constants.AutoConstants.kRobotConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  private Command playThemeMusic() {
    return this.runOnce(
        () -> {
          var musicFilePath =
              Filesystem.getDeployDirectory() + File.separator + "finalcountdown.chrp";
          System.out.println(musicFilePath);
          System.out.println(m_orchestra.loadMusic(musicFilePath));
          System.out.println(m_orchestra.play());
        });
  }

  @Override
  public void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller) {
    moduleTrigger.and(controller.a()).onTrue(createTrajectoryCommand());
    moduleTrigger.and(controller.x()).onTrue(playThemeMusic());
  }

  public Command createTrajectoryCommand() {

    System.out.println("hello");

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(0.5, 0.5)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            this.getPose(),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2.66, 4.03, new Rotation2d()),
            config);

    var thetaController = new ProfiledPIDController(1, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            this::getPose, // Functional interface to feed supplier
            Constants.Swerve.swerveKinematics,

            // Position controllers
            new PIDController(5.0, 0, 0),
            new PIDController(5.0, 0, 0),
            thetaController,
            this::setModuleStates,
            this);

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.sequence(
        new PrintCommand("hello again"),
        new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> this.drive(new Translation2d(0, 0), 0, false, false)));
  }
}
