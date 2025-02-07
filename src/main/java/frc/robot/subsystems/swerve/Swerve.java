package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.TestBindings;
import frc.robot.Constants;
import frc.robot.vision.Vision;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Swerve extends SubsystemBase implements TestBindings {
  public SwerveModule[] mSwerveMods;
  public SwerveModulePosition[] positions;
  private final AHRS m_gyro;
  private final Field2d m_field2d;
  public SwerveDriveOdometry m_odometry;
  private boolean m_debug = true;
  private Vision vision;
  private final SwerveDrivePoseEstimator poseEstimator;
  Thread thread = new Thread();
  ShuffleboardTab VisionSwerveTab = Shuffleboard.getTab("vision and swerve");
  private boolean hasStartedEstimation = false;

  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(Math.PI, Math.PI);

  // vision estimation of robot pose
  Optional<EstimatedRobotPose> visionEst;

  public Swerve(Vision vision) {
    m_gyro = new AHRS(NavXComType.kMXP_SPI);
    m_field2d = new Field2d();
    m_gyro.reset();
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
      SmartDashboard.putData(m_field2d);
    }

    m_odometry =
        new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics,
            m_gyro.getRotation2d(),
            positions,
            new Pose2d(0, 0, new Rotation2d()));
    configurePathPlanner();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, getRotation2d(), getModulePositions(), new Pose2d());
  }

  /* periodic */

  @Override
  public void periodic() {

    // updates the odometry positon
    // var m_odometryPose = m_odometry.update(m_gyro.getRotation2d(), getModulePositions());
    // Renews the field periodically
    // m_field2d.setRobotPose(m_odometryPose);

    poseEstimator.update(m_gyro.getRotation2d(), getModulePositions());

    visionEst = vision.getEstimatedGlobalPose();

    if (visionEst.isPresent() && !hasStartedEstimation) {
      hasStartedEstimation = true;
      estimatePose();
    }

    m_field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    // System.out.println(getRotation2d());

    if (m_debug) {
      // smartdashboardDebug();
      for (SwerveModule mod : mSwerveMods) {
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " CTRE Mag encoder", mod.getMagEncoderPos().getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      }

      SmartDashboard.putNumber("currentpose X", poseEstimator.getEstimatedPosition().getX());
      SmartDashboard.putNumber("currentpose Y", poseEstimator.getEstimatedPosition().getY());
    }
  }

  /* thread */

  public void estimatePose() {

    // if vision estimation is present, create method est to add vision measurment to
    // pose estimator with estimated pose, estimated timestamp and estimated stdDevs

    visionEst.ifPresent(
        est -> {
          var estStdDevs = vision.getEstimationStdDevs();
          poseEstimator.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });

    hasStartedEstimation = false;
  }

  /* drive related things */

  public void drive(
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
    if (m_debug)
      System.out.println(
          String.format(
              "driveRobotRelative: omega: %f, vx: %f, vy : %f",
              robotRelativeSpeeds.omegaRadiansPerSecond,
              robotRelativeSpeeds.vxMetersPerSecond,
              robotRelativeSpeeds.vyMetersPerSecond));
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
    return m_odometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    m_odometry.resetPosition(
        getRotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    m_odometry.resetPosition(
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
    m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  /* pathplanner config */

  private void configurePathPlanner() {
    // TODO make actual configs for autobuilder
    // try catch to remove parsing error
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getSpeeds,
        this::driveRobotRelative,
        Constants.AutoConstants.kPathFollowController,
        Constants.AutoConstants.kRobotConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
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

  @Override
  public void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller) {

    moduleTrigger.and(controller.a()).onTrue(getTestTrajectoryCommand());
  }

  public Command getTestTrajectoryCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(3.0, 3.0)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            Pose2d.kZero,
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, Rotation2d.kZero),
            config);

    var thetaController = new ProfiledPIDController(1, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            this::getPose, // Functional interface to feed supplier
            Constants.Swerve.swerveKinematics,

            // Position controllers
            new PIDController(1.0, 0, 0),
            new PIDController(1.0, 0, 0),
            thetaController,
            this::setModuleStates,
            this);

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.sequence(
        new InstantCommand(() -> this.resetOdometry(exampleTrajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> this.drive(new Translation2d(0, 0), 0, false, false)));
  }
}
