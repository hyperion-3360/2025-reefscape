package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  public SwerveModule[] mSwerveMods;
  public SwerveModulePosition[] positions;
  private final AHRS m_gyro;
  private final Field2d m_field2d;
  public SwerveDriveOdometry m_odometry;
  private boolean m_debug = true;

  public Swerve() {
    // TODO get good port
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_field2d = new Field2d();
    m_gyro.reset();
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
  }

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

  private void configurePathPlanner() {

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        this::getSpeeds,
        this::driveRobotRelative,
        Constants.AutoConstants.kHolonomicPathFollowerConfig,
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
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
    for (SwerveModule mod : mSwerveMods) mod.resetToAbsolute();
  }

  @Override
  public void periodic() {
    if (m_debug) {
      for (SwerveModule mod : mSwerveMods) {
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " CTRE Mag encoder", mod.getMagEncoderPos().getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      }
      m_debug = false;
    }

    m_gyro.getRotation2d();
    // updates the odometry positon
    var m_odometryPose = m_odometry.update(m_gyro.getRotation2d(), getModulePositions());
    // Renews the field periodically
    m_field2d.setRobotPose(m_odometryPose);
  }

  public Command resetOdometryBlueSide() {
    return this.runOnce(
        () ->
            m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                getModulePositions(),
                new Pose2d(2.1, 5, Rotation2d.fromDegrees(-180))));
  }

  public Command resetOdometryRedSide() {
    return this.runOnce(
        () ->
            m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                getModulePositions(),
                new Pose2d(14.4, 5, Rotation2d.fromDegrees(180))));
  }

  private SysIdRoutine m_driveSysIdRoutine =
      new SysIdRoutine(
          //          new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
          new SysIdRoutine.Config(null, null, null, null),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> drive(volts.in(Volts)), null, this));

  public Command runDriveQuasiTest(Direction direction) {
    return m_driveSysIdRoutine.quasistatic(direction);
  }

  public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
    return m_driveSysIdRoutine.dynamic(direction);
  }
}
