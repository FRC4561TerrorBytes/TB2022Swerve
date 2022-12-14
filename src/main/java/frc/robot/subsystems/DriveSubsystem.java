// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  // By default we use a Pigeon for our gyroscope. But if you use another
  // gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.DRIVE_KINEMATICS, new Rotation2d());

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  public DriveSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    //Set Drive and Turn Motor Current Limits
    Mk4ModuleConfiguration m_moduleConfig = new Mk4ModuleConfiguration();
     m_moduleConfig.setDriveCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
     m_moduleConfig.setSteerCurrentLimit(Constants.TURN_CURRENT_LIMIT);

    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        // This parameter is optional, but will allow you to see the current state of
        // the module on the dashboard.
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        //Mk4ModuleConfiguration w/ current limits and nominal voltage defined above
            //Defaults -> Drive Motors - 80, Turn Motors - 20, Nominal Voltage - 12.0
        m_moduleConfig,
        // This can either be STANDARD or FAST depending on your gear configuration
        Mk4iSwerveModuleHelper.GearRatio.L2,
        // This is the ID of the drive motor
        Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
        // This is the ID of the steer motor
        Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
        // This is the ID of the steer encoder
        Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
        // This is how much the steer encoder is offset from true zero (In our case,
        // zero is facing straight forward)
        Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

    // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        m_moduleConfig,
        Mk4iSwerveModuleHelper.GearRatio.L2,
        Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
        Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
        Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        m_moduleConfig,
        Mk4iSwerveModuleHelper.GearRatio.L2,
        Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_ENCODER,
        Constants.BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        m_moduleConfig,
        Mk4iSwerveModuleHelper.GearRatio.L2,
        Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
        Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

    m_navx.calibrate();
  }

  public Rotation2d getGyroscopeRotation() {
    // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_VELOCITY_METERS_PER_SECOND);

    setModuleStates(states);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    m_frontLeftModule.set(states[0].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        states[3].angle.getRadians());

    m_odometry.update(getGyroscopeRotation(), states);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d position) {
    m_navx.reset();
    m_odometry.resetPosition(position, new Rotation2d());
  }

  public void stop() {
    drive(new ChassisSpeeds());
  }

  @Override
  public void periodic() {
  }
}
