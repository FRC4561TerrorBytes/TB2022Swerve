// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootVisionCommand;
import frc.robot.commands.TaxiCommand;
import frc.robot.commands.ZeroTurretCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drivetrainSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(IntakeSubsystem.initializeHardware());
  private final FeederSubsystem m_feederSubsystem = new FeederSubsystem(FeederSubsystem.initializeHardware());
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(ShooterSubsystem.initializeHardware(), 
                                                                           Constants.HOOD_MOTOR_CONFIG, 
                                                                           Constants.FLYWHEEL_CONFIG, 
                                                                           Constants.TURRET_CONFIG);
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(VisionSubsystem.initializeHardware(), Constants.CAMERA_HEIGHT_METERS, Constants.TARGET_HEIGHT_METERS, Constants.CAMERA_PITCH_DEGREES, 10);

  private final XboxController m_primaryController = new XboxController(0);
  private final XboxController m_secondaryController = new XboxController(1);

  private String m_allianceColor = "";

  private SendableChooser<Command> m_autommodeChooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_primaryController.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_primaryController.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_primaryController.getRightX()) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    m_autommodeChooser.setDefaultOption("Do nothing", null);
    m_autommodeChooser.addOption("Taxi back", new TaxiCommand(m_drivetrainSubsystem, 0));
    m_autommodeChooser.addOption("Taxi back right", new TaxiCommand(m_drivetrainSubsystem, 1));
    m_autommodeChooser.addOption("Taxi back left", new TaxiCommand(m_drivetrainSubsystem, -1));

    SmartDashboard.putData(m_autommodeChooser);

    m_shooterSubsystem.resetTurretEncoder();

    // Configure the button bindings
    configureButtonBindings();
  }

  public void setAllianceColor(String color) {
    m_allianceColor = color;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton primaryButtonA = new JoystickButton(m_primaryController, Button.kA.value);
    JoystickButton primaryButtonB = new JoystickButton(m_primaryController, Button.kB.value);
    JoystickButton primaryButtonX = new JoystickButton(m_primaryController, Button.kX.value);
    JoystickButton primaryButtonY = new JoystickButton(m_primaryController, Button.kY.value);
    JoystickButton primaryButtonLBumper = new JoystickButton(m_primaryController, Button.kLeftBumper.value);
    JoystickButton primaryButtonRBumper = new JoystickButton(m_primaryController, Button.kRightBumper.value);
    POVButton primaryDPadUp = new POVButton(m_primaryController, 0);
    POVButton primaryDPadRight = new POVButton(m_primaryController, 90);
    POVButton primaryDPadDown = new POVButton(m_primaryController, 180);
    POVButton primaryDPadLeft = new POVButton(m_primaryController, 270);
    Trigger primaryTriggerLeft = new Trigger(() -> m_primaryController.getLeftTriggerAxis() > 0.25);
    Trigger primaryTriggerRight = new Trigger(() -> m_primaryController.getRightTriggerAxis() > 0.25);

    primaryButtonB.whenPressed(new ZeroTurretCommand(m_shooterSubsystem, m_intakeSubsystem));
    primaryButtonX.whenPressed(new InstantCommand(() -> m_intakeSubsystem.toggleArmPosition()));
    
    primaryButtonRBumper.whenHeld(new IntakeCommand(m_intakeSubsystem, m_feederSubsystem, m_allianceColor));
    primaryButtonLBumper.whenHeld(new OuttakeCommand(m_intakeSubsystem, m_feederSubsystem));
    primaryTriggerLeft.whileActiveOnce(new ShootCommand(m_shooterSubsystem, m_feederSubsystem, 2800));
    primaryTriggerRight.whileActiveOnce(new ShootVisionCommand(m_drivetrainSubsystem, m_shooterSubsystem, m_feederSubsystem, m_visionSubsystem, 0.1));

    // primaryButtonA.whenPressed(new InstantCommand(() -> m_shooterSubsystem.setFlywheel(0.3))).whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop() ));

    primaryDPadUp.whenHeld(new InstantCommand(() -> m_shooterSubsystem.setHoodSpeed(+0.3)))
      .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop()));
    primaryDPadDown.whenHeld(new InstantCommand(() -> m_shooterSubsystem.setHoodSpeed(-0.3)))
      .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop()));

    primaryDPadRight.whenHeld(new InstantCommand(() -> m_shooterSubsystem.setTurretSpeed(+0.1)))
      .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop()));
    primaryDPadLeft.whenHeld(new InstantCommand(() -> m_shooterSubsystem.setTurretSpeed(-0.1)))
      .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop()));

    JoystickButton secondaryButtonX = new JoystickButton(m_secondaryController, Button.kX.value);
    JoystickButton secondaryButtonLBumper = new JoystickButton(m_secondaryController, Button.kLeftBumper.value);
    JoystickButton secondaryButtonRBumper = new JoystickButton(m_secondaryController, Button.kRightBumper.value);

    secondaryButtonX.whenPressed(new InstantCommand(() -> m_intakeSubsystem.toggleArmPosition()));
    secondaryButtonRBumper.whenHeld(new IntakeCommand(m_intakeSubsystem, m_feederSubsystem, m_allianceColor));
    secondaryButtonLBumper.whenHeld(new OuttakeCommand(m_intakeSubsystem, m_feederSubsystem));
    POVButton secondaryDPadUp = new POVButton(m_secondaryController, 0);
    POVButton secondaryDPadRight = new POVButton(m_secondaryController, 90);
    POVButton secondaryDPadDown = new POVButton(m_secondaryController, 180);
    POVButton secondaryDPadLeft = new POVButton(m_secondaryController, 270);

    secondaryDPadRight.whenHeld(new InstantCommand(() -> m_shooterSubsystem.setTurretSpeed(+0.1)))
      .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop()));
    secondaryDPadLeft.whenHeld(new InstantCommand(() -> m_shooterSubsystem.setTurretSpeed(-0.1)))
      .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autommodeChooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value * value, value);

    return value * 0.6;
  }
}
