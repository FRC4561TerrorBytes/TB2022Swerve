// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.autonomous.Line;
import frc.robot.commands.autonomous.Square;
import frc.robot.commands.autonomous.TaxiCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drivetrainSubsystem = new DriveSubsystem();

  private final XboxController m_primaryController = new XboxController(0);

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
            () -> -modifyAxis(m_primaryController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_primaryController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_primaryController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            false
    ));

    m_autommodeChooser.setDefaultOption("Do nothing", null);
    m_autommodeChooser.addOption("Taxi back", new TaxiCommand(m_drivetrainSubsystem, 0, 5));
    m_autommodeChooser.addOption("Taxi back right", new TaxiCommand(m_drivetrainSubsystem, 1, 5));
    m_autommodeChooser.addOption("Taxi back left", new TaxiCommand(m_drivetrainSubsystem, -1, 5));
    m_autommodeChooser.addOption("Line", new Line(m_drivetrainSubsystem));
    m_autommodeChooser.addOption("sQUARE", new Square(m_drivetrainSubsystem));

    SmartDashboard.putData(m_autommodeChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  @SuppressWarnings("unused")
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
    value = Math.copySign(value * value, value);

    return value;
  }
}
