// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ZeroTurretCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(IntakeSubsystem.initializeHardware());
  private final FeederSubsystem m_feederSubsystem = new FeederSubsystem(FeederSubsystem.initializeHardware());
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(ShooterSubsystem.initializeHardware(), 
                                                                           Constants.HOOD_MOTOR_CONFIG, 
                                                                           Constants.FLYWHEEL_CONFIG, 
                                                                           Constants.TURRET_CONFIG, Constants.SHOOTER_VISION_MAP);

  private final XboxController m_controller = new XboxController(0);

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
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton primaryButtonA = new JoystickButton(m_controller, Button.kA.value);
    JoystickButton primaryButtonB = new JoystickButton(m_controller, Button.kB.value);
    JoystickButton primaryButtonX = new JoystickButton(m_controller, Button.kX.value);
    JoystickButton primaryButtonY = new JoystickButton(m_controller, Button.kY.value);
    JoystickButton primaryButtonLBumper = new JoystickButton(m_controller, Button.kLeftBumper.value);
    JoystickButton primaryButtonRBumper = new JoystickButton(m_controller, Button.kRightBumper.value);
    POVButton primaryDPadUp = new POVButton(m_controller, 0);
    POVButton primaryDPadRight = new POVButton(m_controller, 90);
    POVButton primaryDPadDown = new POVButton(m_controller, 180);
    POVButton primaryDPadLeft = new POVButton(m_controller, 270);
    Trigger primaryTriggerLeft = new Trigger(() -> m_controller.getLeftTriggerAxis() > 0.25);

    primaryButtonB.whenPressed(new ZeroTurretCommand(m_shooterSubsystem, m_intakeSubsystem));
    primaryButtonX.whenPressed(new InstantCommand(() -> m_intakeSubsystem.toggleArmPosition()));
    primaryButtonY.whenPressed(new InstantCommand(() -> m_shooterSubsystem.printThing()));
    primaryButtonRBumper.whenHeld(new IntakeCommand(m_intakeSubsystem, m_feederSubsystem));
    primaryButtonLBumper.whenHeld(new OuttakeCommand(m_intakeSubsystem, m_feederSubsystem));
    primaryTriggerLeft.whileActiveOnce(new ShootCommand(m_shooterSubsystem, m_feederSubsystem));

    primaryButtonA.whenPressed(new InstantCommand(() -> m_shooterSubsystem.setHoodPosition(30)));

    primaryDPadUp.whenHeld(new InstantCommand(() -> m_shooterSubsystem.setHoodSpeed(+0.3)))
      .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop()));
    primaryDPadDown.whenHeld(new InstantCommand(() -> m_shooterSubsystem.setHoodSpeed(-0.3)))
      .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop()));

    primaryDPadRight.whenHeld(new InstantCommand(() -> m_shooterSubsystem.setTurretSpeed(+0.1)))
      .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop()));
    primaryDPadLeft.whenHeld(new InstantCommand(() -> m_shooterSubsystem.setTurretSpeed(-0.1)))
      .whenReleased(new InstantCommand(() -> m_shooterSubsystem.stop()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
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
