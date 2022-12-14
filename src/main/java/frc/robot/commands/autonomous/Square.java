// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Square extends SequentialCommandGroup {
  /** Creates a new Square. */
  public Square(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoTrajectory(driveSubsystem, "square1", 1, 1).getCommandAndStop(),
      new AutoTrajectory(driveSubsystem, "square2", 1, 1).getCommandAndStop(),
      new AutoTrajectory(driveSubsystem, "square3", 1, 1).getCommandAndStop(),
      new AutoTrajectory(driveSubsystem, "square4", 1, 1).getCommandAndStop()
    );
  }
}
