// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.AutoModes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autonomous.AutoTrajectory;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveForwardAuto extends SequentialCommandGroup {

  private DrivetrainSubsystem m_drivetrainSubsystem;

  public DriveForwardAuto(DrivetrainSubsystem drivetrainSubsystem) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;

    AutoTrajectory DriveForwardAuto = new AutoTrajectory(
      m_drivetrainSubsystem, "DriveForwardAuto", Constants.MAX_SPEED_METERS_PER_SEC, 
        Constants.MAX_ACCELERATION_METERS_PER_SEC_SQUARED);

    addCommands(
      DriveForwardAuto.getCommandAndStop()
    );
  }
}
