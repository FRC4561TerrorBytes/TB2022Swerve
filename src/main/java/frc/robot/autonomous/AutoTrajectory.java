// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.pathplanner.lib.PathPlanner;

public class AutoTrajectory {

    DrivetrainSubsystem m_driveSubsystem;
    Trajectory m_pathPlannerTrajectory;
    SwerveControllerCommand m_swerveControllerCommand;

    public AutoTrajectory(DrivetrainSubsystem driveSubsystem, String autoPathName, double maxSpeedMetersPerSec, double maxAccelerationMetersPerSecSquared){
        this.m_driveSubsystem = driveSubsystem;

        //Trajectory Settings
        TrajectoryConfig autoTrajectoryConfig = new TrajectoryConfig(
            maxSpeedMetersPerSec, 
            maxAccelerationMetersPerSecSquared)
                .setKinematics(Constants.DRIVE_KINEMATICS);

         //Generate PathPlanner Trajectory
         m_pathPlannerTrajectory = PathPlanner.loadPath(autoPathName, maxSpeedMetersPerSec, maxAccelerationMetersPerSecSquared);

         //Auto PID Controllers
        PIDController xController = new PIDController(Constants.AUTO_X_KP, Constants.AUTO_X_KI, Constants.AUTO_X_KD);
        PIDController yController = new PIDController(Constants.AUTO_Y_KP, Constants.AUTO_Y_KI, Constants.AUTO_Y_KD);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.AUTO_THETA_KP, Constants.AUTO_THETA_KI, Constants.AUTO_THETA_KD, 
            Constants.AUTO_THETA_CONSTRAINTS);

            SwerveControllerCommand m_swerveControllerCommand = new SwerveControllerCommand(
                m_pathPlannerTrajectory, 
                m_driveSubsystem::getPose,
                Constants.DRIVE_KINEMATICS,
                xController, 
                yController, 
                thetaController, 
                m_driveSubsystem::setModuleStates, 
                m_driveSubsystem);

    }

    public void resetOdometry(){
        m_driveSubsystem.resetOdometry(m_pathPlannerTrajectory.getInitialPose());
    }

    public Command getCommandAndStop(){
        return new InstantCommand(() -> resetOdometry(), m_driveSubsystem).andThen(
            m_swerveControllerCommand.withTimeout(m_pathPlannerTrajectory.getTotalTimeSeconds()).andThen(() -> {
                m_driveSubsystem.stopMotors();
            })
        );
    }
}

