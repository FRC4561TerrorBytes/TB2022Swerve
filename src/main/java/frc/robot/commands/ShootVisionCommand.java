// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootVisionCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private VisionSubsystem m_visionSubsystem;
    private FeederSubsystem m_feederSubsystem;
    private int m_loops = 0;
    private int m_loopNum;

    /**
     * Shoot using vision
     * 
     * @param driveSubsystem   drive subsystem
     * @param shooterSubsystem shooter subsystem
     * @param visionSubsystem  vision subsystem
     * @param delay            shoot delay in seconds
     * @param odometry         whether to update drive odometry or not
     */
    public ShootVisionCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem,
            FeederSubsystem feederSubsystem,
            VisionSubsystem visionSubsystem, double delay) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_shooterSubsystem = shooterSubsystem;
        this.m_feederSubsystem = feederSubsystem;
        this.m_visionSubsystem = visionSubsystem;
        this.m_loopNum = (int) Math.round(delay / (1.0 / 60.0));

        // Use addRequirements() here to declare subsystem dependencies
        addRequirements(m_shooterSubsystem, m_visionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_visionSubsystem.isTargetValid()) {

            double yawOffset = m_visionSubsystem.getYaw();
            if (Math.abs(yawOffset) > Constants.TURRET_TOLERANCE) {
                m_shooterSubsystem.setTurretDelta(yawOffset);
            } else {
                m_shooterSubsystem.setTurretSpeed(0.0);
            }

            double dy = Constants.TARGET_HEIGHT_METERS - Constants.CAMERA_HEIGHT_METERS;
            double dx = m_visionSubsystem.getDistance() + Constants.TARGET_DISTANCE_OFFSET;

            double theta = MathUtil.clamp(/*Math.log10(dx-4)* Constants.HOOD_ANGLE_SCALAR*20+20*/0 , 20, 40);
            double velocity = velocityToTarget(Units.degreesToRadians(90 - theta), dx, dy);
            double rpm = metersPerSecondToRPM(velocity);

            // System.out.println("dx" + dx + " Theta: " + theta + " Velocity: " + velocity + " RPM: " + rpm);

            m_shooterSubsystem.setFlywheelSpeed(rpm);
            m_shooterSubsystem.setHoodPosition(theta);

            if (m_visionSubsystem.isOnTarget() && Math.abs(m_shooterSubsystem.getFlywheelSpeed() - rpm) < Constants.FLYWHEEL_TOLERANCE) {
                m_loops++;
                if (m_loops >= m_loopNum) 
                    m_feederSubsystem.feederShoot();
            } else {
                m_loops = 0;
                m_feederSubsystem.stop();
            }
        }
    }

    public double velocityToTarget(double theta, double dx, double dy) {
        try {
            return dx * Math.sqrt(Constants.GRAVITY / (2 * dx * Math.tan(theta) - 2 * dy)) / Math.cos(theta);
        } catch (Exception e) {
            return 0;
        }
    }

    public double metersPerSecondToRPM(double mps) {
        return 60 / Constants.WHEEL_CIRCUMFERENCE_METERS * mps * 2;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop flywheel and feeder
        m_shooterSubsystem.stop();
        m_feederSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
