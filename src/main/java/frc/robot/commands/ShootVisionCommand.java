// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.FeederSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.VisionSubsystem;


// public class ShootVisionCommand extends CommandBase {
//   private DrivetrainSubsystem m_driveSubsystem;
//   private ShooterSubsystem m_shooterSubsystem;
//   private VisionSubsystem m_visionSubsystem;
//   private FeederSubsystem m_feederSubsystem;
//   private int m_loops = 0;
//   private int m_loopNum;
//   /**
//    * Shoot using vision
//    * @param driveSubsystem drive subsystem
//    * @param shooterSubsystem shooter subsystem
//    * @param visionSubsystem vision subsystem
//    * @param delay shoot delay in seconds
//    * @param odometry whether to update drive odometry or not
//    */
//   public ShootVisionCommand(DrivetrainSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem,
//                             FeederSubsystem feederSubsystem, 
//                             VisionSubsystem visionSubsystem, double delay) {
//     this.m_driveSubsystem = driveSubsystem;
//     this.m_shooterSubsystem = shooterSubsystem;
//     this.m_feederSubsystem = feederSubsystem;
//     this.m_visionSubsystem = visionSubsystem;
//     this.m_loopNum = (int)Math.round(delay / (1.0/60.0));

//     // Use addRequirements() here to declare subsystem dependencies
//     addRequirements(m_driveSubsystem, m_shooterSubsystem, m_visionSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // Only run if target is valid
//     if (m_visionSubsystem.isTargetValid()) {
//       // Run flywheel based on distance from 
//       double distance = m_visionSubsystem.getDistance();
      

//       // Aim turret toward target
//       // Only run feeder if flywheel is at speed and robot is on target, else stop
//       if (m_shooterSubsystem.isFlywheelAtSpeed(m_visionSubsystem.getDistance()) &&  m_visionSubsystem.isOnTarget()) {
//         m_loops++;
//         if (m_loops > m_loopNum) m_feederSubsystem.feederShoot();
//       } else {
//         m_loops = 0;
//         m_feederSubsystem.stop();
//       }
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     // Stop flywheel and feeder
//     m_shooterSubsystem.stop();
//     m_feederSubsystem.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//    /**
//    * Automatically sets the flywheel speed based on vision curve
//    * <p>
//    * NOTE: This method ALWAYS shoots high
//    */
//   public void setShooterVision(double distance) {
//     double flywheelSpeed = m_flywheelVisionCurve.value(MathUtil.clamp(distance, m_minDistance, m_maxDistance));
  
//     double hoodSetpoint = m_hoodVisionCurve.value(MathUtil.clamp(distance, m_minDistance, m_maxDistance));

//     setFlywheelSpeed(flywheelSpeed);
//     setHoodPosition(hoodSetpoint);
//   }

//   /**
//    * Checks if flywheel is at set speed
//    * @return True if flywheel is at speed else false
//    */
//   public boolean isFlywheelAtSpeed(double distance) {
//     double flywheelTargetSpeed = m_flywheelVisionCurve.value(MathUtil.clamp(distance, m_minDistance, m_maxDistance));
//     double rightFlywheelError = Math.abs(flywheelTargetSpeed - m_rightFlywheelMotor.getEncoder().getVelocity());
//     double leftFlywheelError = Math.abs(flywheelTargetSpeed - m_leftFlywheelMotor.getEncoder().getVelocity());
//     double hoodError = Math.abs(m_hoodMotor.getClosedLoopError());

//     boolean isRightFlywheelAtSpeed = (rightFlywheelError < 1.0)
//                                     && flywheelTargetSpeed != 0;
//     boolean isLeftFlywheelAtSpeed = (leftFlywheelError < 1.0)
//                                       && flywheelTargetSpeed != 0;
//     boolean isHoodAtAngle = (hoodError < 1.0)
//                               && m_hoodMotor.getClosedLoopTarget() != 0;
  
//     return isRightFlywheelAtSpeed && isLeftFlywheelAtSpeed && isHoodAtAngle;
//   }
// }

