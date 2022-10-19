package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DefaultDriveCommand extends CommandBase {
    private final DriveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private final HolonomicDriveController m_holonomicDriveController = 
        new HolonomicDriveController(
            new PIDController(1,0,0), 
            new PIDController(1,0,0), 
            new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));

    public DefaultDriveCommand(DriveSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                m_holonomicDriveController.calculate(
                    new Pose2d(0, 0, m_drivetrainSubsystem.getGyroscopeRotation()),
                    new Trajectory.State(), // ???
                    Rotation2d.fromDegrees(0)
                )
        );
        // if (m_translationXSupplier.getAsDouble() == 0 && m_translationYSupplier.getAsDouble() == 0 && m_rotationSupplier.getAsDouble() == 0) {
        //     m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.001));
        // }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
