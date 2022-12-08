// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;

// import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class IntakeSubsystem extends SubsystemBase {

  // private final I2C.Port i2cPort = I2C.Port.kOnboard;

  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // private final Color m_defaultColor = new Color(0.150, 0.0, 0.296);

  private static class Hardware {
    private CANSparkMax feederMotor;
    private Solenoid leftSolenoid;
    private Solenoid rightSolenoid;

    public Hardware(CANSparkMax feederMotor, Solenoid leftSolenoid, Solenoid rightSolenoid) {
      this.feederMotor = feederMotor;
      this.leftSolenoid = leftSolenoid;
      this.rightSolenoid = rightSolenoid;
    }
  }

  /** An enumeration of valid intake arm positions. */
  private enum ArmPosition {
    /** The starting and stopped, intake retracted position. */
    Top(leftSolenoid.get()),
    /** The intake extended for intake or outtake operation. */
    Bottom(!leftSolenoid.get());

    /** The intake arm positional PID control set point. */
    public final double setPoint;

    /**
     * Creates a new arm position.
     * 
     * @param setPoint the intake arm positional PID set point.
     */
    private ArmPosition(double setPoint) {
      this.setPoint = setPoint;
    }
  }

  /**
   * This enum describes the steps in handling the extension and retraction of the
   * intake.
   */
  private enum IntakeRequest {
    /** No current state change request. */
    NO_REQUEST_PENDING,
    /** Extension and intake requested. */
    INTAKE_REQUESTED,
    /** Extension and outtake requested. */
    OUTTAKE_REQUESTED,
    /** Retraction and stop of the intake has been requested. */
    RETRACTION_REQUESTED;
  }

  public static Hardware initializeHardware() {
    return new Hardware(
      new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless),
      new Solenoid(Constants.PCM, PneumaticsModuleType.CTREPCM, Constants.LEFT_SOLENOID),
      new Solenoid(Constants.PCM, PneumaticsModuleType.CTREPCM, Constants.RIGHT_SOLENOID)
    );
  }

  private CANSparkMax m_feederMotor;
  private Solenoid m_leftSolenoid;
  private Solenoid m_rightSolenoid;

  /** Creates a new IntakeSubsyste. */
  public IntakeSubsystem(Hardware intakeHardware) {
    this.m_feederMotor = intakeHardware.feederMotor;
    this.m_leftSolenoid = intakeHardware.leftSolenoid;
    this.m_rightSolenoid = intakeHardware.rightSolenoid;

    m_feederMotor.setIdleMode(IdleMode.kCoast);
    m_feederMotor.setInverted(true);
  }

  public void setArmPosition(boolean armPosition) {
    m_leftSolenoid.set(armPosition);
    m_rightSolenoid.set(armPosition);
  }

  public void toggleArmPosition() {
    boolean newPosition = !m_leftSolenoid.get();
    setArmPosition(newPosition);
  }

  public void armDown() {
    setArmPosition(true);
  }

  public void armUp() {
    setArmPosition(false);
  }

  public void intakeSpeed(double speed) {
    m_feederMotor.set(speed);
  }

  public void intake() {
    armDown();
    m_feederMotor.set(+Constants.INTAKE_SPEED);
  }

  public void outtake() {
    if (m_leftSolenoid.get())
      m_feederMotor.set(+Constants.INTAKE_SPEED);
  }

  public void stop() {
    m_feederMotor.stopMotor();
  }

  // public Color getColorDifference() {
  //   Color currentColor = m_colorSensor.getColor();
  //   return new Color(currentColor.red - m_defaultColor.red, 0.0, currentColor.blue - m_defaultColor.blue);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Color detectedColor = m_colorSensor.getColor();
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("Red", detectedColor.red);
  }

  /**
   * Get both limit switches for ShooterSubsystem
   */
  public SparkMaxLimitSwitch[] getLimitSwitches() {
    return new SparkMaxLimitSwitch[] {
      m_feederMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen),
      m_feederMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)
    };
  }
}
