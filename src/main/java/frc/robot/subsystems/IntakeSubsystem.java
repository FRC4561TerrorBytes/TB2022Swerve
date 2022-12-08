// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import java.util.concurrent.atomic.AtomicReference;
import edu.wpi.first.wpilibj.GenericHID; 
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

  private ArmPosition m_armPosition;

  private final AtomicReference<IntakeRequest> m_request = new AtomicReference<>(IntakeRequest.NO_REQUEST_PENDING);
  private final IntakeStateMachine m_stateMachine;
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
  public IntakeSubsystem(Hardware intakeHardware, final GenericHID rumbleController) {
    this.m_feederMotor = intakeHardware.feederMotor;
    this.m_leftSolenoid = intakeHardware.leftSolenoid;
    this.m_rightSolenoid = intakeHardware.rightSolenoid;

    m_feederMotor.setIdleMode(IdleMode.kCoast);
    m_feederMotor.setInverted(true);

    m_stateMachine = new IntakeStateMachine(this, rumbleController);
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

  /**
   * Called to request that we start the intake of cargo.
   * 
   * @return true if the request was granted (currently retracted).
   */
  public boolean requestIntake() {
    return isStateMachineRunning()
        && isRetracted()
        && this.m_request.compareAndSet(IntakeRequest.NO_REQUEST_PENDING, IntakeRequest.INTAKE_REQUESTED);
  }

  /**
   * Called to request that we start the outtake of cargo.
   * 
   * @return true if the request was granted (currently retracted).
   */
  public boolean requestOuttake() {
    return isStateMachineRunning()
        && isRetracted()
        && this.m_request.compareAndSet(IntakeRequest.NO_REQUEST_PENDING, IntakeRequest.OUTTAKE_REQUESTED);
  }

  /**
   * Called to request that the intake be retracted.
   * 
   * @return true if the request was granted.
   */
  public boolean requestRetraction() {
    return isStateMachineRunning()
        && !isRetracted()
        && this.m_request.compareAndSet(IntakeRequest.NO_REQUEST_PENDING, IntakeRequest.RETRACTION_REQUESTED);
  }

  /**
   * @return true if intake has been requested but not yet handled.
   */
  boolean isIntakeRequested() {
    return this.m_request.get() == IntakeRequest.INTAKE_REQUESTED;
  }

  /**
   * @return true if intake has been requested but not yet handled.
   */
  boolean isOuttakeRequested() {
    return this.m_request.get() == IntakeRequest.OUTTAKE_REQUESTED;
  }

  /**
   * @return true if either intake or outake has been requested but not yet
   *         handled.
   */
  boolean isExtensionRequested() {
    return isIntakeRequested() || isOuttakeRequested();
  }

  /**
   * @return true if the intake is retracted or on its way to retracted and false
   *         otherwise.
   */
  boolean isRetracted() {
    return this.m_armPosition == ArmPosition.Top;
  }

  /**
   * @return true if intake retraction has been requested but not yet handled.
   */
  boolean isRetractionRequested() {
    return this.m_request.get() == IntakeRequest.RETRACTION_REQUESTED;
  }

  /**
   * @return true (the caller can move to next state machine state) if an
   *         intake had been requested.
   */
  boolean intakeHandled() {
    return this.m_request.compareAndSet(IntakeRequest.INTAKE_REQUESTED, IntakeRequest.NO_REQUEST_PENDING);
  }

  /**
   * @return true (the caller can move to next state machine state) if an
   *         outtake had been requested.
   */
  boolean outtakeHandled() {
    return this.m_request.compareAndSet(IntakeRequest.OUTTAKE_REQUESTED, IntakeRequest.NO_REQUEST_PENDING);
  }

  /**
   * @return true (the caller can move to next state machine state) if a
   *         retraction had been requested.
   */
  boolean retractionHandled() {
    return this.m_request.compareAndSet(IntakeRequest.RETRACTION_REQUESTED, IntakeRequest.NO_REQUEST_PENDING);
  }

  /**
   * @return true if the state machine is running and false otherwise.
   */
  public boolean isStateMachineRunning() {
    return this.m_stateMachine.getDefaultCommand().isScheduled();
  }
}
