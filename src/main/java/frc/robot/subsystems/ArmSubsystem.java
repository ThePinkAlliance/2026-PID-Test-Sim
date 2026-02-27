// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmSubsystemConstants.DIRECTION;

public class ArmSubsystem extends SubsystemBase {

  // Subsystem Components
  private DigitalInput m_EndSwitch;
  private DIOSim m_EndSwitchSim;
  private DigitalInput m_StartSwitch;
  private DIOSim m_StartSwitchSim;
  private Encoder m_PivotEncoder;
  private EncoderSim m_PivotEncoderSim;
  private SparkFlex m_PivotMotor;
  private DCMotor m_PivotMotorSimType;
  private SparkFlexSim m_PivotMotorSim;
  private SparkFlexConfig m_PivotConfig;
  private PIDController m_PivotPID;
  private double kP, kI, kD;
  private Pigeon2 m_IMU;
  private SingleJointedArmSim m_ArmSim;
  private final Mechanism2d m_mech2d = new Mechanism2d(90, 90,new Color8Bit(Color.kDarkSlateGray));

  
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 45, 21.75);
  
  private final MechanismLigament2d m_arm_base_right = m_armPivot.append(
      new MechanismLigament2d(
          "Arm Base Right",
          22,
          0, //start angle counterclockwise from the positive X axis, so -225 starts the arm pointing down and to the left
          20,
          new Color8Bit(Color.kPink)));
  private final MechanismLigament2d m_arm_base_left = m_armPivot.append(
      new MechanismLigament2d(
          "Arm Base Left",
          22,
          180, //start angle counterclockwise from the positive X axis, so -225 starts the arm pointing down and to the left
          20,
          new Color8Bit(Color.kPink)));
  
  private final MechanismLigament2d m_arm_base_stand = m_armPivot.append(
      new MechanismLigament2d(
          "Arm Base Stand",
          10,
          90, //start angle counterclockwise from the positive X axis, so -225 starts the arm pointing down and to the left
          20,
          new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d m_arm_bar = m_arm_base_stand.append(
      new MechanismLigament2d(
          "Arm Bar",
          20,
          0, //start angle counterclockwise from the positive X axis, so -225 starts the arm pointing down and to the left
          7,
          new Color8Bit(Color.kSnow)));

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Setup components
    SetupMotors();
    SetupEncoders();
    SetupSwitches(); 
    SetupPIDController(); 
    //SetupIMU();
    if (RobotBase.isSimulation()) {
      SetupMotorsSim();
      SetupEncodersSim();
      SetupSwitchesSim();
      SetupSimulation();
    }
  }

  // Setup the motors
  private void SetupMotors() {
    // Pivot Motor Config
    m_PivotMotor = new SparkFlex(Constants.ArmSubsystemConstants.ARM_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
    m_PivotConfig = new SparkFlexConfig();
    m_PivotConfig.idleMode(IdleMode.kBrake); // Coast or Brake
    m_PivotConfig.inverted(true); // Inverted or Not
    m_PivotConfig.secondaryCurrentLimit(Constants.ArmSubsystemConstants.PIVOT_MOTOR_CURRENT_LIMIT);
    m_PivotMotor.configure(
    (SparkBaseConfig) m_PivotConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  // Setup the motors in simulation
  private void SetupMotorsSim() {
    m_PivotMotorSimType = DCMotor.getNeoVortex(1);
    m_PivotMotorSim = new SparkFlexSim(m_PivotMotor, m_PivotMotorSimType);
  } 

  // Setup the encoders
  private void SetupEncoders() {
    m_PivotEncoder = new Encoder(Constants.ArmSubsystemConstants.ARM_ENCODER_DIO_A, Constants.ArmSubsystemConstants.ARM_ENCODER_DIO_B);
    m_PivotEncoder.setReverseDirection(true);
    m_PivotEncoder.reset();
  }

  // Setup the encoders in simulation
  private void SetupEncodersSim() {
    m_PivotEncoderSim = new EncoderSim(m_PivotEncoder);
    m_PivotEncoderSim.setDistance(0); // Start at 0 ticks in simulation
  }

  // Setup PID controller
  private void SetupPIDController() {
    // Sets PID coefficients for the Pivot neo
    kP = 0.001;
    kI = 0;
    kD = 0;
    m_PivotPID = new PIDController(kP, kI, kD);
    m_PivotPID.setTolerance(Constants.ArmSubsystemConstants.ARM_PIVOT_PID_TOLERANCE);
  }

  // Setup the limit switches
  private void SetupSwitches() {
    m_EndSwitch = new DigitalInput(Constants.ArmSubsystemConstants.ARM_ENDSWITCH_DIO_ID);
    m_StartSwitch = new DigitalInput(Constants.ArmSubsystemConstants.ARM_STARTSWITCH_DIO_ID);
  }

  // Setup the limit switches in simulation  
  private void SetupSwitchesSim() {
    m_EndSwitchSim = new DIOSim(m_EndSwitch);
    m_StartSwitchSim = new DIOSim(m_StartSwitch);
  } 

  // Setup the IMU
  private void SetupIMU() {
    m_IMU = new Pigeon2(Constants.ArmSubsystemConstants.ARM_IMU_CAN_ID);
    m_IMU.clearStickyFaults();
    m_IMU.reset();
  }

  private void SetupSimulation() {
    // Set initial conditions for simulation here if needed
    m_ArmSim = new SingleJointedArmSim(
      m_PivotMotorSimType, 
      100, 
      1.0, 
      inchesToMeters(10), 
      0.0, 
      4.06, 
      false, 
      degreesToRadians(0));
  }

  private double degreesToRadians(double degrees) {
    return degrees * (Math.PI / 180);
  }

  private double inchesToMeters(double inches) {
    return inches * 0.0254;
  } 

  public void runSimulation() {
    // Update the simulation of the arm
    m_ArmSim.setInput(m_PivotMotor.get() * RobotController.getBatteryVoltage());
    m_ArmSim.update(0.02);
    m_ArmSim.getOutput();
    m_PivotMotorSim.setMotorCurrent(m_ArmSim.getCurrentDrawAmps());

    // Update the encoder simulation
    double angle = m_ArmSim.getAngleRads();
    double ticks = (angle / (2 * Math.PI)) * Constants.ArmSubsystemConstants.ARM_ENCODER_CPR; // Convert radians to ticks
    m_PivotEncoderSim.setDistance(ticks);

    // Update the limit switch simulations
    if (ticks <= Constants.ArmSubsystemConstants.ARM_ENCODER_SIM_AT_START_POSITION) {
      m_StartSwitchSim.setValue(true); // At start position
    } else {
      m_StartSwitchSim.setValue(false);
    }

    if (ticks >= Constants.ArmSubsystemConstants.ARM_ENCODER_SIM_AT_END_POSITION) {
      m_EndSwitchSim.setValue(true); // At end position
    } else {
      m_EndSwitchSim.setValue(false);
    }

    // Update the Mechanism2d visualization
    double angleDegrees = (Math.toDegrees(angle))-118;
    //System.out.println("Arm Angle (degrees): " + angleDegrees + " :: " + angle);
    
    m_arm_bar.setAngle((angleDegrees)); // Convert radians
  }

  // Returns true if the arm is at the end position
  public boolean isArmAtEnd() {
    boolean value = false;
    if (RobotBase.isSimulation()) {
      value = m_EndSwitchSim.getValue ();
    } else {
      // The physical limit switch is wired such that it returns false when pressed, so we need to invert the value
      value = !m_EndSwitch.get();
    }
    return value;
  }

  // Returns true if the arm is at the start position
  public boolean isArmAtStart() {
    return m_StartSwitch.get();
  }

  // Returns the current pivot tick/position from the encoder
  // Encoder: 2048 ticks equals 1 revolution or 360 degrees
  public double getPivotAngle() {
    return m_PivotEncoder.get() * 0.176;
  }

  // Returns the current raw tick count from the encoder.  These are ticks from the last ZERO position
  public double getPivotTicks() {
    return m_PivotEncoder.get();
  }

  // When called it resets encoder such that the current arm position becomes ZERO position
  public void resetPivotEncoder() {
    m_PivotEncoder.reset();
  }

  // Pivots the arm at the given speed
  public void pivot(double speed) {
    double governedSpeed = speed;//Constants.ArmSubsystemConstants.GetSafePivotSpeed(speed);
    System.out.println("Pivoting at speed: " + governedSpeed);
    m_PivotMotor.set(governedSpeed);
  }

  // Pivots based on requested setPoint
  public void movePivotWithPID(double currentDistance, double setPoint) {
    System.out.println("Current Distance: " + currentDistance);
    m_PivotMotor.set(
        MathUtil.clamp(
            m_PivotPID.calculate(currentDistance, setPoint),
            Constants.ArmSubsystemConstants.ARM_PIVOT_PID_MIN_OUTPUT,
            Constants.ArmSubsystemConstants.ARM_PIVOT_PID_MAX_OUTPUT));
  }

  // Pivot PID error
  public double getPivotPIDError() {
    return m_PivotPID.getError();
  }

  // Get PitvotPID at set point property
  public boolean getPivotPIDAtSetpoint() {
    return m_PivotPID.atSetpoint();
  }

  // Get current IMU angle
  public Double getIMUPitch() {
    return m_IMU.getPitch().getValueAsDouble();
  }

  // Get current IMU Yaw
  public double getIMUYaw() {
    return m_IMU.getYaw().getValueAsDouble();
  }

  // Get current IMU Roll
  public double getIMURoll() {
    return m_IMU.getRoll().getValueAsDouble();
  }

  // Get acceleration on X axis
  public double getAccelerationX() {
    return m_IMU.getAccelerationX().getValueAsDouble();
  }

  // Get gravity value on X axis
  public double getGravityX() {
    return m_IMU.getGravityVectorX().getValueAsDouble();  
  } 

  // Rest the Arm's IMU
  public void resetArmIMU() {
    m_IMU.reset();
  }
  
  // Returns a command that will pivot the arm at the given speed while scheduled
  public Command commandPivot(DoubleSupplier speed) {
    return new RunCommand(
      () -> this.pivot(speed.getAsDouble()),
      this);
  }

  // Returns a command that will reset the encoder position to zero
  public Command resetEncoder() {
    return new RunCommand(
      () -> this.resetPivotEncoder(),
      this);
  }

  // **WARNING** UNTESTED - need to verify once hardware is available
  public DIRECTION getPivotDirection() {
    if (m_PivotMotor.get() > 0) {
      return DIRECTION.FORWARD;
    } else {
      return DIRECTION.REVERSE;
    }
  }

  // Returns a command that will reset the IMU
  public Command resetIMU() {
    return new RunCommand(
      () -> this.resetArmIMU(),
      this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Pivot Angle", getPivotAngle());
    SmartDashboard.putNumber("Arm Pivot Ticks", getPivotTicks());
    SmartDashboard.putBoolean("Arm at Start", isArmAtStart());
    SmartDashboard.putBoolean("Arm at End", isArmAtEnd());
    SmartDashboard.putData("Arm Sim", m_mech2d);
    // SmartDashboard.putNumber("Arm Yaw", getIMUYaw());
    // SmartDashboard.putNumber("Arm Roll", getIMURoll());
    // SmartDashboard.putNumber("Arm Pitch", getIMUPitch());
    // SmartDashboard.putNumber("Arm Acceleration X", getAccelerationX());
    // SmartDashboard.putNumber("Arm Gravity X", getGravityX()); 
  }
}
