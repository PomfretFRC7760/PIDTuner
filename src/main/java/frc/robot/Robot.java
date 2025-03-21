package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;

public class Robot extends TimedRobot {
  private SparkMax motor1 = new SparkMax(5, MotorType.kBrushless);
  private SparkMax motor2 = new SparkMax(6, MotorType.kBrushless);
  private RelativeEncoder encoder1 = motor1.getEncoder();
  private RelativeEncoder encoder2 = motor2.getEncoder();
  private SparkClosedLoopController closedLoopController1 = motor1.getClosedLoopController();
  private SparkClosedLoopController closedLoopController2 = motor2.getClosedLoopController();
  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  private final CommandXboxController driverController = new CommandXboxController(
      0);
  private final SendableChooser<Boolean> manualControl = new SendableChooser<>();

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double targetPosition = 0;

  public Robot() {
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    // Default PID values
    kP = 0; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP)
        .i(kI)
        .d(kD)
        .iZone(kIz)
        .velocityFF(kFF)
        .outputRange(kMinOutput, kMaxOutput);

    motor1.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    motor2.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void robotInit() {
    // Add SmartDashboard buttons for tuning and running command
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Target Position", targetPosition);

    // Add a button to run the command
    SmartDashboard.putData("Run Motor", new InstantCommand(() -> run()));
    manualControl.setDefaultOption("Off", false);
    manualControl.addOption("On", true);

    SmartDashboard.putData("Manual Control", manualControl);
  }

  @Override
  public void robotPeriodic() {
    // Read PID values from SmartDashboard
    kP = SmartDashboard.getNumber("P Gain", kP);
    kI = SmartDashboard.getNumber("I Gain", kI);
    kD = SmartDashboard.getNumber("D Gain", kD);
    kIz = SmartDashboard.getNumber("I Zone", kIz);
    kFF = SmartDashboard.getNumber("Feed Forward", kFF);
    targetPosition = SmartDashboard.getNumber("Target Position", targetPosition);

    // Display actual velocity and position
    SmartDashboard.putNumber("Actual Velocity Motor 1", encoder1.getVelocity());
    SmartDashboard.putNumber("Actual Position Motor 1", encoder1.getPosition());
    SmartDashboard.putNumber("Actual Velocity Motor 2", encoder2.getVelocity());
    SmartDashboard.putNumber("Actual Position Motor 2", encoder2.getPosition());
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if (manualControl.getSelected()) {
      if (driverController.getLeftTriggerAxis() > 0 && driverController.getRightTriggerAxis() > 0) {
        motor1.set(0);
        motor2.set(0);
      } 
      else if (driverController.getLeftTriggerAxis() > 0) {
        motor1.set(-driverController.getLeftTriggerAxis()/2);
        motor2.set(-driverController.getLeftTriggerAxis()/2);
      }
      else {
        motor1.set(driverController.getRightTriggerAxis()/2);
        motor2.set(driverController.getRightTriggerAxis()/2);
      }
    }
    SmartDashboard.putNumber("Current P", kP);
    SmartDashboard.putNumber("Current I", kI);
    SmartDashboard.putNumber("Current D", kD);
    SmartDashboard.putNumber("Current Iz", kIz);
    SmartDashboard.putNumber("Current FF", kFF);
    SmartDashboard.putNumber("Current Target Position", targetPosition);
  }

  public void run() {
    // Update motor config with new PID values
    motorConfig.closedLoop
        .p(kP)
        .i(kI)
        .d(kD)
        .iZone(kIz)
        .velocityFF(kFF);

    motor1.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    motor2.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Reset encoder and run motor to target position
    encoder1.setPosition(0);
    encoder2.setPosition(0);
    closedLoopController1.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    closedLoopController2.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
