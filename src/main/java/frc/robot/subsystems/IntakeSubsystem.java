// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

  PIDController ArmAnglePID = new PIDController(IntakeConstants.armAngleP, IntakeConstants.armAngleI, IntakeConstants.armAngleD);
  PIDController ArmExtensionPID = new PIDController(IntakeConstants.armExtP, IntakeConstants.armExtI, IntakeConstants.armExtD);
  PIDController WristMotorPID = new PIDController(IntakeConstants.wristP, IntakeConstants.wristI, IntakeConstants.wristD);
  
  CANSparkMax IntakeMotor1 = new CANSparkMax(IntakeConstants.IntakeMotor1ID, MotorType.kBrushless);
  //CANSparkMax feederMotor = new CANSparkMax(IntakeConstants.feederMotorID, MotorType.kBrushed);
  CANSparkMax ArmAngleMotor = new CANSparkMax(IntakeConstants.IntakeArmAngleMotorID, MotorType.kBrushless);
  CANSparkMax ArmExtensionMotor = new CANSparkMax(IntakeConstants.IntakeArmExtensionMotorID, MotorType.kBrushless);
  CANSparkMax WristMotor = new CANSparkMax(IntakeConstants.IntakeWristMotorID, MotorType.kBrushless);

  RelativeEncoder ArmAngleEncoder = ArmAngleMotor.getEncoder();
  RelativeEncoder ArmExtensionEncoder = ArmExtensionMotor.getEncoder();
  RelativeEncoder WristEncoder = WristMotor.getEncoder();


  double angleSetpoint = 0;
  double extensionSetpoint = 0;
  double wristSetpoint = 0;

  DigitalInput ringSensor = new DigitalInput(IntakeConstants.IRSensorSignalID); //The sensor for the rings
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    IntakeMotor1.restoreFactoryDefaults();
    ArmAngleMotor.restoreFactoryDefaults();
    ArmExtensionMotor.restoreFactoryDefaults();
    WristMotor.restoreFactoryDefaults();
    //feederMotor.restoreFactoryDefaults();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Ring In Intake", !(ringSensor.get())); //Thing for the top sensor of the ring

    //SmartDashboard.putNumber("Angle Pos: ", ArmAngleEncoder.getPosition());
    //SmartDashboard.putNumber("Extension Pos: ", ArmExtensionEncoder.getPosition());
    //sSmartDashboard.putNumber("Wrist Pos: ", WristEncoder.getPosition());
    
    angleSetpoint = MathUtil.clamp(angleSetpoint, 0, 100);
    extensionSetpoint = MathUtil.clamp(extensionSetpoint, -50, 50);
    wristSetpoint = MathUtil.clamp(wristSetpoint, 0, 100);

    ArmAngleMotor.set(MathUtil.clamp(ArmAnglePID.calculate(ArmAngleEncoder.getPosition(), angleSetpoint), -0.75, 0.75));
    ArmExtensionMotor.set(MathUtil.clamp(ArmExtensionPID.calculate(ArmExtensionEncoder.getPosition(), extensionSetpoint), -0.75, 0.75));
    WristMotor.set(MathUtil.clamp(WristMotorPID.calculate(WristEncoder.getPosition(), wristSetpoint), -0.75, 0.75));

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }



  public void setIntake(double speed)
  {
    IntakeMotor1.set(-speed);
  }

  public void setFeeder(double speed)
  {
    //feederMotor.set(-speed);
  }
  public void moveIntakeAngle(double speed)
  {
    angleSetpoint += speed;
  }

  public void moveIntakeExtension(double speed)
  {
    extensionSetpoint += speed;
  }
  public void moveWristAngle(double speed)
  {
    wristSetpoint += speed;
  }


}
