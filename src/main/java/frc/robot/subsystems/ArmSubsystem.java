// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  PIDController pid = new PIDController(0.01, 0, 0.001);
  CANSparkMax ArmMotor1 = new CANSparkMax(Constants.ArmConstants.armMotor1ID, MotorType.kBrushless); //assigns the motors and stuff
  CANSparkMax ArmMotor2 = new CANSparkMax(Constants.ArmConstants.armMotor2ID, MotorType.kBrushless);
  public RelativeEncoder armEncoder1 = ArmMotor1.getEncoder();
  public RelativeEncoder armEncoder2 = ArmMotor2.getEncoder();
  double setpoint = 0;
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    ArmMotor1.restoreFactoryDefaults();
    ArmMotor2.restoreFactoryDefaults();
    
    
    //ArmMotor2.follow(ArmMotor1);


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
    SmartDashboard.putNumber("Arm Pos", armEncoder1.getPosition());
    
    ArmMotor1.set(MathUtil.clamp(pid.calculate(armEncoder1.getPosition(), setpoint), -0.5, 0.5));
    ArmMotor2.set(MathUtil.clamp(pid.calculate(armEncoder2.getPosition(), -setpoint), -0.5, 0.5));
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setPosLow()
  {
    setpoint = 0;
  }
  public void setPosMid()
  {
    setpoint = 100;
  }
  public void setPosHigh()
  {
    setpoint = 177;
  }

}
