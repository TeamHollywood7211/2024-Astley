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
import frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
  PIDController pid = new PIDController(ArmConstants.armP, ArmConstants.armI, ArmConstants.armD);
  CANSparkMax ArmMotor1 = new CANSparkMax(ArmConstants.armMotor1ID, MotorType.kBrushless); //assigns the motors and stuff
  CANSparkMax ArmMotor2 = new CANSparkMax(ArmConstants.armMotor2ID, MotorType.kBrushless);
  public RelativeEncoder armEncoder1 = ArmMotor1.getEncoder();
  public RelativeEncoder armEncoder2 = ArmMotor2.getEncoder();
  double setpoint = 0;
  double leftOffset = 0;
  double rightOffset = 0;
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
    
    ArmMotor1.set(MathUtil.clamp(pid.calculate(armEncoder1.getPosition(), setpoint+leftOffset), -0.5, 0.5));
    ArmMotor2.set(MathUtil.clamp(pid.calculate(armEncoder2.getPosition(), (-setpoint)+rightOffset), -0.5, 0.5));
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

  public void leftManualUp()
  {
    leftOffset++;
    System.out.print("L: " + leftOffset);
  }
  public void leftManualDown()
  {
    leftOffset--;
    System.out.print("L: " + leftOffset);
  }



  public void rightManualUp()
  {
    rightOffset++;
    System.out.print("R: " + rightOffset);
  }
  
  public void rightManualDown()
  {
    rightOffset--;
    System.out.print("R: " + rightOffset);
  }
  
  public void resetOffsets()
  {
    rightOffset = 0;
    leftOffset = 0;
  }
  public void resetZeros()
  {
    armEncoder1.setPosition(0);
    armEncoder2.setPosition(0);
    resetOffsets();
  }

}
