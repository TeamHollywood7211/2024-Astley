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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  PIDController armPID = new PIDController(ArmConstants.armP, ArmConstants.armI, ArmConstants.armD);
  PIDController wristPID = new PIDController(ArmConstants.wristP, ArmConstants.wristI, ArmConstants.wristD);


  CANSparkMax ArmMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
  CANSparkMax WristMotor = new CANSparkMax(ArmConstants.wristMotorID, MotorType.kBrushless);
  
  public RelativeEncoder armEncoder = ArmMotor.getEncoder();
  public RelativeEncoder wristEncoder = WristMotor.getEncoder();


  double armSetpoint = armEncoder.getPosition(); //this makes it so when you repush code the robot doesnt get all wonky with arm pos
  double wristSetpoint = wristEncoder.getPosition();
  public ArmSubsystem() {
    ArmMotor.restoreFactoryDefaults();
    WristMotor.restoreFactoryDefaults();


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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("wrist Pos", wristEncoder.getPosition());
    SmartDashboard.putNumber("Arm Pos", armEncoder.getPosition());

    WristMotor.set(MathUtil.clamp(wristPID.calculate(wristEncoder.getPosition(), wristSetpoint), -0.75, 0.75));
    ArmMotor.set(MathUtil.clamp(armPID.calculate(armEncoder.getPosition(), armSetpoint), -0.75, 0.75));
  



  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void posZero()
  {
    wristSetpoint = 0;
    armSetpoint = 0;
    RobotContainer.shooterSpeed = 0.6;
  }
  public void posAmp()
  {
    wristSetpoint = 9.02;
    armSetpoint = 180;
    RobotContainer.shooterSpeed = 0.2;
  }

  public void posMid()
  {
    wristSetpoint = 0;
    armSetpoint = 43.6;
    RobotContainer.shooterSpeed = 0.66;
  }

  public void posLong()
  {
    wristSetpoint = 0;
    armSetpoint = 60.4; //56.47;  
    RobotContainer.shooterSpeed = 1;
  }

  public void posExLong()
  {
    wristSetpoint = 0;
    armSetpoint = 53;
    RobotContainer.shooterSpeed = 1;
  }
  public void posClimb()
  {
    wristSetpoint = 0;
    armSetpoint = 186;
    RobotContainer.shooterSpeed = 1;
  }

  public void manuArm(double speed)
  {
    armSetpoint += speed/2;
  }

  public void manuWrist(double speed)
  {
    wristSetpoint += speed/2;
  }


  public double calcAngle(double distance)
  {

    return -0.705625 * (distance*distance) + 14.4712* distance + -46.5725;
  }

  public void testCalcAngle()
  {
    //armSetpoint = calcAngle();
    wristSetpoint = 0;
  }
}



