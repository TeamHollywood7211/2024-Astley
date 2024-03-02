// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.LimelightHelpers;
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

  



  double targetX = 0;
  double targetY = 0;
  public ArmSubsystem() {
    ArmMotor.restoreFactoryDefaults();
    WristMotor.restoreFactoryDefaults();
    SmartDashboard.putNumber("Setpoint Amp Arm", 180);
    SmartDashboard.putNumber("Setpoint Amp Wrist", 9.02);

    SmartDashboard.putNumber("Setpoint Mid", 43.6);
    SmartDashboard.putNumber("Setpoint Long", 60.4);

    SmartDashboard.putNumber("Setpoint ExLong", 53);

    SmartDashboard.putNumber("Setpoint Climb", 186);


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
     Pose3d pos = LimelightHelpers.getBotPose3d("limelight");

     SmartDashboard.putNumber("Limelight X", pos.getX());
     SmartDashboard.putNumber("Limelight Y", pos.getY());

    SmartDashboard.putNumber("wrist Pos", wristEncoder.getPosition());
    SmartDashboard.putNumber("Arm Pos", armEncoder.getPosition());

    SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint);
    SmartDashboard.putNumber("ArmSetpoint", armSetpoint);

    armSetpoint = MathUtil.clamp(armSetpoint,0,186);

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
    
    wristSetpoint = SmartDashboard.getNumber("Setpoint Amp Wrist",9.02);
    armSetpoint = SmartDashboard.getNumber("Setpoint Amp Arm",180);
    RobotContainer.shooterSpeed = 0.2;
  }

  public void posMid()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint Mid", 43.6);
    RobotContainer.shooterSpeed = 0.66;
  }

  public void posLong()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint Long", 60.4); //56.47;  
    RobotContainer.shooterSpeed = 1;
  }

  public void posExLong()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint ExLong", 53);
    RobotContainer.shooterSpeed = 1;
  }
  public void posClimb()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint Climb", 186);
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


  public void calcAngle()
  {
    Pose3d pos = LimelightHelpers.getBotPose3d("limelight");
    double botX = pos.getX();  
    double botY = pos.getY();  


    DriverStation.getAlliance().ifPresent((allianceColor) -> {
          if(allianceColor == Alliance.Red)
          {
            targetX = -8.305;
            targetY = 1.562;
          }
          else
          {
            targetX = 8.305;
            targetY = 1.325;
          }
    });
    double distance = Math.sqrt(((targetX-botX) * (targetX-botX)) + ((targetY - botY) * (targetY - botY)));

    //You cant ^ in Java :pensive:
    armSetpoint =  -0.705625 * (distance*distance) + 14.4712* distance + -46.5725;
  }

  public void testCalcAngle()
  {
    //armSetpoint = calcAngle();
    wristSetpoint = 0;
  }





}



