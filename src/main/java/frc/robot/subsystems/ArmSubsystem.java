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
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  //Assigns PID
  PIDController armPID = new PIDController(ArmConstants.armP, ArmConstants.armI, ArmConstants.armD);
  PIDController wristPID = new PIDController(ArmConstants.wristP, ArmConstants.wristI, ArmConstants.wristD);

  //Assigns Motors
  CANSparkMax ArmMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
  CANSparkMax WristMotor = new CANSparkMax(ArmConstants.wristMotorID, MotorType.kBrushless);
  //Assigns encoders from motor
  public RelativeEncoder armEncoder = ArmMotor.getEncoder();
  public RelativeEncoder wristEncoder = WristMotor.getEncoder();

  //Sets the setpoints to encoder positions so that we can redeploy code without issue
  double armSetpoint = armEncoder.getPosition(); //this makes it so when you repush code the robot doesnt get all wonky with arm pos
  double wristSetpoint = wristEncoder.getPosition();

  int invertArmPos = -1;



  double targetX = 0;
  double targetY = 0;
  public ArmSubsystem() {
    if(Constants.bot == 0)
    {
      invertArmPos = -1;
    }
    else
    {
      invertArmPos = 1;
    }
    ArmMotor.restoreFactoryDefaults();
    WristMotor.restoreFactoryDefaults();



    ArmMotor.setSmartCurrentLimit(40);
    WristMotor.setSmartCurrentLimit(40);

    SmartDashboard.putNumber("Setpoint Amp Arm", -180.14*invertArmPos); //Allows us to mid-comp change robot arm positions w/o redeploy 
    SmartDashboard.putNumber("Setpoint Amp Wrist", 18.14);

    SmartDashboard.putNumber("Setpoint Mid", -17.57*invertArmPos);
    SmartDashboard.putNumber("Setpoint Long", -38.28*invertArmPos);

    SmartDashboard.putNumber("Setpoint ExLong", -53*invertArmPos);

    SmartDashboard.putNumber("Setpoint Climb", -186*invertArmPos);
    SmartDashboard.putNumber("Setpoint offshot", -25.142*invertArmPos);

    SmartDashboard.putNumber("Setpoint CTF", -71.285*invertArmPos);


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
     Pose3d pos = LimelightHelpers.getBotPose3d("limelight"); //Limelight stuff

     SmartDashboard.putNumber("Limelight X", pos.getX()); //tells us the bots X,Y
     SmartDashboard.putNumber("Limelight Y", pos.getY());

    SmartDashboard.putNumber("wrist Pos", wristEncoder.getPosition()); //Tells us encoder positions
    SmartDashboard.putNumber("Arm Pos", armEncoder.getPosition());

    SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint); //Tells us setpoints
    SmartDashboard.putNumber("ArmSetpoint", armSetpoint);

    armSetpoint = MathUtil.clamp(armSetpoint,-186,0); //Locks the arm setpoint between its 0 and a 

    WristMotor.set(MathUtil.clamp(wristPID.calculate(wristEncoder.getPosition(), wristSetpoint), -0.75, 0.75)); //PID stuff I stole directly from the WPI website
    ArmMotor.set(MathUtil.clamp(armPID.calculate(armEncoder.getPosition(), armSetpoint), -0.75, 0.75));         //
  



  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void posZero() //Different positions for Arm (also changes speeds of motors)
  {
    wristSetpoint = 0;
    armSetpoint = 0;
    RobotContainer.shooterSpeed = 0.6;
  }
  public void posAmp()
  {
    
    wristSetpoint = SmartDashboard.getNumber("Setpoint Amp Wrist",19.14); //Pulls the values from SmartDashboard 
    armSetpoint = SmartDashboard.getNumber("Setpoint Amp Arm",-180.14*invertArmPos);
    RobotContainer.shooterSpeed = 0.2;
  }

  public void posMid()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint Mid", -17.57*invertArmPos);
    RobotContainer.shooterSpeed = 0.66;
  }

  public void posLong()
  {
    wristSetpoint = 0; //hey this is like 8.45 from target
    armSetpoint = SmartDashboard.getNumber("Setpoint Long", -38.28*invertArmPos); //56.47;  
    RobotContainer.shooterSpeed = 1;
  }

  public void posExLong()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint ExLong", -53*invertArmPos);
    RobotContainer.shooterSpeed = 1;
  }
  public void posClimb()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint Climb", -186*invertArmPos);
    RobotContainer.shooterSpeed = 1;
  }
  public void posOff()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint offshot", -25.142*invertArmPos);
    RobotContainer.shooterSpeed = 0.6;
  }
  public void posCross()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint CTF", -71.285*invertArmPos);
  }

  public void manuArm(double speed)
  {
    armSetpoint += speed/2;
  }

  public void manuWrist(double speed)
  {
    wristSetpoint -= speed/2;
  }


  public void calcAngle()
  {
    
    Pose3d pos = LimelightHelpers.getBotPose3d("limelight");
    double botX = pos.getX();  
    double botY = pos.getY();  //Gets X and Y from Limelight

    DriverStation.getAlliance().ifPresent((allianceColor) -> {
          if(allianceColor != Alliance.Red)
          {
            
            targetX = -8.305;
            targetY = 1.562;
            
          }
          else
          {
            targetX = 8.305;
            targetY = 1.325;
            
          }
    }); //Alliance position

    SmartDashboard.putNumber("target X", targetX);
    SmartDashboard.putNumber("target Y", targetY);
    double distance = Math.sqrt(((targetX-botX) * (targetX-botX)) + ((targetY - botY) * (targetY - botY))); 

    SmartDashboard.putNumber("Disntace to Target", distance);
    //You cant ^ in Java :pensive:
    armSetpoint = -6.62955 * (distance * distance) + 47.4922 * distance + -55.4878;
    
    //-0.705625 * (distance*distance) + 14.4712* distance + -46.5725;
  }

  public void testCalcAngle()
  {
    //armSetpoint = calcAngle();
    wristSetpoint = 0;
  }





}



