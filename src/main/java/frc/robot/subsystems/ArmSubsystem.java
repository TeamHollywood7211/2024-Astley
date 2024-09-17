// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.`command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.newClimberSubsystem;

public class ArmSubsystem extends SubsystemBase {
  //Assigns PID
  PIDController armPID = new PIDController(ArmConstants.armP, ArmConstants.armI, ArmConstants.armD);
  PIDController wristPID = new PIDController(ArmConstants.wristP, ArmConstants.wristI, ArmConstants.wristD);
  PIDController climber_wristPID = new PIDController(ArmConstants.cli_wristP, ArmConstants.cli_wristI, ArmConstants.cli_wristD);
  //Assigns Motors
  static CANSparkFlex ArmMotor = new CANSparkFlex(ArmConstants.armMotorID, MotorType.kBrushless);
  static CANSparkFlex WristMotor = new CANSparkFlex(ArmConstants.wristMotorID, MotorType.kBrushless);
  //Assigns encoders from motor
  public static RelativeEncoder armEncoder = ArmMotor.getEncoder();
  public static RelativeEncoder wristEncoder = WristMotor.getEncoder();


  
  PIDController pid = wristPID;

  //Sets the setpoints to encoder positions so that we can redeploy code without issue
  static double armSetpoint = armEncoder.getPosition(); //this makes it so when you repush code the robot doesnt get all wonky with arm pos
  static double wristSetpoint = wristEncoder.getPosition();

  int invertArmPos = -1;


  double armsHeighestPos = -200;

  double targetX = 0;
  double targetY = 0;
  public ArmSubsystem() {
    if(Constants.bot == 0) //This tells us what bot we use. (0 = practice, 1 = main)
    {
      invertArmPos = -1;
      armsHeighestPos = armsHeighestPos*-1;
    }
    else
    {
      invertArmPos = 1;
    }
    ArmMotor.restoreFactoryDefaults();
    WristMotor.restoreFactoryDefaults();

   //This leaves a height limit to the arms position
    

    ArmMotor.setSmartCurrentLimit(40);
    WristMotor.setSmartCurrentLimit(40);

    SmartDashboard.putNumber("Setpoint Amp Arm", -180.14*invertArmPos); //Allows us to mid-comp change robot arm positions w/o redeploy 
    SmartDashboard.putNumber("Setpoint Amp Wrist", 18.14);

    SmartDashboard.putNumber("Setpoint Mid", -24.64*invertArmPos); //-18.14
    SmartDashboard.putNumber("Setpoint Long", -48.28*invertArmPos);

    SmartDashboard.putNumber("Setpoint ExLong", -49.35*invertArmPos); 

    SmartDashboard.putNumber("Setpoint Climb", -195*invertArmPos); //-186 before
    SmartDashboard.putNumber("Setpoint offshot", -33.14*invertArmPos);

    SmartDashboard.putNumber("Setpoint CTF", 0*invertArmPos);

    SmartDashboard.putNumber("Setpoint HPS", -69*invertArmPos); //human player station


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

    armSetpoint = MathUtil.clamp(armSetpoint,armsHeighestPos,0); //Locks the arm setpoint between its 0 and a 
    //The above armSetpoint doesnt work for practice thanks to this clamp, please fix later (im at comp rn and dont wanna risk it)
    //((although it is a stupid easy fix just an fyi dont risk random things if theres no need to risk it.))



    /*if(newClimberSubsystem.armDown)
    {
      pid = climber_wristPID;
    }
    else
    {
      pid = armPID;
    }*/
    pid = wristPID;

    ArmMotor.set(MathUtil.clamp(armPID.calculate(armEncoder.getPosition(), armSetpoint), -1, 1));

    WristMotor.set(MathUtil.clamp(pid.calculate(wristEncoder.getPosition(), wristSetpoint), -0.75, 0.75)); //PID stuff I stole directly from the WPI website
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

    logArmPos(armSetpoint);
  }

  public void posAmp()
  {
    
    wristSetpoint = SmartDashboard.getNumber("Setpoint Amp Wrist",19.14); //Pulls the values from SmartDashboard 
    armSetpoint = SmartDashboard.getNumber("Setpoint Amp Arm",-180.14*invertArmPos);
    RobotContainer.shooterSpeed = 0.2;
    logArmPos(armSetpoint);
  }

  public void posMid()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint Mid", -18.14*invertArmPos);
    RobotContainer.shooterSpeed = 0.66;
    logArmPos(armSetpoint);
  }

  public void posLong()
  {
    wristSetpoint = 0; //hey this is like 8.45 from target
    armSetpoint = SmartDashboard.getNumber("Setpoint Long", -45.28*invertArmPos); //56.47;  
    RobotContainer.shooterSpeed = 1;
    logArmPos(armSetpoint);
  }

  public void posExLong()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint ExLong", -49.35*invertArmPos);
    RobotContainer.shooterSpeed = 1;
    logArmPos(armSetpoint);
  }

  public void posClimb()
  {
    wristSetpoint = 0; 
    armSetpoint = SmartDashboard.getNumber("Setpoint Climb", -195*invertArmPos);
    RobotContainer.shooterSpeed = 1;
    logArmPos(armSetpoint);
  }

  public void posOff()
  {
    wristSetpoint = 0;
    armSetpoint = SmartDashboard.getNumber("Setpoint offshot", -33.14*invertArmPos);
    RobotContainer.shooterSpeed = 0.6;
    logArmPos(armSetpoint);
  }
  public void posCross()
  {
    wristSetpoint = 0;
    RobotContainer.shooterSpeed = 0.44;
    armSetpoint = SmartDashboard.getNumber("Setpoint CTF", 0*invertArmPos);
    logArmPos(armSetpoint);
  }
  public void specFC3()
  {
    wristSetpoint = 0;
    armSetpoint = -28.05*invertArmPos;
    logArmPos(armSetpoint);
  }
  public void posNote2()
  {
    RobotContainer.shooterSpeed = 0.75; 
    wristSetpoint = 0;
    armSetpoint = -31.5*invertArmPos;
    logArmPos(armSetpoint);
  }

  public void posHPS()
  {
    wristSetpoint =  18.4;
    armSetpoint = -210.48;
    logArmPos(armSetpoint);
  }

  public void manuArm(double speed)
  {
    armSetpoint += speed/2;
  }

  public void manuWrist(double speed)
  {
    wristSetpoint -= speed/2;
  }

  public void logArmPos(double pos)
  {
    Robot.sentArmPos.append(pos);
  }


  public void calcAngle()
  {
    armSetpoint = 0;
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
    //double distance = Math.sqrt(((targetX-botX) * (targetX-botX)) + ((targetY - botY) * (targetY - botY))); 
    
    double distance = Math.sqrt(Math.pow(targetX-botX, 2) + Math.pow(targetY - botY, 2));

    //Math.pow() allows you to have exponents :3

    SmartDashboard.putNumber("Disntace to Target", distance);
    //armSetpoint = -6.62955 * (distance * distance) + 47.4922 * distance + -55.4878; //This calculation is used for the april tag auto aim for the arm
    
    armSetpoint = -0.974807 * (distance*distance) + -7.27931 * distance + 0.596068;

    //armSetpoint = 3.36827 * (distance * distance) + -18.1038 * distance + -0.37939;
    //-0.705625 * (distance*distance) + 14.4712* distance + -46.5725;
  }

  public void testCalcAngle()
  {
    //armSetpoint = calcAngle();
    wristSetpoint = 0;
  }


  //The following setpoints are specifically for trapping, 
  //and are designated with a "trap_", and the next one is
  //W=Wrist, A=Arm

  public void trap_WunderChain(){ //Gets you under the stage 
    armSetpoint = -38; 
  }
  public void trap_stage1() //Once under, both arm and wirst go to stage 1
  {
    armSetpoint = -68;
    wristSetpoint = 21;
  }
  public void trap_state2() //Second stage for safety
  {

  }

  
  





}



