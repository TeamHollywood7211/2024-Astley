// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax IntakeMotor1 = new CANSparkMax(50, MotorType.kBrushless); //assigns the motors and stuff
  CANSparkMax IntakeMotor2 = new CANSparkMax(51, MotorType.kBrushless);

  CANSparkMax IntakeMotor3 = new CANSparkMax(49, MotorType.kBrushed); //assigns the motors and stuff

  
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    //IntakeMotor2.follow(IntakeMotor1); //use follow instead (controller groups are deprecated)
    IntakeMotor1.restoreFactoryDefaults();
    IntakeMotor2.restoreFactoryDefaults();


    IntakeMotor2.setInverted(false); //set true if needs to be inverted

  }

  public Command exampleMethodCommand() {

    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public void runGripSpeed(double speed1, double speed2)
  {
    IntakeMotor1.set(speed1); //sets the leader motor to the designated speed
    IntakeMotor2.set(speed2);
  }

  public void runRearSpeed(double speed3)
  {
    IntakeMotor3.set(speed3);
  }
}
