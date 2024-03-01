// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  NetworkTable limelightTable;
  NetworkTableEntry tx, ty, ta;
      // how many degrees back is your limelight rotated from perfectly vertical?
      
  double limelightMountAngleDegrees = 25.0; 

      // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 13.0; 
  public LimelightSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
  }

  public double getLimelightX()
  {
    return tx.getDouble(0);
  }

  public double getLimelightY()
  {
    return ty.getDouble(0);
  }

  public double getLimelightArea()
  {
    return ta.getDouble(0);
    
  }

  /*public double findDistanceToTarget()
  {
    /*double goalHeightInches = 0;
    double angleToGoalDegrees = limelightMountAngleDegrees + getLimelightY();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0); 
    double dis = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    System.out.print(dis);
    //var array = limelightTable.getEntry("botpose_targetSpace").getDoubleArray(new double[6]);
    //double pos = array[0];
    SmartDashboard.putNumber("limelight dis", pos);
    return pos;*/

  //}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
