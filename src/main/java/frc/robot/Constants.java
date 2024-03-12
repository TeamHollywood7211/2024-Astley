package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
    public static final double stickDeadband = 0.1;;

    public static class ArmConstants
    {
      public static final int armMotorID = 45; //fill in with actual thing
      public static final int wristMotorID = 47;
  
      public static final double armP = 0.025;
      public static final double armI = 0;
      public static final double armD = 0.0005;

      //wrist
      public static final double wristP = 0.02;
      public static final double wristI = 0;
      public static final double wristD = 0.003;
  
    }
    public static class IntakeConstants
    {
      public static final int IntakeMotor1ID = 41;
      public static final int IntakeMotor2ID = 42;
      public static final int feederMotor1ID = 43;
      public static final int feederMotor2ID = 44;
      public static final int IRSensorIntakePowerID = 0;
      public static final int IRSensorIntakeSignalID = 1;

      public static final int IRSensorShooterSignal = 2;
      public static final int IRSensorShooterSignalID = 3;
    }
    
    public static class ShooterConstants
    {
      public static final int ShooterMoverMotorID = 45;
      public static final double kP = 0.01;
      public static final double kI = 0;
      public static final double kD = 0.001;
      public static final int shooterMotor1ID = 50;
      public static final int shooterMotor2ID = 51;
      

      public static final int armMotorID = 45;
    }

    public static final int ledID = 59; 

    public static final int cameraResolution = 240;

    public static class ClimberConstants //Remove climber next time if need be
    {
      public static final double kP = 0.01;
      public static final double kI = 0;
      public static final double kD = 0.001;

      public static final int arm1ID = 60;
      public static final int arm2ID = 61;
    }

    public static final int CANdleID = 51;


}
