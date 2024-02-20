package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
    public static final double stickDeadband = 0.1;;

    public static final class articulation {

        //articulator can IDs
        public static final int armLeft = 31;
        public static final int armRight = 32;

        public static final int intake = 33;
        
        public static final int shooter1 = 34; //bottom
        public static final int shooter2 = 35; //top

        //arm CANcoder ID
        public static final int armEncoder = 39;
        public static final double armEncoderOffset = 0;

        //Arm limits
        public static final int fwdLimit = 0;
        public static final int revLimit = 0;

        //Arm PID Constants
        public static final double armP = 0.0;
        public static final double armI = 0.0;
        public static final double armD = 0.0;
        public static final double armK = 0.0;

        //Shooter PID Constants
        public static final double shooterP = 0.0;
        public static final double shooterI = 0.0;
        public static final double shooterD = 0.0;
        public static final double shooterK = 0.0;

        //arm converstion math
        public static final double gearRatio = (5*5*5*(40/24));

        //Arm controller polling rate
        public static final double ScalingRatio = 2;
        

    }

    public static final class PoseEstimator{
        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> VisionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }
    public static class ArmConstants
    {
      public static final int armMotor1ID = 60;
      public static final int armMotor2ID = 61;
  
      public static final double armP = 0.01;
      public static final double armI = 0;
      public static final double armD = 0.001;
  
    }
    public static class IntakeConstants
    {
      public static final int IntakeMotor1ID = 41;
      public static final int IntakeMotor2ID = 42;
      public static final int feederMotorID = 43;
      public static final int IRSEnsorPowerID = 1;
      public static final int IRSensorSignalID = 0;
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
}
