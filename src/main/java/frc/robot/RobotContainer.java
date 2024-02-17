package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.*;
import frc.robot.subsystems.Articulation.PoseEstimator;
import frc.robot.subsystems.swerve.rev.RevSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Subsystems */
//    private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  
    public static double shooterSpeed = 0.50;

    /* Controllers */
    private final Joystick driver = new Joystick(0);

    private final Joystick operator = new Joystick(1);

    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_operator = new CommandXboxController(1);
    private final CommandXboxController m_diagnostic = new CommandXboxController(2);

    private final MoveShooterCommand m_moveShooter = new MoveShooterCommand(shooterSubsystem, m_operator);

    private final diagnosticsCommand m_diagnosticCommand = new diagnosticsCommand(m_diagnostic, armSubsystem);

    private final IntakeShooterCommand m_intakeShooterCommand = new IntakeShooterCommand(intakeSubsystem, shooterSubsystem, m_operator);
    /* Commands */
    //private final ClimberCommand climberCommand = new ClimberCommand(m_ClimberSubsystem, m_driver);
    //private final ShooterCommand shooterCommand = new ShooterCommand(m_ShooterSubsystem, m_driver);


    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 4;
    //private final int speedDial = 5;

    
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 8);

    private final JoystickButton dampen = new JoystickButton(driver, 6);

    private final POVButton up = new POVButton(driver, 90);
    private final POVButton down = new POVButton(driver, 270);
    private final POVButton right = new POVButton(driver, 180);
    private final POVButton left = new POVButton(driver, 0);

    /* Subsystems */
    private final RevSwerve s_Swerve = new RevSwerve();
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();

    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();


        SmartDashboard.putData("Auto Chooser", autoChooser);


        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis)*-1, //those stinky drivers whine to much, so I flipped there right joystick >:3 (jk they complained about controls being flipped)
                () -> false, //Robot centric
                () -> dampen.getAsBoolean(),
                () -> 1 //speed multiplier 
            )
        );




        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
        
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

 
        //heading lock bindings
        up.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d90)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        left.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d180)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        right.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d0)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        down.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d270)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );

//    new Trigger(m_driver.povUp()).onTrue(climberCommand);
//    new Trigger(m_driver.povDown()).onTrue(climberCommand);

    //new Trigger(m_driver.rightTrigger(0.1)).onTrue(climberCommand);
    //new Trigger(m_driver.leftTrigger(0.1)).onTrue(climberCommand);
    //new Trigger(m_driver.rightBumper()).onTrue(shooterCommand);
    //new Trigger(m_driver.leftBumper()).onTrue(shooterCommand);
    //Arm Move Stuff

    //new Trigger(m_operator.a()).onTrue(new InstantCommand(armSubsystem::setPosLow)); 
    //new Trigger(m_operator.x()).onTrue(new InstantCommand(armSubsystem::setPosMid));
    //new Trigger(m_operator.y()).onTrue(new InstantCommand(armSubsystem::setPosHigh));



    //Shooter controls, right trigger shoots, right bumper retracts
    //Intake controls, left trigger intakes, left bumper outakes (retracts?)

    //SHOOTER//
    new Trigger(m_operator.rightTrigger()).onTrue(m_intakeShooterCommand);
    new Trigger(m_operator.rightBumper()).onTrue(m_intakeShooterCommand);
    
    new Trigger(m_operator.rightTrigger()).and(m_operator.rightBumper()).onFalse(m_intakeShooterCommand);



    

    new Trigger(m_operator.povDown()).onTrue(new InstantCommand(shooterSubsystem::shooterPosLow)); //For when you need the real zoomin' speed or smth and wanna 0 the bot.
    new Trigger(m_operator.y()).onTrue(new InstantCommand(shooterSubsystem::shooterPosSpeaker)); //For when we are next to the speaker thingy
    new Trigger(m_operator.x()).onTrue(new InstantCommand(shooterSubsystem::shooterPosLongShot)); //For when we are REALLLY far away
    new Trigger(m_operator.b()).onTrue(new InstantCommand(shooterSubsystem::shooterPosRingPickup)); //pickup
    new Trigger(m_operator.a()).onTrue(new InstantCommand(shooterSubsystem::shooterPosSuperLongShot));


    //INTAKE//
    new Trigger(m_operator.leftTrigger()).onTrue(m_intakeShooterCommand);
    new Trigger(m_operator.leftBumper()).onTrue(m_intakeShooterCommand);
    
    new Trigger(m_operator.leftTrigger()).or(m_operator.leftBumper()).onFalse(m_intakeShooterCommand);

    //Move Arm
    new Trigger(m_operator.rightStick()).whileTrue(m_moveShooter);


    //Diagnostics

    new Trigger(m_diagnostic.leftStick()).onTrue(m_diagnosticCommand);
    new Trigger(m_diagnostic.rightStick()).onTrue(m_diagnosticCommand);
    
    //Driver

    new Trigger(m_driver.button(7)).onTrue(new InstantCommand(s_Swerve::zeroGyro)); //Imma be doin' both buttons since those driver goobers dont know what button to press and complain
    new Trigger(m_driver.button(8)).onTrue(new InstantCommand(s_Swerve::zeroGyro)); //

    /* 
    new Trigger(m_diagnostic.rightTrigger()).whileTrue(new InstantCommand(armSubsystem::rightManualUp));
    new Trigger(m_diagnostic.rightBumper()).whileTrue(new InstantCommand(armSubsystem::rightManualDown));

    new Trigger(m_diagnostic.leftTrigger()).whileTrue(new InstantCommand(armSubsystem::leftManualDown)); //These are swapped due to motors being backwards
    new Trigger(m_diagnostic.leftBumper()).whileTrue(new InstantCommand(armSubsystem::leftManualUp));
    
    new Trigger(m_diagnostic.button(7)).onTrue(new InstantCommand(armSubsystem::resetOffsets)); //In case you really eff up and need to reset both offsets
    new Trigger(m_diagnostic.button(8)).onTrue(new InstantCommand(armSubsystem::resetZeros)); //Sets your zeros for when you un-eff up the robot
    */

}
    


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return null;
    }
}
