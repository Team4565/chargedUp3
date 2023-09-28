// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ChangeDriveMode;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drivetotag;
import frc.robot.commands.locateCube;
import frc.robot.commands.targetFinding;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EncodersSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem();
  private final EncodersSubsystem m_encoderRightSubsystem = new EncodersSubsystem();
  

  private final Command m_driveForSeconds = 
  Autos.driveForSeconds(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsTwo = 
  Autos.driveForSecondsTwo(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsThree = 
  Autos.driveForSecondsThree(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsFour = 
  Autos.driveForSecondsFour(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsFive = 
  Autos.driveForSecondsFive(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsSix = 
  Autos.driveForSecondsSix(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsSeven = 
  Autos.driveForSecondsSeven(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsEight = 
  Autos.driveForSecondsEight(m_drivetrainSubsystem);

  private final Command m_driveForSecondsNine = 
  Autos.driveForSecondsNine(m_drivetrainSubsystem);

  private final Command m_driveForSecondsTen = 
  Autos.driveForSecondsTen(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsEleven = 
  Autos.driveForSecondsEleven(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsTwelve = 
  Autos.driveForSecondsTwelve(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsThirteen = 
  Autos.driveForSecondsThirteen(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsFourteen = 
  Autos.driveForSecondsFourteen(m_drivetrainSubsystem, m_encoderRightSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsFifteen = 
  Autos.driveForSecondsFifteen(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveForSecondsSixteen = 
  Autos.driveForSecondsSixteen(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_doNothing = 
  Autos.doNothing(m_drivetrainSubsystem);
  
  // private final Command m_driveForwardBackwardBalance = 
  // Autos.forwardsBackwardsBalance(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveToTag = 
  new drivetotag(m_drivetrainSubsystem, m_visionSubsystem);

  private final Command m_turn = 
  Autos.turn(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_turnTwo = 
  Autos.turnTwo(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_gyro =
  Autos.gyro(m_gyroSubsystem, m_drivetrainSubsystem);

  private final Command m_test =
  Autos.test(m_drivetrainSubsystem);



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  public SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    //Moves Robot Using Joysticks
    m_drivetrainSubsystem.setDefaultCommand(new RunCommand(() ->
     m_drivetrainSubsystem.teleopDrive(m_driverController.getLeftY(), m_driverController.getRightX()), m_drivetrainSubsystem));
    // m_drivetrainSubsystem.teleopDrive(m_driverController.getLeftY(), m_driverController.getLeftX()), m_drivetrainSubsystem));
     configureBindings();


    m_chooser.addOption("Auto Set 1 (Forwards then left) - Charging Station + Mobility", m_driveForSeconds);
    m_chooser.addOption("Auto Set 2 (Forwards then right) - Charging Station + Mobility", m_driveForSecondsTwo);
    m_chooser.addOption("Auto Set 3 (Forwards then back) - Charging Station + Mobility", m_driveForSecondsThree);
    m_chooser.addOption("Auto Set 4 (Same as Set 1) - Charging Station + Mobility + Scoring", m_driveForSecondsFour);
    m_chooser.addOption("Auto Set 5 (Same as Set 2) - Charging Station + Mobility + Scoring", m_driveForSecondsFive);
    m_chooser.addOption("*MIDDLE* Auto Set 6 (Same as Set 3) - Charging Station + Mobility + Scoring", m_driveForSecondsSix);
    m_chooser.addOption("Auto Set 7 (Backwards then forward) - Scoring + Mobility", m_driveForSecondsSeven);
    m_chooser.addOption("Auto Set 7A Left Adjust (Backwards then forward) - Scoring + Mobility", m_driveForSecondsFifteen);
    m_chooser.addOption("Auto Set 7B Right Adjust (Backwards then forward) - Scoring + Mobility", m_driveForSecondsSixteen);
    m_chooser.addOption("Auto Set 8 (Forwards) - Mobility", m_driveForSecondsEight);
    m_chooser.addOption("Auto Set 9 (Backwards) - Scoring", m_driveForSecondsNine);
    m_chooser.addOption("Auto Set 10 (Same as Set 3 but w/weight) - Charging Station + Mobility + Scoring", m_driveForSecondsTen);
    m_chooser.addOption("Auto Set 11 (Same as Set 1 but w/weight) - Charging Station + Mobility", m_driveForSecondsEleven);
    m_chooser.addOption("Turning 180 fast", m_turn);
    m_chooser.addOption("Turning 180 fast v2", m_turnTwo);
    m_chooser.addOption("Balancing", m_gyro);
    m_chooser.addOption("Speed", m_test);
    //m_chooser.addOption("scoring.path", object);
    m_chooser.setDefaultOption("None", m_doNothing);
 // m_chooser.setDefaultOption("None", m_gyro);    
    SmartDashboard.putData(m_chooser);
  }

  String filename = "paths/scoring.path.wpilib.json";

    /** public Command loadPathplannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry) {
      Trajectory trajectory;
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException exception) {
        DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
        System.out.println("Unable to read from file" + filename);
        return new InstantCommand();
      }

      // RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrainSubsystem::getPose, null, null, null, null, null, null);
        
      }
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Spins Motor if April Tags are Recognized for 20 Ticks
    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new ChangeDriveMode(m_drivetrainSubsystem, "max"));
    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new ChangeDriveMode(m_drivetrainSubsystem, "fast"));
    new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(new ChangeDriveMode(m_drivetrainSubsystem, "normal"));
    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new ChangeDriveMode(m_drivetrainSubsystem, "creep speed (slow)"));
    new JoystickButton(m_driverController, XboxController.Button.kStart.value).onTrue(new ChangeDriveMode(m_drivetrainSubsystem, "no speed"));
  } 

  /* 
  creep speed is button 1 
  normal is button 3 
  fast is button 2 
  max is button 4 
  FOR THE NEWEST CONTROLLER
  */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return m_chooser.getSelected();
  }
}
