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
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  Autos.driveForSecondsSeven(m_drivetrainSubsystem);

  private final Command m_driveForwardBackwardBalance = 
  Autos.forwardsBackwardsBalance(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_driveToTag = 
  new drivetotag(m_drivetrainSubsystem, m_visionSubsystem );

  private final Command m_turn = 
  Autos.turn(m_drivetrainSubsystem, m_gyroSubsystem);

  private final Command m_gyro =
  Autos.gyro(m_gyroSubsystem, m_drivetrainSubsystem);



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  public SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    //Moves Robot Using Joysticks
    m_drivetrainSubsystem.setDefaultCommand(new RunCommand(() ->
     m_drivetrainSubsystem.teleopDrive(m_driverController.getLeftY(), m_driverController.getLeftX()), m_drivetrainSubsystem));
    configureBindings();


    m_chooser.addOption("Auto Set 1 (Forward then left) - Charging Station ", m_driveForSeconds);
    m_chooser.addOption("Auto Set 2 (Forward then right) - Charging Station", m_driveForSecondsTwo);
    m_chooser.addOption("Auto Set 3 (Forward then back) - Charging Station", m_driveForSecondsThree);
    m_chooser.addOption("Auto Set 4 (Same as Set 1 but forwards and backwards at start) - Charging Station", m_driveForSecondsFour);
    m_chooser.addOption("Auto Set 5 (Straight then back) - Not used for Charging Station", m_driveForSecondsFive);
    m_chooser.addOption("Auto Set 6 (Backwards then left) - Charging Station", m_driveForSecondsSix);
    m_chooser.addOption("Auto Set 7 (Forward then back ) - Charging Station (Newest, not tested yet)", m_driveForwardBackwardBalance);
    m_chooser.addOption("Auto Set 8 (Backwards then forward - Scoring + Mobility", m_driveForSecondsSeven);
    m_chooser.setDefaultOption("None", m_gyro);
    SmartDashboard.putData(m_chooser);
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
  }

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
