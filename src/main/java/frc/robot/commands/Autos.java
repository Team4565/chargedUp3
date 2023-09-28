// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EncodersSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static CommandBase driveForSeconds(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 4.3, -0.55),
      new TurnForAngle(gyro, drivetrain, -60, -0.55),
      new DriveForTime(drivetrain, 2, -0.55),
      new TurnForAngle(gyro, drivetrain, -60, -0.55),
      new DriveForTime(drivetrain, 1.5, -0.73),
      new Balance(gyro, drivetrain)
    );
    return group;
  }  

  public static CommandBase driveForSecondsTwo(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 5.5, -0.5),
      new TurnForAngle(gyro, drivetrain, 75, 0.55),
      new DriveForTime(drivetrain, 2.2, -0.55),
      new TurnForAngle(gyro, drivetrain, 78, 0.55),
      new DriveForTime(drivetrain, 1.45, -0.73),
      new Balance(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsThree(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 3.7, -0.66),
      new TurnForAngle(gyro, drivetrain, 152, 0.6),
      new DriveForTime(drivetrain, 1.3, -0.73),
      new Balance(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsFour(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.9, 0.65),
      new DriveForTime(drivetrain, 0.2, -0.43),
      new DriveForTime(drivetrain, 5.6, -0.5),
      new TurnForAngle(gyro, drivetrain, -75, -0.55),
      new DriveForTime(drivetrain, 2.5, -0.5),
      new TurnForAngle(gyro, drivetrain, -75, -0.55),
      new DriveForTime(drivetrain, 1.2, -0.65),
      new Balance(gyro, drivetrain)
    );
    return group;
  }  

  public static CommandBase driveForSecondsFive(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.9, 0.65),
      new DriveForTime(drivetrain, 0.2, -0.43),
      new DriveForTime(drivetrain, 5.5, -0.5),
      new TurnForAngle(gyro, drivetrain, 78, 0.55),
      new DriveForTime(drivetrain, 2.3, -0.5),
      new TurnForAngle(gyro, drivetrain, 79, 0.55),
      new DriveForTime(drivetrain, 1.45, -0.73),
      new Balance(gyro, drivetrain)
    );
    return group;
  }   

  public static CommandBase driveForSecondsSix(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.8, 0.60),
      new DriveForTime(drivetrain, 0.15, -0.3),
      new TurnForAngle(gyro, drivetrain, -3, -0.4),
      new DriveForTime(drivetrain, 3.25, -0.65),
      new TurnForAngle(gyro, drivetrain, 115, 0.7),
      new DriveForTime(drivetrain, 1.3, -0.70),
      new Balance(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsSeven(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.8, 0.65),
      new DriveForTime(drivetrain, 0.15, -0.43),
      new DriveForTime(drivetrain, 4, -0.6)
    );
    return group;
  }

  public static CommandBase driveForSecondsEight(DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 3.7, -0.60)
    ); 
    return group;
  }

  public static CommandBase driveForSecondsNine(DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.8, 0.65),
      new DriveForTime(drivetrain, 0.15, -0.40)
    );
    return group;
  }

  public static CommandBase driveForSecondsTen(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.8, 0.62),
      new DriveForTime(drivetrain, 0.2, -0.3),
      new DriveForTime(drivetrain, 3.5, -0.66),
      new TurnForAngle(gyro, drivetrain, 160, 0.6),
      new DriveForTime(drivetrain, 1.55, -0.70),
      new BalanceWithWeight(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsEleven(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 4.3, -0.55),
      new TurnForAngle(gyro, drivetrain, -60, -0.57),
      new DriveForTime(drivetrain, 2, -0.55),
      new TurnForAngle(gyro, drivetrain, -60, -0.57),
      new DriveForTime(drivetrain, 1.7, -0.73),
      new BalanceWithWeight(gyro, drivetrain)
    );
    return group;
  }
  
  public static CommandBase driveForSecondsTwelve(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 4.3, -0.55),
      new TurnForAngle(gyro, drivetrain, -60, -0.55),
      new DriveForTime(drivetrain, 2, -0.55),
      new TurnForAngle(gyro, drivetrain, -60, -0.55),
      new DriveForTime(drivetrain, 1.5, -0.73),
      new TurnForAngle(gyro, drivetrain, 75, 0.55)
      
    );
    return group;
  }

  public static CommandBase driveForSecondsThirteen(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 5.5, -0.5),
      new TurnForAngle(gyro, drivetrain, 75, 0.55),
      new DriveForTime(drivetrain, 2.2, -0.55),
      new TurnForAngle(gyro, drivetrain, 78, 0.55),
      new DriveForTime(drivetrain, 1.45, -0.73),
      new TurnForAngle(gyro, drivetrain, -60, -0.55)
    );
    return group;
  }

  public static CommandBase driveForSecondsFourteen(DrivetrainSubsystem drivetrain, EncodersSubsystem encoderRightSubsystem, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveWithEncoders(drivetrain, encoderRightSubsystem, 1.25),
      new TurnForAngle(gyro, drivetrain, -60, -0.55),
      new DriveWithEncoders(drivetrain, encoderRightSubsystem, 0.8),
      new DriveWithEncoders(drivetrain, encoderRightSubsystem, 0.3),
      new Balance(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsFifteen(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.8, 0.65),
      new DriveForTime(drivetrain, 0.25, -0.43),
      new TurnForAngle(gyro, drivetrain, -4, -0.6),
      new DriveForTime(drivetrain, 4, -0.6)
    );
    return group;
  }
  public static CommandBase driveForSecondsSixteen(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.8, 0.65),
      new DriveForTime(drivetrain, 0.25, -0.43),
      new TurnForAngle(gyro, drivetrain, 4, 0.6),
      new DriveForTime(drivetrain, 4, -0.6)
    );
    return group;
  }



  public static CommandBase doNothing(DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup();
    return group;
  }

  
  public static CommandBase gyro(GyroSubsystem m_gyro, DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new Balance(m_gyro, drivetrain)
    );
    return group; 
  }

  public static CommandBase turn(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new TurnForAngle(gyro, drivetrain, 5, 0.6)
    );
    return group;
  }

  public static CommandBase turnTwo(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new TurnForAngle(gyro, drivetrain, -5, -0.6)
    );
    return group;
  }

  public static CommandBase test(DrivetrainSubsystem drivetrain) {
    return Commands.sequence(new DriveForTime(drivetrain, 5, -0.1));
  }
  


/*public static CommandBase forwardsBackwardsBalance(DrivetrainSubsystem drivetrain, GyroSubsystem gyro){
  return new SequentialCommandGroup(
    new DriveStraightForTime(drivetrain,4.5, -0.6, gyro),
    new DriveStraightForTime(drivetrain,2,.6, gyro), 
    new Balance(gyro, drivetrain)
  );
} */

  private Autos () {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
