// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GyroSubsystem;
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
      new DriveForTime(drivetrain, 5.5, -0.5),
      new TurnForAngle(gyro, drivetrain, -75, -0.55),
      new DriveForTime(drivetrain, 2.8, -0.5),
      new TurnForAngle(gyro, drivetrain, -75, -0.55),
      new DriveForTime(drivetrain, 1.2, -0.65),
      new Balance(gyro, drivetrain)
    );
    return group;
  }  

  public static CommandBase driveForSecondsTwo(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 5.3, -0.5),
      new TurnForAngle(gyro, drivetrain, 79, 0.55),
      new DriveForTime(drivetrain, 2, -0.5),
      new TurnForAngle(gyro, drivetrain, 78, 0.55),
      new DriveForTime(drivetrain, 1.2, -0.65),
      new Balance(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsThree(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
     new DriveForTime(drivetrain, 3.7, -0.66),
    new DriveForTime(drivetrain, 1.35, 0.8),
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
      new DriveForTime(drivetrain, 5.7, -0.5),
      new TurnForAngle(gyro, drivetrain, 78, 0.55),
      new DriveForTime(drivetrain, 2.3, -0.5),
      new TurnForAngle(gyro, drivetrain, 79, 0.55),
      new DriveForTime(drivetrain, 1.2, -0.65),
      new Balance(gyro, drivetrain)
    );
    return group;
  }   

  public static CommandBase driveForSecondsSix(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.9, 0.65),
      new DriveForTime(drivetrain, 0.2, -0.43),
      new DriveForTime(drivetrain, 3.7, -0.66),
      new DriveForTime(drivetrain, 1.35, 0.8),
      new Balance(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsSeven(DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.9, 0.65),
      new DriveForTime(drivetrain, 0.2, -0.43),
      new DriveForTime(drivetrain, 3.3, -0.55)
    );
    return group;
  }

  public static CommandBase driveForSecondsEight(DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 3, -0.65)
    );
    return group;
  }

  public static CommandBase driveForSecondsNine(DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 0.9, 0.65),
      new DriveForTime(drivetrain, 0.2, -0.43)
    );
    return group;
  }

  public static CommandBase gyro(GyroSubsystem m_gyro, DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new Balance(m_gyro, drivetrain)
    );
    return group; 
  }

  public static CommandBase turn(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    return Commands.sequence(new TurnForAngle(gyro, drivetrain, 88, 0.55));
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
