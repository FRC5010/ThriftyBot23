// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveForward extends CommandBase {
  private final double HEADING_DIVIDER_PERCENT = 360.0;
  private DrivetrainSubsystem drivetrainSubsystem;
  private double speed;
  private double initialHeading;
  /** Creates a new DriveForward. */
  public DriveForward(DrivetrainSubsystem drivetrainSubsystem, double speed) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialHeading = drivetrainSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading = drivetrainSubsystem.getHeading();
    double error = heading - initialHeading;
    System.out.print("Error: "+error);
    System.out.print(" Percent: "+error/HEADING_DIVIDER_PERCENT);
    System.out.println(" Speed: "+speed);
    drivetrainSubsystem.drive(speed, error/HEADING_DIVIDER_PERCENT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
