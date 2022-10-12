// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveUtil;

public class AutoTurn extends CommandBase {
  /** Creates a new AutoTurn. */
  private DriveUtil driveUtil;
  private Timer timer;
  private double turnTime;
  private double speed;
  public AutoTurn(DriveUtil du, double speed, double turnTime) {
    this.driveUtil = du;
    this.speed = speed;
    this.turnTime = turnTime;
    addRequirements(this.driveUtil);
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveUtil.tankDrive(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveUtil.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > turnTime;
  }
}
