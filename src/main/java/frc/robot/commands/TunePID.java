// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/*
 * Unused after PID tuning was completed.
 */
public class TunePID extends Command {
  /** Creates a new TunePID. */

  private final char type;
  private final boolean increase;

  public TunePID(char type, boolean increase) {
    this.type = type;
    this.increase = increase;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      switch (type) {
          case 'P' -> {
              if (increase) {
                  ArmConstants.kP += 0.02; // Increase P value
              } else {
                  ArmConstants.kP -= 0.02; // Decrease P value
              }
          }
          case 'I' -> {
              if (increase) {
                  ArmConstants.kI += 0.02; // Increase I value
              } else {
                  ArmConstants.kI -= 0.02; // Decrease I value
              }
          }
          case 'D' -> { 
              if (increase) {
                  ArmConstants.kD += 0.02; // Increase D value
              } else {
                  ArmConstants.kD -= 0.02; // Decrease D value
              }
          }
          default -> {
          }
      }
      CommandScheduler.getInstance().cancel(this); // Kill the command when its done
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
