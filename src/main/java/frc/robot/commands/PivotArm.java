// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmSubsystemConstants.DIRECTION;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotArm extends Command {
  private ArmSubsystem armSubsystem;
  private double speed;


  /** Creates a new PivotArm. */
  public PivotArm(ArmSubsystem armSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.speed = speed;
    addRequirements(this.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start watchdog timer.  Note: MUST reset before starting.\
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.pivot(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Place subsystem into a SAFE state
    armSubsystem.pivot(0);
  }

  // Returns true if either limit switch is triggered based on the direction of movement
  private boolean atSwitch() {
    boolean value = false;
    //going forward, check end switch
    if (armSubsystem.getPivotDirection() == DIRECTION.FORWARD && armSubsystem.isArmAtEnd()) {
      value = true;
    //going reverse, check start switch
    } else if (armSubsystem.getPivotDirection() == DIRECTION.REVERSE && armSubsystem.isArmAtStart()) {
      value = true;
    } else {
      //no switch triggered, keep going
      value = false;
    }
    return value;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //End command if either switch is true or a timeout occurs
    return false;
  }
}
