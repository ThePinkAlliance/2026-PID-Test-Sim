// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotArmPID extends Command {
  private ArmSubsystem armSubsystem;
  private double setPoint;
  private double timeoutSeconds;
  private Timer watchDogTimer;

  /** Creates a new PivotArm. */
  public PivotArmPID(ArmSubsystem armSubsystem, double setPoint, double timeoutSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.setPoint = setPoint;
    this.timeoutSeconds = timeoutSeconds;
    this.watchDogTimer = new Timer();
    addRequirements(this.armSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start watchdog timer.  Note: MUST reset before starting.
    watchDogTimer.reset();
    watchDogTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.movePivotWithPID(armSubsystem.getPivotTicks(), setPoint);
    System.out.println("PID Error: " + armSubsystem.getPivotPIDError());
    System.out.println("PID Arrived: " + armSubsystem.getPivotPIDAtSetpoint());
    System.out.println("CMD Watchdog: " + watchDogTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.pivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false because PID move is supposed to hold
    return armSubsystem.getPivotPIDAtSetpoint() || watchDogTimer.hasElapsed(timeoutSeconds);
  }
}
