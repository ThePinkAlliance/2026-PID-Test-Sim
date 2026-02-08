// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.PivotArm;
import frc.robot.commands.PivotArmPID;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {

  public final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final CommandXboxController m_driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().whileTrue(
      new PivotArm(m_armSubsystem, () -> Constants.ArmSubsystemConstants.GetSafePivotSpeed(Constants.ArmSubsystemConstants.ARM_MAX_OPERATION_SPEED_POWER), 
      Constants.ArmSubsystemConstants.ARM_MAX_OPERATION_TIME_SECONDS)
    );
    
    m_driverController.b().whileTrue(
      new PivotArm(m_armSubsystem, () -> Constants.ArmSubsystemConstants.GetSafePivotSpeed(-Constants.ArmSubsystemConstants.ARM_MAX_OPERATION_SPEED_POWER), 
      Constants.ArmSubsystemConstants.ARM_MAX_OPERATION_TIME_SECONDS)
    );

    m_driverController.start().onTrue(
      m_armSubsystem.resetEncoder()
    );

    m_driverController.y().onTrue(
      new PivotArmPID(m_armSubsystem, Constants.ArmSubsystemConstants.ARM_PIVOT_PID_SET_POINT_DEPLOYED,
      Constants.ArmSubsystemConstants.ARM_MAX_OPERATION_TIME_SECONDS)
    );

    m_driverController.x().onTrue(
      new PivotArmPID(m_armSubsystem, Constants.ArmSubsystemConstants.ARM_PIVOT_PID_SET_POINT_STOWED,
      Constants.ArmSubsystemConstants.ARM_MAX_OPERATION_TIME_SECONDS)
    );
  }

  public ArmSubsystem getArmSubsystem() {
    return m_armSubsystem;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
