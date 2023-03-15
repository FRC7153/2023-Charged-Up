// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GameState;
import frc.robot.subsystems.DriveBase;

/**
 * Command to drive swerve drive base in teleop
 */
public class TeleopDriveCommand extends CommandBase {
  // Subsystems
  private DriveBase base;

  // Suppliers
  private Supplier<Double> xSupply;
  private Supplier<Double> ySupply;
  private Supplier<Double> rSupply;
  private Supplier<GameState> stateSupply;

  public TeleopDriveCommand(DriveBase swerveSubsystem, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rotSupplier, Supplier<GameState> stateSupplier) {
    base = swerveSubsystem;
    xSupply = xSupplier;
    ySupply = ySupplier;
    rSupply = rotSupplier;
    stateSupply = stateSupplier;

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!stateSupply.get().equals(GameState.TELEOP)) { cancel(); return; }
    base.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    base.driveRobotOriented(xSupply.get(), ySupply.get(), rSupply.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
