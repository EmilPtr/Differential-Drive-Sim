// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.RotateArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /*
   * psuedocode:
   * init drivetrain
   * init arm
   * init controller at port 0
   * set default command to ArcadeDriveCommand with drivetrain and controller
   * rotate arm to 90 degrees when A is pressed
   * rotate arm to 0 degrees when B is pressed
   * increase kP when X is pressed
   * increase kI when Y is pressed
   * increase kD when right bumper is pressed
   * decrease kP when left bumper is pressed
   * decrease kI when POV up is pressed
   * decrease kD when POV down is pressed
   */

  private final DriveSubsystem m_drive = new DriveSubsystem(); 
  private final ArmSubsystem m_arm = new ArmSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // TODO: Insert your default command here...
    m_drive.setDefaultCommand(new ArcadeDriveCommand(m_drive, m_driverController));
    Trigger rotateArm90 = new Trigger(
      m_driverController.a().onTrue(new RotateArmCommand(m_arm, 90))
      );
    Trigger rotateArm0 = new Trigger(
      m_driverController.b().onTrue(new RotateArmCommand(m_arm, 0))
      );
    Trigger rotateArm45 = new Trigger(
      m_driverController.x().onTrue(new RotateArmCommand(m_arm, 45))
      );
    /* Trigger increaseKP = new Trigger(
      m_driverController.x().onTrue(new TunePID('P', true))
    );
    Trigger increaseKI = new Trigger(
      m_driverController.y().onTrue(new TunePID('I', true))
    );
    Trigger increaseKD = new Trigger(
      m_driverController.rightBumper().onTrue(new TunePID('D', true))
    );
    Trigger decreaseKP = new Trigger(
      m_driverController.leftBumper().onTrue(new TunePID('P', false))
    );
    Trigger decreaseKI = new Trigger(
      m_driverController.povUp().onTrue(new TunePID('I', false))
    );
    Trigger decreaseKD = new Trigger(
      m_driverController.povDown().onTrue(new TunePID('D', false))
    ); */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
