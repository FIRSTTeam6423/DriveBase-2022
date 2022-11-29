// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoDance;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.OperateDrive;
import frc.robot.subsystems.DriveUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveUtil driveUtil = new DriveUtil();

  private final OperateDrive operateDrive = new OperateDrive(driveUtil);

  private static XboxController driver;
  // private static XboxController operator;

  /**
   * Added a new object - JoystickButton
   * This one is used to Toggle the Climb Arm out and back.
   */
 

  public static SendableChooser<Byte> driveType;
  public static SendableChooser<Byte> noobMode;
  public static SendableChooser<String> teamColorChooser; 
  public final static Byte arcade = 0;
  public final static Byte tank = 1;
  public final static Byte curvature = 2;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driver = new XboxController(Constants.XBOX_DRIVER);

    driveType = new SendableChooser<>();
    driveType.setDefaultOption("Arcade", arcade);
    driveType.addOption("Tank", tank);
    driveType.addOption("Curvature", curvature);
    SmartDashboard.putData("Drive Type", driveType);
    
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
    
    autoChooser.setDefaultOption("Autoturn 2 Seconds", new AutoTurn(driveUtil, 1, 2));
    autoChooser.addOption("Autoturn 5 Seconds", new AutoTurn(driveUtil, 0.5, 5));
    autoChooser.addOption("AutoDrive 8 Seconds", new AutoDrive(driveUtil, .5 , 8));
    autoChooser.addOption("Dance", new AutoDance(driveUtil));

    SmartDashboard.putData("Autonomous Command", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**
     * Actually added code here this time.
     * First you instantiate your Button (toggleClimb).
     * Although you use the generic JoystickButton class here
     * be careful.  The second variable in the constructor does all the work.
     * I used the Button object in this class:
     *    edu.wpi.first.wpilibj.XboxController.Button;
     * However there are Button classes for PS4 game controllers and more!!!!
     * Careful what you choose!
     * 
     */
    
    /**
     * Could have done this any number of ways, a real command or an instant command.
     * I went with InstantCommand, just as an example.  It will work.  Much more lightweight
     * than a full blown Command class given we just want to toggle the state of something.
     * 
     * You will notice that we got rid of the OperateCommand() command class as it is
     * not needed in the case of an InstantCommand().
     * 
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }

  private void configureDefaultCommands(){
    driveUtil.setDefaultCommand(operateDrive);
  }

  public static double getDriverLeftXboxX(){
    return driver.getLeftX();
  }

  public static double getDriverLeftXboxY(){
    return driver.getLeftY();
  }

  public static double getDriverRightXboxX(){
    return driver.getRightX();
  }

  public static double getDriverRightXboxY(){
    return driver.getRightY();
  }

  public static double getDriverLeftXboxTrigger(){
    return driver.getLeftTriggerAxis();
  }

  public static double getDriverRightXboxTrigger(){
    return driver.getRightTriggerAxis();
  }

  public static boolean getDriverAButton(){
    return driver.getAButton();
  }

  public static boolean getDriverBButton(){
    return driver.getBButton();
  }

  public static boolean getDriverXButton(){
    return driver.getXButton();
  }

  public static boolean getDriverYButton(){
    return driver.getYButton();
  }

  public static boolean getDriverLeftBumper(){
    return driver.getLeftBumper();
  }

  public static boolean getDriverRightBumper(){
    return driver.getRightBumper();
  }

  public static boolean getDriverLeftStickButton(){
    return driver.getLeftStickButton();
  }  

  public static boolean getDriverRightStickButton(){
    return driver.getRightStickButton();
  } 
}
