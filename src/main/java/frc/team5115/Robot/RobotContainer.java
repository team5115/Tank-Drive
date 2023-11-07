package frc.team5115.Robot;

import static frc.team5115.Constants.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team5115.Classes.Software.*;

public class RobotContainer {
    private final Timer timer;
    private final Joystick joy;
    private final Drivetrain drivetrain;

    public RobotContainer() {
        joy = new Joystick(0);
        
        drivetrain = new Drivetrain();

        timer = new Timer();
        timer.reset();
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        new JoystickButton(joy, 1).onTrue(new InstantCommand(drivetrain :: toggleSlowMode));
    }
    public void startTeleop(){
        System.out.println("Starting teleop");
        drivetrain.resetEncoders();
    }

    public void disabledInit(){
        drivetrain.stop();
    }

    public void stopEverything(){
        drivetrain.stop();
    }

    public void teleopPeriodic(){
         
        double forward = -joy.getRawAxis(JOY_Y_AXIS_ID); // negated because Y axis on controller is negated
        double turn = joy.getRawAxis(JOY_Z_AXIS_ID);
        drivetrain.tankDrive(forward, turn);
    }
}
