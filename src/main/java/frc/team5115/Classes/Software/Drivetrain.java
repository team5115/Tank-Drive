package frc.team5115.Classes.Software;

import static frc.team5115.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Acessory.ThrottleControl;
import frc.team5115.Classes.Hardware.HardwareDrivetrain;

public class Drivetrain extends SubsystemBase{

    private final ThrottleControl throttle;
    private final HardwareDrivetrain hardwareDrivetrain;

    public Drivetrain() {
        throttle = new ThrottleControl(3, -3, 0.2);
        hardwareDrivetrain = new HardwareDrivetrain();
    }

    public void stop() {
        hardwareDrivetrain.plugAndVoltDrive(0, 0, 0, 0);
    }

    public double getLeftDistance(){
        return hardwareDrivetrain.getEncoderDistance(BACK_LEFT_MOTOR_ID);
    }

    public double getRightDistance(){
        return hardwareDrivetrain.getEncoderDistance(BACK_RIGHT_MOTOR_ID);
    }

    public void resetEncoders() {
        hardwareDrivetrain.resetEncoders();
    }

    public void toggleThrottle(){
        throttle.toggleThrottle();
    }

    public void toggleSlowMode() {
        throttle.toggleSlowMode();
    }

    /**
     * enable or disable throttle. set to false to make throttle.getThrottle() return 0, true for normal function
     * @param enable true to allow stuff using throttle to move, false will just make getThrottle return 0
     */    
    public void setThrottleEnabled(boolean enable) {
        throttle.setThrottleEnabled(enable);
    }

    /**
     * Drive the robot using a tankdrive setup.
     * @param forward is for driving forward/backward: positive is forward, negative is backward
     * @param turn is for turning right/left: positive is right, negative is left
     */
    public void tankDrive(double forward, double turn) { 
        turn *= 0.7;

        double leftSpeed = (forward + turn);
        double rightSpeed = (forward - turn);
        
        double[] v = normalizeVector(leftSpeed, rightSpeed);
        leftSpeed = v[0];
        rightSpeed = v[1];

        leftSpeed *= throttle.getThrottle();
        rightSpeed *= throttle.getThrottle();
        hardwareDrivetrain.plugandFFDrive(leftSpeed, rightSpeed);
    }

    private double[] normalizeVector(double x, double y) {
        if(Math.abs(x) > 1){
            x = x/Math.abs(x);
            y = y/Math.abs(x);
        }
        else if (Math.abs(y) > 1){
            y = y/Math.abs(y);
            x = x/Math.abs(y);
        }
        return new double[] {x, y};
    }

    public boolean updateMoving(double dist, double startLeftDist, double startRightDist, double speedMagnitude) {
        final double remainingLeftDistance = startLeftDist + dist - getLeftDistance();
        final double remainingRightDistance = startRightDist + dist - getLeftDistance();

        final double speed = speedMagnitude * Math.signum((remainingLeftDistance + remainingRightDistance) / 2);
        hardwareDrivetrain.plugandFFDrive(speed, speed);

        final double tolerance = 0.05;
        return Math.abs(remainingLeftDistance) < tolerance || Math.abs(remainingRightDistance) < tolerance;
    }

}