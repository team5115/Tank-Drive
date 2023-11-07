package frc.team5115.Classes.Hardware;

import static frc.team5115.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.MathUtil;

public class HardwareDrivetrain {
    // Competition feedforward values - 6 inch diameter on KITT comp robot with arm
    // and ballasts
    private static final double leftKs = 0.0378;
    private static final double leftKv = 2.7479;
    private static final double leftKa = 0.32825;

    private static final double rightKs = 0.0528;
    private static final double rightKv = 2.8399;
    private static final double rightKa = 0.26071;

    private static final double leftKp = 0.0;
    private static final double rightKp = 0.0;
    private static final double Ki = 0.1;
    private static final double Kd = 0.1;
    // END of testbed values

    private static final double accelerationLimit = 1.5;

    private final SimpleMotorFeedforward leftFeedForward = new SimpleMotorFeedforward(leftKs, leftKv, leftKa);
    private final SimpleMotorFeedforward rightFeedForward = new SimpleMotorFeedforward(rightKs, rightKv, rightKa);
    private final PIDController leftPID = new PIDController(leftKp, Ki, Kd);
    private final PIDController rightPID = new PIDController(rightKp, Ki, Kd);

    private final CANSparkMax frontLeft = new CANSparkMax(FRONT_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax frontRight = new CANSparkMax(FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax backLeft = new CANSparkMax(BACK_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax backRight = new CANSparkMax(BACK_RIGHT_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = frontLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    private final RelativeEncoder rightEncoder = frontRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    public HardwareDrivetrain() {
        resetEncoders();
        frontRight.setInverted(true);
    }

    public double getEncoderDistance(int motorID) {
        switch (motorID) {
            case BACK_LEFT_MOTOR_ID:
            case FRONT_LEFT_MOTOR_ID:
                return leftEncoder.getPosition() * NEO_DISTANCE_CALIBRATION;

            case BACK_RIGHT_MOTOR_ID:
            case FRONT_RIGHT_MOTOR_ID:
                return rightEncoder.getPosition() * NEO_DISTANCE_CALIBRATION;

            default:
                throw new Error("Encoder ID " + motorID + " is invalid!");
        }
    }

    public Double getEncoderVelocity(int motorID) {
        switch (motorID) {
            case BACK_LEFT_MOTOR_ID:
            case FRONT_LEFT_MOTOR_ID:
                return leftEncoder.getVelocity() * NEO_VELOCITY_CALIBRATION;

            case BACK_RIGHT_MOTOR_ID:
            case FRONT_RIGHT_MOTOR_ID:
                return rightEncoder.getVelocity() * NEO_VELOCITY_CALIBRATION;

            default:
                throw new Error("Encoder ID " + motorID + " is invalid!");
        }
    }

    public void plugAndVoltDrive(double frontLeftVoltage, double frontRightVoltage, double backLeftVoltage, double backRightVoltage) {
        frontLeft.setVoltage(frontLeftVoltage);
        frontRight.setVoltage(frontRightVoltage);
        backLeft.setVoltage(backLeftVoltage);
        backRight.setVoltage(backRightVoltage);
    }

    /**
     * Sets the speeds of the motors. Uses feedforward but not PID. (right now PID
     * is broken)
     * 
     * @param leftSpeed  the speed for the left motors in meters per second
     * @param rightSpeed the speed for the right motors in meters per second
     */
    public void plugandFFDrive(double leftSpeed, double rightSpeed) {
        final double currentLeftVelocity = getLeftVelocity();
        final double currentRightVelocity = getRightVelocity();

        // limit left acceleration
        if (Math.abs(leftSpeed - currentLeftVelocity) > accelerationLimit) {
            leftSpeed = currentLeftVelocity + accelerationLimit * Math.signum(leftSpeed - currentLeftVelocity);
        }
        // limit right acceleration
        if (Math.abs(rightSpeed - currentRightVelocity) > accelerationLimit) {
            rightSpeed = currentRightVelocity + accelerationLimit * Math.signum(rightSpeed - currentRightVelocity);
        }

        double leftVoltage = leftFeedForward.calculate(leftSpeed);
        double rightVoltage = rightFeedForward.calculate(rightSpeed);
        leftVoltage += leftPID.calculate(currentLeftVelocity, leftSpeed);
        rightVoltage += rightPID.calculate(currentRightVelocity, rightSpeed);
        // Work on better PID Analyzer

        leftVoltage = MathUtil.clamp(leftVoltage, -DRIVE_MOTOR_MAX_VOLTAGE, DRIVE_MOTOR_MAX_VOLTAGE);
        rightVoltage = MathUtil.clamp(rightVoltage, -DRIVE_MOTOR_MAX_VOLTAGE, DRIVE_MOTOR_MAX_VOLTAGE);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);
        frontLeft.setVoltage(leftVoltage);
        frontRight.setVoltage(rightVoltage);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getLeftVelocity() {
        return leftEncoder.getVelocity() * NEO_VELOCITY_CALIBRATION;
    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity() * NEO_VELOCITY_CALIBRATION;
    }
}
