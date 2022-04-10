// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    // public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
    // SdsModuleConfigurations.MK4_L1.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI; // this is
    // roughly 4.116 for our setup
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.

    // public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
    // MAX_VELOCITY_METERS_PER_SECOND /
    // Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS /
    // 2.0); // This formula for our setup is ~13.2
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 6.0;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    // By default we use a Pigeon for our gyroscope. But if you use another
    // gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating
    // the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    // FIXME Remove if you are using a Pigeon

    // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
    private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(DRIVETRAIN_PIGEON_ID);

    // FIXME Uncomment if you are using a NavX
    // private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX
    // connected over MXP

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    // removed below as part of conversion to support odometry
    // private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // adding SwerveOdometry
    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());
    // private Pose2d m_odometryPose = new Pose2d();
    private SwerveModuleState[] m_swerveModuleStates = new SwerveModuleState[4]; // added while adding odometry
                                                                                 // support
                                                                                 // & replaced m_chassisSpeeds
    private Field2d m_field = new Field2d();

    private double m_lastRotationSpeed;

    public DrivetrainSubsystem() {
        m_swerveModuleStates[0] = new SwerveModuleState();
        m_swerveModuleStates[1] = new SwerveModuleState();
        m_swerveModuleStates[2] = new SwerveModuleState();
        m_swerveModuleStates[3] = new SwerveModuleState();

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        ShuffleboardTab fieldtab = Shuffleboard.getTab("Field");
        fieldtab.add(m_field).withWidget(BuiltInWidgets.kField)
                .withSize(8, 8)
                .withPosition(0, 0);

        // There are 4 methods you can call to create your swerve modules.
        // The method you use depends on what motors you are using.
        //
        // Mk3SwerveModuleHelper.createFalcon500(...)
        // Your module has two Falcon 500s on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createNeo(...)
        // Your module has two NEOs on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createFalcon500Neo(...)
        // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
        // and the NEO is for steering.
        //
        // Mk3SwerveModuleHelper.createNeoFalcon500(...)
        // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
        // Falcon 500 is for steering.
        //
        // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
        // class.

        // By default we will use Falcon 500s in standard configuration. But if you use
        // a different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        // FIXME Setup motor configuration
        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of
                // the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4SwerveModuleHelper.GearRatio.L1,
                // This is the ID of the drive motor
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case,
                // zero is facing straight forward)
                FRONT_LEFT_MODULE_STEER_OFFSET);

        // We will do the same for the other modules
        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L1,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L1,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET);

        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L1,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        // FIXME Remove if you are using a Pigeon
        // m_pigeon.setFusedHeading(0.0);
        m_pigeon.setYaw(0.0, 10);

        // FIXME Uncomment if you are using a NavX
        // m_navx.zeroYaw();
    }

    /**
     * updates the current gyro yaw. this update is asynchronous and while it waits
     * 10ms for the update, the actual update may take longer.
     * 
     * @param yawDegrees
     */
    public void setGyroScope(double yawDegrees) {
        m_pigeon.setYaw(yawDegrees, 10);
    }

    public void resetOdometry() {
        zeroGyroscope();
        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());
        // m_odometryPose =
        m_odometry.update(getGyroscopeRotation(), m_swerveModuleStates); // THIS IT???
    }

    public Rotation2d getGyroscopeRotation() {
        // FIXME Remove if you are using a Pigeon
        // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
        return Rotation2d.fromDegrees(m_pigeon.getYaw());

        // FIXME Uncomment if you are using a NavX
        // if (m_navx.isMagnetometerCalibrated()) {
        // // We will only get valid fused headings if the magnetometer is calibrated
        // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        // }
        //
        // // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    // clockwise rotoation is a positive change in angle
    public Rotation2d getGyroHeading() {
        Rotation2d r = new Rotation2d();
        r = r.fromDegrees(-m_pigeon.getYaw());
        return r;
    }

    private void updateSDSSwerveModules() {
        m_frontLeftModule.set(
                m_swerveModuleStates[0].speedMetersPerSecond /
                        MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                m_swerveModuleStates[0].angle.getRadians());
        m_frontRightModule.set(
                m_swerveModuleStates[1].speedMetersPerSecond /
                        MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                m_swerveModuleStates[1].angle.getRadians());
        m_backLeftModule.set(
                m_swerveModuleStates[2].speedMetersPerSecond /
                        MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                m_swerveModuleStates[2].angle.getRadians());
        m_backRightModule.set(
                m_swerveModuleStates[3].speedMetersPerSecond /
                        MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                m_swerveModuleStates[3].angle.getRadians());

    }

    /**
     * 
     * @param chassisSpeeds
     * @param rotationSpeed - used for simulation only
     */
    public void drive(ChassisSpeeds chassisSpeeds, double rotationSpeed) {
        m_swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                m_swerveModuleStates,
                MAX_VELOCITY_METERS_PER_SECOND);

        m_lastRotationSpeed = rotationSpeed;
        // update the actual swerve modules
        this.updateSDSSwerveModules();
    }

    // method created to facilitate autonomous using odometry
    public void setSwerveModulesStates(SwerveModuleState[] states) {
        m_swerveModuleStates = states;
        SwerveDriveKinematics.desaturateWheelSpeeds(
                m_swerveModuleStates,
                MAX_VELOCITY_METERS_PER_SECOND);

        // update the actual swerve modules
        this.updateSDSSwerveModules();
    }

    @Override
    public void periodic() {

        // Moved section below (the setting of the modules) into the drive method
        // and create a new method to set the swerve modules using the states

        // SwerveModuleState[] states =
        // m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        // SwerveDriveKinematics.desaturateWheelSpeeds(states,
        // MAX_VELOCITY_METERS_PER_SECOND);

        // m_frontLeftModule.set(states[0].speedMetersPerSecond /
        // MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        // states[0].angle.getRadians());
        // m_frontRightModule.set(states[1].speedMetersPerSecond /
        // MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        // states[1].angle.getRadians());
        // m_backLeftModule.set(states[2].speedMetersPerSecond /
        // MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        // states[2].angle.getRadians());
        // m_backRightModule.set(states[3].speedMetersPerSecond /
        // MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        // states[3].angle.getRadians());

        // update odometry
        // m_odometryPose =
        m_odometry.update(getGyroscopeRotation(), m_swerveModuleStates);

        // update field sim
        if (Robot.isSimulation()) {
            Pose2d simPose = new Pose2d(
                    getOdometryPose().getX(),
                    getOdometryPose().getY(),
                    new Rotation2d(m_lastRotationSpeed));
            m_field.setRobotPose(simPose);
        } else {
            m_field.setRobotPose(getOdometryPose());
        }

    }

    public Pose2d getOdometryPose() {
        return m_odometry.getPoseMeters();
    }

    public SwerveDriveOdometry getOdometry() {
        return m_odometry;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
