// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FuelSim;
import frc.robot.Constants.ShooterConstants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Add your docs here. */
public class FuelLaunchSim {
    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> fieldSpeedsSupplier;
    private int fuelStored = 8;

    public FuelLaunchSim(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    }

    private Translation3d launchVel(double fuelVel, double hoodAngle) {
        Pose2d robot = poseSupplier.get();
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        double horizontalVel = Math.cos(Math.toRadians(hoodAngle)) * fuelVel;
        double verticalVel = Math.sin(Math.toRadians(hoodAngle)) * fuelVel;
        double xVel = horizontalVel * Math.cos(robot.getRotation().getRadians());
        double yVel = horizontalVel * Math.sin(robot.getRotation().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    public boolean canIntake() {
        return fuelStored < ShooterConstants.k_maxFuelStorage;
    }

    public void intakeFuel() {
        fuelStored++;
    }

    public void launchFuel(double vel, double angle) {
        if (fuelStored == 0)
            return;
        fuelStored--;
        Transform2d leftShotTransform = new Transform2d(0.213, 0.08, new Rotation2d());
        Pose2d shotLeft = poseSupplier.get().transformBy(leftShotTransform);

        Translation3d initialPosLeft = new Translation3d(shotLeft.getX(), shotLeft.getY(), 0.5);

        FuelSim.getInstance().spawnFuel(initialPosLeft, launchVel(vel, angle));

        if (fuelStored == 0)
            return;
        fuelStored--;

        Transform2d rightShotTransform = new Transform2d(0.213, -0.08, new Rotation2d());
        Pose2d shotRight = poseSupplier.get().transformBy(rightShotTransform);

        Translation3d initialPosRight = new Translation3d(shotRight.getX(), shotRight.getY(), 0.5);

        FuelSim.getInstance().spawnFuel(initialPosRight, launchVel(vel, angle));
    }

    public Command repeatedlyLaunchFuel(
            DoubleSupplier velSupplier, DoubleSupplier angleSupplier) {
        return Commands.runOnce(() -> launchFuel(velSupplier.getAsDouble(), angleSupplier.getAsDouble()))
                .andThen(Commands.waitSeconds(0.2))
                .repeatedly();
    }
}