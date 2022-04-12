/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.robot.ComputerVision;

import static java.lang.Math.abs;


@Autonomous(name="AutonomousBot", group="Autonomous")
//@Disabled
public class AutonomousBot extends LinearOpMode {
    protected CompetitionBot robot;

    @Override
    public void runOpMode() {
        robot = new CompetitionBot(hardwareMap, telemetry);

        fullVisionAuto();
        //visionTestAuto();
    }

    public void fullVisionAuto() {
        ComputerVision vision = new ComputerVision(hardwareMap, telemetry);

        waitForStart();

        int temp = -1;
        if (vision.pipeline.position == ComputerVision.TeamElementPipeline.ElementPosition.MIDDLE) { temp = 0; }
        else if (vision.pipeline.position == ComputerVision.TeamElementPipeline.ElementPosition.RIGHT) { temp = 1; }

        forwardBetter(.3,3.7,.05, telemetry);
        sleep(500);
        turnByBetter(.3,272, .05, telemetry);
        backwardBetter(.3, 13, .02, telemetry);
        sleep(500);

        double currentAngle = robot.getPitch();

        robot.setMotors(.1,.1);
        sleep(600);
        robot.setMotors(0,0);

        robot.DuckWheel.setPower(.8);
        sleep(2500);
        robot.DuckWheel.setPower(0);

        //if (abs(currentAngle-robot.getPitch()) > 2) { turnByBetter(.2,(currentAngle-robot.getPitch()-2)%360, .03, telemetry); }
        turnBy(.4,2.2, telemetry);
        forwardBetter(.4,38,.1, telemetry);
        turnByBetter(.3, 267, .05, telemetry);

        if (temp == -1) {
            backwardBetter(.25, 12, .05, telemetry);
            robot.setMotors(.1,.1);
            sleep(500);
            robot.setMotors(0,0);

            //forwardBetter(.2, .5, .03, telemetry);

            robot.LinearSlide.setPower(.8);
            sleep(1000);
            robot.LinearSlide.setPower(0);
            robot.BoxServo.setPosition(.25);
            sleep(1000);
            robot.BoxServo.setPosition(CompetitionBot.BOX_VERT);
            robot.LinearSlide.setPower(-.8);
            sleep(900);
            robot.LinearSlide.setPower(0);

            forwardBetter(.3, 4, .05, telemetry);
            turnByBetter(.2,85, .02, telemetry);
            forward(.8, 60, telemetry);
        }
        if (temp == 0) {
            backwardBetter(.25, 12, .05, telemetry);
            robot.setMotors(.1,.1);
            sleep(500);
            robot.setMotors(0,0);
            sleep(500);

            forwardBetter(.2, 3.5, .03, telemetry);

            robot.LinearSlide.setPower(.8);
            sleep(1000);
            robot.LinearSlide.setPower(0);
            robot.BoxServo.setPosition(.25);
            sleep(1000);
            robot.BoxServo.setPosition(CompetitionBot.BOX_VERT);
            sleep(1000);
            robot.LinearSlide.setPower(-.8);
            sleep(1000);
            robot.LinearSlide.setPower(0);

            forwardBetter(.3, 2.5, .05, telemetry);
            turnByBetter(.2,85, .02, telemetry);
            forward(.8, 60, telemetry);
        }
        if (temp == 1) {
            backwardBetter(.25, 12, .05, telemetry);
            robot.setMotors(.1,.1);
            sleep(500);
            robot.setMotors(0,0);
            sleep(500);

            forwardBetter(.2, 4.7, .03, telemetry);

            robot.LinearSlide.setPower(.8);
            sleep(600);
            robot.LinearSlide.setPower(0);
            robot.BoxServo.setPosition(.25);
            sleep(1000);
            robot.BoxServo.setPosition(CompetitionBot.BOX_VERT);
            sleep(1000);
            robot.LinearSlide.setPower(-.8);
            sleep(500);
            robot.LinearSlide.setPower(0);

            forwardBetter(.3, 2, .05, telemetry);
            turnByBetter(.2,85, .02, telemetry);
            forward(.8, 60, telemetry);
        }
    }

    public void redCarouselAuto(){
        forward(.3, 3.5, telemetry);
        turnBy(.4,273, telemetry);
        backward(.2, 12, telemetry);
        robot.setMotors(.12,.12);

        sleep(500);
        robot.setMotors(0,0);

        robot.DuckWheel.setPower(CompetitionBot.DUCK_POWER);
        sleep(2500);
        robot.DuckWheel.setPower(0);

        turnBy(.4,2.2, telemetry);
        forward(.3,41, telemetry);
        turnBy(.4,267, telemetry);
        backward(.25, 12, telemetry);
        robot.setMotors(.1,.1);
        sleep(500);
        robot.setMotors(0,0);

        robot.LinearSlide.setPower(.8);
        sleep(1000);
        robot.LinearSlide.setPower(0);
        robot.BoxServo.setPosition(.25);
        sleep(1000);
        robot.BoxServo.setPosition(CompetitionBot.BOX_VERT);
        robot.LinearSlide.setPower(-.8);
        sleep(900);
        robot.LinearSlide.setPower(0);

        turnBy(.2,80, telemetry);
        forward(.8, 60, telemetry);
    }

    public void visionTestAuto(){
        ComputerVision vision = new ComputerVision(hardwareMap, telemetry);

        waitForStart();

        int temp = -1;
        if (vision.pipeline.position == ComputerVision.TeamElementPipeline.ElementPosition.MIDDLE) { temp = 0; }
        else if (vision.pipeline.position == ComputerVision.TeamElementPipeline.ElementPosition.RIGHT) { temp = 1; }

        telemetry.addData("Vision: ", temp);
        telemetry.update();

        sleep(10000);
    }

    // Autonomous Motion

    public void forward(double speed, double inchDistance, Telemetry telemetry) {
        int stepCount = (int) (inchDistance*CompetitionBot.IN_TO_POS);
        int avgPos = (int) avgMotorPos();
        int initPos = avgPos;
        int referencePos = avgPos;
        int targetPos = avgPos + stepCount;

        double initTime = System.currentTimeMillis();
        double initCollisionTime = 0;
        double currentTime;
        double currentVel = 0;

        boolean collisionDetected = false;

        double rampedSpeed;
        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) avgMotorPos();

            currentTime = System.currentTimeMillis();
            telemetry.addData("time: ", currentTime - initTime);

            // Calculating encoder velocity and checking for collision
            if (currentTime - initTime > 100) {
                currentVel = abs(avgPos - referencePos) / .1;
                initTime = currentTime;
                referencePos = avgPos;
            }
            telemetry.addData("currentVel: ", currentVel);
            if (!collisionDetected && currentVel < 50) {
                initCollisionTime = currentTime;
                collisionDetected = true;
            }
            if (collisionDetected && currentVel > 100) {
                collisionDetected = false;
            }
            if (collisionDetected && currentTime - initCollisionTime > 400) {
                break;
            }

            rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .1, true);

            telemetry.addData("rampedSpeed: ", rampedSpeed);
            telemetry.update();

            robot.setMotors(-clamp(rampedSpeed), -clamp(rampedSpeed));
        }
        stopMotors(250);
    }

    public void backward(double speed, double inchDistance, Telemetry telemetry) {
        int stepCount = (int) (inchDistance*CompetitionBot.IN_TO_POS);
        int avgPos = (int) avgMotorPos();
        int initPos = avgPos;
        int referencePos = avgPos;
        int targetPos = avgPos - stepCount;

        double initTime = System.currentTimeMillis();
        double initCollisionTime = 0;
        double currentTime;
        double currentVel = 0;

        boolean collisionDetected = false;

        double rampedSpeed;
        while (opModeIsActive() && avgPos > targetPos) {
            avgPos = (int) avgMotorPos();

            currentTime = System.currentTimeMillis();
            telemetry.addData("time: ", currentTime - initTime);

            // Calculating encoder velocity and checking for collision
            if (currentTime - initTime > 100) {
                currentVel = abs(avgPos - referencePos) / .1;
                initTime = currentTime;
                referencePos = avgPos;
            }
            telemetry.addData("currentVel: ", currentVel);
            if (!collisionDetected && currentVel < 50) {
                initCollisionTime = currentTime;
                collisionDetected = true;
            }
            if (collisionDetected && currentVel > 100) {
                collisionDetected = false;
            }
            if (collisionDetected && currentTime - initCollisionTime > 400) {
                break;
            }

            rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .1, true);

            telemetry.addData("rampedSpeed: ", rampedSpeed);
            telemetry.update();

            robot.setMotors(clamp(rampedSpeed), clamp(rampedSpeed));
        }
        stopMotors(250);
    }

    public void turnBy(double maxSpeed, double deltaAngle, Telemetry telemetry) {
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double speed = maxSpeed;
        double dir = 1;
        double rampStart = .75;


        while (opModeIsActive() && Math.abs(diff) > 1) {
            currentAngle = robot.getPitch();
            diff = angleDiff(currentAngle, targetAngle);

            if (diff < 0) dir = -1;
            else dir = 1;

            if (Math.abs(diff) < rampStart * deltaAngle) {
                if (Math.abs(diff) < 2) speed = .125;
                else speed = Math.min(maxSpeed * (Math.abs(diff) / deltaAngle),.75);
            }

            speed *= dir;

            robot.LFmotor.setPower(speed);
            robot.LBmotor.setPower(speed);
            robot.RFmotor.setPower(-speed);
            robot.RBmotor.setPower(-speed);

            telemetry.addData("gyro: ", robot.getPitch());
            telemetry.addData("diff: ", diff);
            telemetry.addData("speed: ", speed);
            telemetry.addData("Alex_test: ", speed);
            telemetry.update();

        }
        stopMotors(250);
    }

    private double rampSpeed(double currentVal, double initVal, double targetVal, double speed, double minSpeed, boolean linearRamp) {
        double range = abs(targetVal - initVal);
        double currentDist = abs(currentVal - initVal);
        double remainingDist = abs(targetVal - currentVal);
        double rampRange = .75*range;

        telemetry.addData("dist: ", Math.round(currentDist / range * 100.0) / 100.0);

        if (currentDist <= range) {
            if (remainingDist <= rampRange) {
                return (remainingDist / rampRange) * (speed - minSpeed) * (linearRamp ? 1 : (speed - minSpeed)) + minSpeed;
            }
        } else {
            stopMotors(250);
            return 0;
        }
        return speed;
    }

    private void stopMotors(int durationTime) {
        robot.LFmotor.setTargetPosition(robot.LFmotor.getCurrentPosition());
        robot.LBmotor.setTargetPosition(robot.LBmotor.getCurrentPosition());
        robot.RFmotor.setTargetPosition(robot.RFmotor.getCurrentPosition());
        robot.RBmotor.setTargetPosition(robot.RBmotor.getCurrentPosition());

        robot.LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setMotors(.75, .75);

        sleep(durationTime);

        robot.setMotors(0, 0);

        robot.LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double avgMotorPos() { return ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0); }

    private double angleDiff(double angle1, double angle2) {
        double d1 = angle2 - angle1;
        if (d1 > 180) {
            return d1 - 360;
        } else if (d1 < -180) {
            return d1 + 360;
        } else {
            return d1;
        }
    }

    private double clamp(double power) {
        // ensures power does not exceed abs(1)
        if (power > 1) {
            return 1;
        }
        if (power < -1) {
            return -1;
        }
        return power;
    }

    private void forwardBetter(double speed, double inchDistance, double minSpeed, Telemetry telemetry){
        int stepCount = (int) (inchDistance*CompetitionBot.IN_TO_POS);
        int avgPos = (int) avgMotorPos();
        int initPos = avgPos;
        int targetPos = avgPos + stepCount;

        double rampedSpeed, realSpeed;
        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) avgMotorPos();

            rampedSpeed = rampBetter(avgPos, initPos, targetPos, speed, true);

            if (rampedSpeed < speed/50) { break; }

            telemetry.addData("rampedSpeed: ", rampedSpeed);
            telemetry.update();

            realSpeed = -Math.max(clamp(rampedSpeed), minSpeed);
            robot.setMotors(realSpeed, realSpeed);
        }
        stopMotors(250);
    }

    private void backwardBetter(double speed, double inchDistance, double minSpeed, Telemetry telemetry){
        int stepCount = (int) (inchDistance*CompetitionBot.IN_TO_POS);
        int avgPos = (int) avgMotorPos();
        int initPos = avgPos;
        int targetPos = avgPos - stepCount;

        double rampedSpeed, realSpeed;
        while (opModeIsActive() && avgPos > targetPos) {
            avgPos = (int) avgMotorPos();

            rampedSpeed = rampBetter(avgPos, initPos, targetPos, speed, true);

            if (rampedSpeed < speed/50) { break; }

            telemetry.addData("rampedSpeed: ", rampedSpeed);
            telemetry.update();

            realSpeed = Math.max(clamp(rampedSpeed), minSpeed);
            robot.setMotors(realSpeed, realSpeed);
        }
        stopMotors(250);
    }

    private double rampBetter(double currentVal, double initVal, double targetVal, double speed, boolean linearRamp) {
        double range = abs(targetVal - initVal);
        double currentDist = abs(currentVal - initVal);
        double remainingDist = abs(targetVal - currentVal);
        double rampRange = .75*range;

        telemetry.addData("dist: ", Math.round(currentDist / range * 100.0) / 100.0);

        if (currentDist <= range) {
            if (remainingDist <= rampRange) {
                return (remainingDist / rampRange) * speed * (linearRamp ? 1 : speed);
            }
        } else {
            stopMotors(250);
            return 0;
        }
        return speed;
    }

    public void turnByBetter(double maxSpeed, double deltaAngle, double minSpeed, Telemetry telemetry) {
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double speed = maxSpeed;
        double dir = 1;
        double rampStart = .75;


        while (opModeIsActive() && Math.abs(diff) > 1) {
            currentAngle = robot.getPitch();
            diff = angleDiff(currentAngle, targetAngle);

            if (diff < 0) dir = -1;
            else dir = 1;

            if (Math.abs(diff) < rampStart * deltaAngle) {
                if (Math.abs(diff) < 2) speed = minSpeed;
                else speed = Math.min(maxSpeed * (Math.abs(diff) / deltaAngle),.75);
            }

            speed *= dir;

            robot.LFmotor.setPower(speed);
            robot.LBmotor.setPower(speed);
            robot.RFmotor.setPower(-speed);
            robot.RBmotor.setPower(-speed);

            telemetry.addData("gyro: ", robot.getPitch());
            telemetry.addData("diff: ", diff);
            telemetry.addData("speed: ", speed);
            telemetry.update();

        }
        stopMotors(250);
    }
}
