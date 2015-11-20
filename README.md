# GyroMountainBalancer
/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class GyroSafeGuard extends OpMode {

  private static final double MOTOR_POWER = .5;
  private static final int DEGREES_TOO_FAR = -10;

  private Servo swingArm;
  private DcMotor motorLeft;
  private DcMotor motorRight;
  private GyroSensor gyro;

  private double swingLeft;
  private double swingRight;
  private long timeLast;
  private double gyroOffset;
  private double degreesWeHaveTurned;


  @Override
  public void init() {

      swingArm = hardwareMap.servo.get("swingArm");
      motorRight = hardwareMap.dcMotor.get("MR");
      motorLeft = hardwareMap.dcMotor.get("ML");
      motorLeft.setDirection(DcMotor.Direction.REVERSE);

      gyro = hardwareMap.gyroSensor.get("gyro");


  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {

      swingLeft = 1;
      swingRight = 0;
      timeLast = System.currentTimeMillis();
      gyroOffset = gyro.getRotation();
      degreesWeHaveTurned = 0.0;

  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {

      long timeNow = System.currentTimeMillis();
      degreesWeHaveTurned += (gyro.getRotation() - gyroOffset) * (timeNow - timeLast) / 1000.0;
      timeLast = timeNow; //Says value timeNow assigned to timeLast is never used

      // If we suspect the degrees we have turned (based on the degrees per second) is
      // greater than 90, stop.
      if (degreesWeHaveTurned >= DEGREES_TOO_FAR) {
          swingArm.setPosition(swingLeft);
      }

      if (degreesWeHaveTurned < DEGREES_TOO_FAR) {
          swingArm.setPosition(swingRight);
      }

      telemetry.addData("deg/second", gyro.getRotation() - gyroOffset);
      telemetry.addData("TURN_AMOUNT", "We have possibly turned " + degreesWeHaveTurned);

  }
}
