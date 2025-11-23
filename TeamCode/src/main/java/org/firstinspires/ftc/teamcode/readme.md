# 📘 **FTC Robot Codebase -- README**

Welcome to our FTC robot codebase!
This document explains how our code works, how to extend it, and
everything future programmers need to know to contribute.

------------------------------------------------------------------------

# 🔧 **Codebase Overview**

Our robot software is built using:

-   **Android Studio**
-   **FTC SDK**
-   **Limelight 3A**
-   **Modular subsystem architecture**

Directory structure:

    TeamCode/
    │
    ├── DriveTrain.java
    ├── AutoDrive.java
    ├── Constants.java
    ├── ImuUtil.java
    ├── Shooter.java
    ├── VisionAlign.java
    │
    ├── TeleOps/
    │   ├── GREENBOT.java
    │   └── TestRobot.java
    │
    └── Autos/
        ├── Shooting1.java
        ├── LimeLightAuto.java
        └── DriveOffLine.java

Each subsystem manages **one specific part** of the robot.
OpModes simply **combine** subsystems --- they contain minimal logic.

------------------------------------------------------------------------

# ⚙️ **Subsystems**

------------------------------------------------------------------------

## 🚗 **DriveTrain.java**

The main drive system for TeleOp and Vision.

### Features:

-   `driveFieldCentric(x, y, rx, heading, slow, fast, precision)`
-   `driveRobot(x, y, rx)`
-   Motor RPM monitoring (`getRPM()`)
-   Nudge movements (`nudgeRight()`, `nudgeLeft()`...)
-   Used in **GREENBOT**, **VisionAlign**, and testing.

------------------------------------------------------------------------

## 🧭 **ImuUtil.java**

Wrapper for the REV IMU.

### Provides:

-   `getHeadingRad()` (used for field-centric control)
-   `resetYaw()`
-   Consistent heading measurements for TeleOp & Auto

------------------------------------------------------------------------

## 🎯 **VisionAlign.java**

Our Limelight 3A helper.

### Features:

-   Limelight initialization & polling
-   Align to AprilTags (`aimStepRobotCentric()`)
-   Align + distance control (`aimAndApproachStepRobotCentric()`)
-   Autonomous utilities (`aimUntil()`, `aimAndApproachUntil()`)
-   Latest LL result (`latest()`)

### Uses Limelight data:

-   **tx** → turn error\
-   **ta** → distance (target area)

### Key constants stored in `Constants.java`:

-   `LL_K_TURN`
-   `LL_MIN_TURN`
-   `LL_K_FORWARD`
-   `LL_MIN_FORWARD`
-   `LL_TARGET_AREA`
-   `LL_APPROACH_TOL_TA`

------------------------------------------------------------------------

## 🔫 **Shooter.java**

Controls: - Flywheel spin - Intake & feeder - Single-shot
(`feedOne()`) - Reverse feed mode - Human player assist feed - Internal
timing for clean firing

------------------------------------------------------------------------

## 🧭 **AutoDrive.java**

Used for **legacy encoder-based autos**.

### Provides:

-   `driveStraightInches()`
-   `turnToHeadingDegrees()`
-   Simple timed drive methods

Not used in Limelight autos.

------------------------------------------------------------------------

# 🎮 **TeleOp Structure**

We use a structured "**Option Menu**" system in `TestRobot.java`:

Option   Purpose
  -------- -----------------------
**1**    Drive Train Testing
**2**    Shooter Testing
**3**    Vision Testing
**4**    Individual Motor Test

This keeps the code clean and allows fast testing during development.

------------------------------------------------------------------------

## **GREENBOT.java -- Main TeleOp**

⬆️⬆️⬆️ **Drivers use this during competition.**

Contains: - Field-centric driving
- IMU reset
- Shooter controls
- Assist movements
- Limelight assist (VisionAlign)
- Intake modes

------------------------------------------------------------------------

# 🤖 **Autonomous Structure**

Autos are built using three tools:

✔ Blind movements (backup, shift, small turns)
✔ VisionAlign for alignment & distance
✔ Shooter subsystem

### Example Auto (Shooting1)

1.  Back up so Limelight can see tag
2.  Align with tag
3.  Back up until `ta` reaches desired shooting distance
    (`LL_TARGET_AREA`)
4.  Shoot 1
5.  Move or park

### General Auto Layout

``` java
waitForStart();
if (isStopRequested()) return;

vision.start(hardwareMap);
drive.move();
vision.align();
shooter.spinUp();
shooter.feedOne();
```

------------------------------------------------------------------------

# 📡 **Limelight Guide (For New Programmers)**

Limelight returns:

-   `tx` --- horizontal angle to tag
-   `ty` --- vertical angle
-   `ta` --- size of tag in image (distance measurement)
-   `valid` --- whether a tag is detected

### How VisionAlign uses it:

-   **Turn** = `tx * LL_K_TURN`
-   **Forward/backward** = `(LL_TARGET_AREA - ta) * LL_K_FORWARD`
-   Constants push through drivetrain static friction.

### `LL_TARGET_AREA` = desired shooting distance

Tune this using **LLDebug**.

------------------------------------------------------------------------

# 🎯 **Tuning LL_TARGET_AREA**

1.  Place robot at desired shooting distance
2.  Run **LLDebug**
3.  Note the `ta` value
4.  Set inside Constants:

``` java
LL_TARGET_AREA = <measured ta>;
```

------------------------------------------------------------------------

# 🧪 **Testing Tools**

### **TestRobot.java**

Subsystem testing for: - Drive - Shooter - Vision - Individual motors

### **LLDebug.java**

Displays: - tx
- ty
- ta
- valid
- pipeline index

Use it to tune distance + alignment.

------------------------------------------------------------------------

# 🏗 **How to Add a New OpMode**

1.  Create a new `.java` file in `TeamCode`\
2.  Extend `LinearOpMode`\
3.  Construct subsystems:

``` java
DriveTrain drive = new DriveTrain(hardwareMap);
ImuUtil imu = new ImuUtil(hardwareMap);
Shooter shooter = new Shooter(hardwareMap);
VisionAlign vision = new VisionAlign(drive, imu);
```

4.  Call `waitForStart()`
5.  Write loop/auto steps

------------------------------------------------------------------------

# 💻 **Building & Deploying Code**

Two deployment methods:

------------------------------------------------------------------------

## **Option 1 --- USB-C**

1.  Plug laptop → Control Hub USB-C
2.  Android Studio → **Run ▶**
3.  Select
    **REV Robotics Control Hub v1.0**\
4.  Code installs

------------------------------------------------------------------------

## **Option 2 --- Wireless (REV Hardware Client)**

1.  Connect to robot WiFi
2.  Open REV Hardware Client
3.  Ensure Control Hub shows up
4.  Android Studio → Run ▶
5.  Select Control Hub

------------------------------------------------------------------------

# 📱 **Viewing OpModes on Driver Station**

OpModes appear under:

-   TeleOp
-   Autonomous
-   Test (if annotated)

------------------------------------------------------------------------

# 🎓 **Tips for New Programmers**

-   ❌ **Never** write blocking loops inside main loop
-   ✔ Use subsystem methods (clean OpModes)
-   ✔ Test in TestRobot before competition
-   ✔ Retune `LL_TARGET_AREA` when robot changes
-   ✔ Increase LL_MIN_FORWARD / LL_MIN_TURN if robot won't move
-   ✔ Communicate with mechanical team

------------------------------------------------------------------------

# 🚀 **Future Expansion Ideas**

-   [ ] Road Runner integration
-   [ ] Pure pursuit + vision
-   [ ] AprilTag ID-based targeting
-   [ ] PIDF shooter velocity control
-   [ ] LED driver signals
-   [ ] Limelight-based path adjustments
-   [ ] Multi-shot LL auto

------------------------------------------------------------------------

# 🎉 **Welcome to the Code Team!**

If you need help, ask a senior programmer or check the subsystem code.\
Everything is clean and documented so you can learn quickly.

Let's build the best robot we can! 💙
