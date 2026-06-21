#include <Arduino.h>
#include <ESP32Encoder.h>

struct MotorPin
{
    uint8_t inA;
    uint8_t inB;
    uint8_t en;
    uint8_t pwmCh;
};

struct WheelValue
{
    float left;
    float right;
};

struct Pid
{
    float kp;
    float ki;
    float kd;
    float outLimit;
    float iLimit;
    float deadband;
    float integral = 0.0f;
    float lastErr = 0.0f;

    Pid(float kp_, float ki_, float kd_, float outLimit_, float iLimit_, float deadband_)
        : kp(kp_), ki(ki_), kd(kd_), outLimit(outLimit_), iLimit(iLimit_), deadband(deadband_)
    {
    }

    void reset()
    {
        integral = 0.0f;
        lastErr = 0.0f;
    }

    float compute(float target, float actual, float dt)
    {
        float rawErr = target - actual;
        float err = (fabsf(rawErr) < deadband) ? 0.0f : rawErr;

        if ((err > 0.0f && lastErr < 0.0f) || (err < 0.0f && lastErr > 0.0f))
        {
            integral *= 0.25f;
        }

        float candidateI = integral + (err * dt);
        candidateI = constrain(candidateI, -iLimit, iLimit);

        float d = 0.0f;
        if (dt > 0.0f)
        {
            d = (err - lastErr) / dt;
        }

        float unsat = (kp * err) + (ki * candidateI) + (kd * d);
        float out = constrain(unsat, -outLimit, outLimit);

        bool canIntegrate = (out == unsat) ||
                            (out >= outLimit && err < 0.0f) ||
                            (out <= -outLimit && err > 0.0f);

        if (canIntegrate)
        {
            integral = candidateI;
        }

        lastErr = err;
        return out;
    }
};

// ============================================================
// Pin and polarity config
// ============================================================
const MotorPin motorL = {25, 26, 33, 0};
const MotorPin motorR = {27, 14, 32, 1};

const uint8_t encLA = 18;
const uint8_t encLB = 19;
const uint8_t encRA = 17;
const uint8_t encRB = 16;

// Flip motorSign if a wheel physically runs backward for a positive command.
// Flip encSign if CPS is negative while that wheel is physically moving forward.
const int motorSignL = 1;
const int motorSignR = 1;
const int encSignL = 1;
const int encSignR = 1;

// ============================================================
// Robot and control config
// ============================================================
const int pwmFreq = 20000;
const int pwmBits = 8;
const uint32_t controlMs = 20;
const uint32_t cmdTimeoutMs = 2000;

const float wheelRadius = 0.03f;
const float wheelBase = 0.170f;
const float ticksPerRev = 1975.0f;
const float twoPi = 6.28318530718f;

const float stopCps = 8.0f;
const float cpsAlpha = 0.35f;
const float maxWheelMps = 1.00f;
const float cmdAccelCpsPerSec = 5000.0f;
const float cmdDecelCpsPerSec = 4500.0f;
const float stopHoldCps = 35.0f;
const uint32_t stopHoldMs = 300;
const int stopBrakePwmL = 190;
const int stopBrakePwmR = 230;

const int basePwm = 200;
const int minPwmL = 68;
const int minPwmR = 52;
const int minSpinPwmL = 115;
const int minSpinPwmR = 110;
const float baseCpsL = 1680.0f;
const float baseCpsR = 1950.0f;

const float syncKp = 0.16f;
const float syncKi = 0.030f;
const float syncILimit = 700.0f;
const int syncLimit = 130;
const int syncStep = 20;
const float syncAlpha = 0.15f;

const bool enableDebug = false;
const uint32_t debugMs = 200;
const float pidDeadbandCpsL = 25.0f;
const float pidDeadbandCpsR = 25.0f;

// ============================================================
// Runtime state
// ============================================================
ESP32Encoder encL;
ESP32Encoder encR;
Pid pidL(0.24f, 0.065f, 0.0f, 90.0f, 2600.0f, pidDeadbandCpsL);
Pid pidR(0.24f, 0.065f, 0.0f, 90.0f, 2600.0f, pidDeadbandCpsR);

WheelValue cmdMps = {0.0f, 0.0f};
WheelValue cmdCps = {0.0f, 0.0f};
WheelValue targetMps = {0.0f, 0.0f};
WheelValue targetCps = {0.0f, 0.0f};

long prevTickL = 0;
long prevTickR = 0;
uint32_t prevMs = 0;
uint32_t lastCmdMs = 0;
uint32_t lastDebugMs = 0;
uint32_t stopHoldUntilMs = 0;

float cpsL = 0.0f;
float cpsR = 0.0f;
float syncErrF = 0.0f;
float syncI = 0.0f;
int syncPwm = 0;
bool syncActive = false;
bool stopHoldActive = false;
long syncBaseTickL = 0;
long syncBaseTickR = 0;
int syncSignL = 0;
int syncSignR = 0;

String serialBuf;

// ============================================================
// Small helpers
// ============================================================
int roundInt(float value)
{
    return (value >= 0.0f) ? (int)(value + 0.5f) : (int)(value - 0.5f);
}

float lpf(float oldValue, float newValue, float alpha)
{
    return (alpha * oldValue) + ((1.0f - alpha) * newValue);
}

float mpsToCps(float mps)
{
    return (mps * ticksPerRev) / (twoPi * wheelRadius);
}

float cpsToMps(float cps)
{
    return (cps * twoPi * wheelRadius) / ticksPerRev;
}

float clampWheelMps(float mps)
{
    return constrain(mps, -maxWheelMps, maxWheelMps);
}

float rampToward(float now, float target, float maxStep)
{
    if (target > now + maxStep)
    {
        return now + maxStep;
    }

    if (target < now - maxStep)
    {
        return now - maxStep;
    }

    return target;
}

float rampRateFor(float now, float target)
{
    bool sameDirection = (now >= 0.0f && target >= 0.0f) ||
                         (now <= 0.0f && target <= 0.0f);
    bool slowingDown = sameDirection && fabsf(target) < fabsf(now);

    return slowingDown ? cmdDecelCpsPerSec : cmdAccelCpsPerSec;
}

int signForTarget(float target)
{
    return (target >= 0.0f) ? 1 : -1;
}

bool hasCommand()
{
    return fabsf(cmdCps.left) >= stopCps || fabsf(cmdCps.right) >= stopCps;
}

bool hasTarget()
{
    return fabsf(targetCps.left) >= stopCps || fabsf(targetCps.right) >= stopCps;
}

bool isSpinTarget()
{
    if (fabsf(targetCps.left) < stopCps || fabsf(targetCps.right) < stopCps)
    {
        return false;
    }

    return (targetCps.left > 0.0f) != (targetCps.right > 0.0f);
}

void resetPid()
{
    pidL.reset();
    pidR.reset();
    syncErrF = 0.0f;
    syncI = 0.0f;
    syncPwm = 0;
    syncActive = false;
    syncSignL = 0;
    syncSignR = 0;
}

void clearStopHold()
{
    stopHoldActive = false;
    stopHoldUntilMs = 0;
}

// ============================================================
// Motor control
// ============================================================
void setupMotor(const MotorPin &m)
{
    pinMode(m.inA, OUTPUT);
    pinMode(m.inB, OUTPUT);
    pinMode(m.en, OUTPUT);

    ledcSetup(m.pwmCh, pwmFreq, pwmBits);
    ledcAttachPin(m.en, m.pwmCh);
}

void writeMotor(const MotorPin &m, int pwm)
{
    pwm = constrain(pwm, -255, 255);

    if (pwm > 0)
    {
        digitalWrite(m.inA, LOW);
        digitalWrite(m.inB, HIGH);
        ledcWrite(m.pwmCh, pwm);
    }
    else if (pwm < 0)
    {
        digitalWrite(m.inA, HIGH);
        digitalWrite(m.inB, LOW);
        ledcWrite(m.pwmCh, -pwm);
    }
    else
    {
        digitalWrite(m.inA, LOW);
        digitalWrite(m.inB, LOW);
        ledcWrite(m.pwmCh, 0);
    }
}

void brakeMotor(const MotorPin &m, int pwm)
{
    pwm = constrain(pwm, 0, 255);
    digitalWrite(m.inA, HIGH);
    digitalWrite(m.inB, HIGH);
    ledcWrite(m.pwmCh, pwm);
}

void driveBoth(int pwmL, int pwmR)
{
    writeMotor(motorL, motorSignL * pwmL);
    writeMotor(motorR, motorSignR * pwmR);
}

void brakeBoth(int pwm)
{
    brakeMotor(motorL, pwm);
    brakeMotor(motorR, pwm);
}

void brakeBoth(int pwmL, int pwmR)
{
    brakeMotor(motorL, pwmL);
    brakeMotor(motorR, pwmR);
}

int feedPwm(float target, int minPwm, float baseCps)
{
    float mag = fabsf(target);
    if (mag < stopCps)
    {
        return 0;
    }

    int pwm = roundInt((mag / baseCps) * basePwm);
    pwm = constrain(pwm, minPwm, 255);

    return (target >= 0.0f) ? pwm : -pwm;
}

// ============================================================
// Target commands
// ============================================================
void stopRobot();

void setTargetWheelMps(float leftMps, float rightMps)
{
    cmdMps.left = clampWheelMps(leftMps);
    cmdMps.right = clampWheelMps(rightMps);

    cmdCps.left = mpsToCps(cmdMps.left);
    cmdCps.right = mpsToCps(cmdMps.right);

    lastCmdMs = millis();

    if (hasCommand())
    {
        clearStopHold();
    }
}

void setTargetCmdVel(float linearX, float angularZ)
{
    float leftMps = linearX - (angularZ * wheelBase * 0.5f);
    float rightMps = linearX + (angularZ * wheelBase * 0.5f);

    setTargetWheelMps(leftMps, rightMps);
}

void stopRobot()
{
    cmdMps = {0.0f, 0.0f};
    cmdCps = {0.0f, 0.0f};
    targetMps = {0.0f, 0.0f};
    targetCps = {0.0f, 0.0f};
    resetPid();
    stopHoldActive = true;
    stopHoldUntilMs = millis() + stopHoldMs;
    brakeBoth(stopBrakePwmL, stopBrakePwmR);
}

void updateTargetRamp(float dt)
{
    float maxStepL = rampRateFor(targetCps.left, cmdCps.left) * dt;
    float maxStepR = rampRateFor(targetCps.right, cmdCps.right) * dt;

    targetCps.left = rampToward(targetCps.left, cmdCps.left, maxStepL);
    targetCps.right = rampToward(targetCps.right, cmdCps.right, maxStepR);

    if (fabsf(targetCps.left) < stopCps && fabsf(cmdCps.left) < stopCps)
    {
        targetCps.left = 0.0f;
    }

    if (fabsf(targetCps.right) < stopCps && fabsf(cmdCps.right) < stopCps)
    {
        targetCps.right = 0.0f;
    }

    targetMps.left = cpsToMps(targetCps.left);
    targetMps.right = cpsToMps(targetCps.right);
}

void startStopHold(uint32_t now)
{
    stopHoldActive = true;
    stopHoldUntilMs = now + stopHoldMs;
    resetPid();
}

bool shouldHoldStop(uint32_t now)
{
    if (!stopHoldActive)
    {
        return false;
    }

    bool stillMoving = fabsf(cpsL) > stopHoldCps || fabsf(cpsR) > stopHoldCps;
    bool holdTimeLeft = (int32_t)(stopHoldUntilMs - now) > 0;

    if (stillMoving || holdTimeLeft)
    {
        return true;
    }

    clearStopHold();
    return false;
}

// ============================================================
// Sync helper for straight driving only
// ============================================================
int stepToward(int now, int target, int step)
{
    if (target > now + step)
    {
        return now + step;
    }

    if (target < now - step)
    {
        return now - step;
    }

    return target;
}

bool canSyncBalancedPair()
{
    if (fabsf(targetCps.left) < stopCps || fabsf(targetCps.right) < stopCps)
    {
        return false;
    }

    if ((targetCps.left > 0.0f) != (targetCps.right > 0.0f))
    {
        return false;
    }

    float magL = fabsf(targetCps.left);
    float magR = fabsf(targetCps.right);
    float maxMag = max(magL, magR);
    float tolerance = max(80.0f, 0.12f * maxMag);

    return fabsf(magL - magR) <= tolerance;
}

int calcSyncPwm(long tickL, long tickR, float dt)
{
    if (!canSyncBalancedPair())
    {
        syncErrF = lpf(syncErrF, 0.0f, 0.5f);
        syncI *= 0.8f;
        syncPwm = stepToward(syncPwm, 0, syncStep);
        syncActive = false;
        syncSignL = 0;
        syncSignR = 0;
        return syncPwm;
    }

    int signL = signForTarget(targetCps.left);
    int signR = signForTarget(targetCps.right);

    if (!syncActive || signL != syncSignL || signR != syncSignR)
    {
        syncActive = true;
        syncBaseTickL = tickL;
        syncBaseTickR = tickR;
        syncSignL = signL;
        syncSignR = signR;
        syncErrF = 0.0f;
        syncI = 0.0f;
        syncPwm = 0;
    }

    float progressL = (float)syncSignL * (float)(tickL - syncBaseTickL);
    float progressR = (float)syncSignR * (float)(tickR - syncBaseTickR);

    float err = progressR - progressL;
    syncErrF = lpf(syncErrF, err, syncAlpha);

    syncI += syncErrF * dt;
    syncI = constrain(syncI, -syncILimit, syncILimit);

    float out = (syncKp * syncErrF) + (syncKi * syncI);
    int target = constrain(roundInt(out), -syncLimit, syncLimit);

    syncPwm = stepToward(syncPwm, target, syncStep);
    return syncPwm;
}

// ============================================================
// Serial protocol
// ============================================================
bool parseTwoFloats(const String &line, float &a, float &b)
{
    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);

    if (p1 < 0 || p2 < 0)
    {
        return false;
    }

    a = line.substring(p1 + 1, p2).toFloat();
    b = line.substring(p2 + 1).toFloat();
    return true;
}

void handleLine(String line)
{
    line.trim();
    if (line.length() == 0)
    {
        return;
    }

    if (line.startsWith("WHEEL_VEL,") || line.startsWith("WV,"))
    {
        float leftMps = 0.0f;
        float rightMps = 0.0f;

        if (parseTwoFloats(line, leftMps, rightMps))
        {
            setTargetWheelMps(leftMps, rightMps);
        }

        return;
    }

    if (line.startsWith("CMD_VEL,") || line.startsWith("CV,"))
    {
        float linearX = 0.0f;
        float angularZ = 0.0f;

        if (parseTwoFloats(line, linearX, angularZ))
        {
            setTargetCmdVel(linearX, angularZ);
        }

        return;
    }

    if (line == "STOP")
    {
        stopRobot();
        return;
    }

    if (line == "RESET_ENC")
    {
        encL.clearCount();
        encR.clearCount();
        prevTickL = 0;
        prevTickR = 0;
        cpsL = 0.0f;
        cpsR = 0.0f;
        resetPid();
        return;
    }
}

void handleSerial()
{
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();

        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            handleLine(serialBuf);
            serialBuf = "";
        }
        else if (serialBuf.length() < 160)
        {
            serialBuf += c;
        }
        else
        {
            serialBuf = "";
        }
    }
}

// ============================================================
// Telemetry
// ============================================================
void sendEnc(long tickL, long tickR)
{
    Serial.print("ENC,");
    Serial.print(tickL);
    Serial.print(",");
    Serial.println(tickR);
}

void sendDebug(int pwmL, int pwmR)
{
    Serial.print("DBG,");
    Serial.print(targetMps.left, 4);
    Serial.print(",");
    Serial.print(targetMps.right, 4);
    Serial.print(",");
    Serial.print(targetCps.left, 2);
    Serial.print(",");
    Serial.print(targetCps.right, 2);
    Serial.print(",");
    Serial.print(cpsL, 2);
    Serial.print(",");
    Serial.print(cpsR, 2);
    Serial.print(",");
    Serial.print(pwmL);
    Serial.print(",");
    Serial.print(pwmR);
    Serial.print(",");
    Serial.println(syncPwm);
}

// ============================================================
// Arduino setup/loop
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    setupMotor(motorL);
    setupMotor(motorR);

    ESP32Encoder::useInternalWeakPullResistors = UP;
    encL.attachHalfQuad(encLA, encLB);
    encR.attachHalfQuad(encRA, encRB);

    delay(500);

    encL.clearCount();
    encR.clearCount();

    prevTickL = encSignL * encL.getCount();
    prevTickR = encSignR * encR.getCount();

    prevMs = millis();
    lastCmdMs = prevMs;
    lastDebugMs = prevMs;

    stopRobot();

    Serial.println("ESP32 wheel controller ready");
    Serial.println("RX: WHEEL_VEL,left_mps,right_mps | WV,left_mps,right_mps");
    Serial.println("RX: CMD_VEL,linear_x,angular_z | CV,linear_x,angular_z");
    Serial.println("TX: ENC,left_ticks,right_ticks");
}

void loop()
{
    handleSerial();

    uint32_t now = millis();
    if (now - prevMs < controlMs)
    {
        return;
    }

    long tickL = encSignL * encL.getCount();
    long tickR = encSignR * encR.getCount();

    float dt = (now - prevMs) / 1000.0f;
    if (dt <= 0.0f)
    {
        return;
    }

    float rawCpsL = (tickL - prevTickL) / dt;
    float rawCpsR = (tickR - prevTickR) / dt;

    cpsL = lpf(cpsL, rawCpsL, cpsAlpha);
    cpsR = lpf(cpsR, rawCpsR, cpsAlpha);

    if ((now - lastCmdMs) > cmdTimeoutMs)
    {
        cmdMps = {0.0f, 0.0f};
        cmdCps = {0.0f, 0.0f};
    }

    bool hadTargetBeforeRamp = hasTarget();
    updateTargetRamp(dt);

    if (hadTargetBeforeRamp && !hasTarget() && !hasCommand())
    {
        startStopHold(now);
    }

    int pwmL = 0;
    int pwmR = 0;

    if (hasTarget())
    {
        bool spin = isSpinTarget();
        int minL = spin ? minSpinPwmL : minPwmL;
        int minR = spin ? minSpinPwmR : minPwmR;

        pwmL = feedPwm(targetCps.left, minL, baseCpsL) +
               roundInt(pidL.compute(targetCps.left, cpsL, dt));

        pwmR = feedPwm(targetCps.right, minR, baseCpsR) +
               roundInt(pidR.compute(targetCps.right, cpsR, dt));

        int sync = calcSyncPwm(tickL, tickR, dt);

        pwmL += signForTarget(targetCps.left) * sync;
        pwmR -= signForTarget(targetCps.right) * sync;

        pwmL = constrain(pwmL, -255, 255);
        pwmR = constrain(pwmR, -255, 255);

        driveBoth(pwmL, pwmR);
    }
    else
    {
        if (shouldHoldStop(now))
        {
            brakeBoth(stopBrakePwmL, stopBrakePwmR);
        }
        else
        {
            driveBoth(0, 0);
        }
    }

    sendEnc(tickL, tickR);

    if (enableDebug && (now - lastDebugMs >= debugMs))
    {
        sendDebug(pwmL, pwmR);
        lastDebugMs = now;
    }

    prevTickL = tickL;
    prevTickR = tickR;
    prevMs = now;
}
