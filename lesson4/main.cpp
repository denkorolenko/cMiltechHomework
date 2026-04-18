/******************************************************************************
Homework for Lesson 4
Denys Korolenko
*******************************************************************************/
#include <iostream>
#include <fstream>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <iomanip>

#define _USE_MATH_DEFINES

#define DEBUG_TEST_FOLDER "test5" // test1, test2, test3, test4, test5

static const int   NUM_TARGETS = 5;
static const int   TIME_STEPS  = 60;
static const int   MAX_STEPS   = 10000;
static const float G               = 9.81f;
static const float HIT_RADIUS_KOEF = 0.5f;

enum DroneState {
    STOPPED      = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING      = 3,
    MOVING       = 4
};

// боєприпаси: назва, маса, лобовий опір, підйомна сила
static const char  bombNames[][15] = {"VOG-17","M67","RKG-3","GLIDING-VOG","GLIDING-RKG"};
static const float bombM[] = {0.35f, 0.6f, 1.2f, 0.45f, 1.4f};
static const float bombD[] = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f};
static const float bombL[] = {0.0f,  0.0f,  0.0f,  1.0f,  1.0f};

static float targetXInTime[NUM_TARGETS][TIME_STEPS];
static float targetYInTime[NUM_TARGETS][TIME_STEPS];

// буфери для запису траєкторії
static float outX[MAX_STEPS+1];
static float outY[MAX_STEPS+1];
static float outDir[MAX_STEPS+1];
static int   outState[MAX_STEPS+1];
static int   outTarget[MAX_STEPS+1];
// масив для обраної точки скиду в поточний момент часу
static float outFireX[MAX_STEPS+1];
static float outFireY[MAX_STEPS+1];
// масив для екстрапольованої позиції цілі у майбутнє, куди цілиться дрон
static float outPredTargetX[MAX_STEPS+1];
static float outPredTargetY[MAX_STEPS+1];

struct DroneConfig {
    float x, y, z;
    float initialDir;
    float attackSpeed;
    float accelerationPath;
    float m, d, l;          // властивості боєприпасу
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

struct SimState {
    float cx, cy;
    float dir, vel;
    DroneState state;
    float currentTime;
    float turnTarget;
    float turnRemain;
    int   chosenTarget;
};

// ============================================================
// Утилітарні фізичні функції
// ============================================================

int findAmmoIndex(const char* name) {
    for (int i = 0; i < 5; i++) {
        if (std::strcmp(bombNames[i], name) == 0) return i;
    }
    return -1;
}

// розв'язок кубічного рівняння методом Кардано: a*t^3 + b*t^2 + c = 0
float solveCubicTime(float a, float b, float c) {
    const float EPS = 1e-6f;

    if (fabs(a) < EPS)
        throw std::runtime_error("Coefficient 'a' is too close to zero");

    float p = -b * b / (3.0f * a * a);
    float q = (2.0f * b * b * b) / (27.0f * a * a * a) + c / a;
    float arg = 3.0f * q / (2.0f * p) * sqrtf(-3.0f / p);

    if (arg < -1.0f - EPS || arg > 1.0f + EPS)
        throw std::runtime_error("acos argument out of range: " + std::to_string(arg));

    arg = std::max(-1.0f, std::min(1.0f, arg)); // на випадок флотингових похибок
    float phi = acosf(arg);
    float t = 2.0f * sqrtf(-p / 3.0f) * cosf((phi + 4.0f * M_PI) / 3.0f) - b / (3.0f * a);
    return t;
}

float computeFlightTime(float z0, float V0, float m, float d, float l) {
    float a = d * G * m - 2.0f * d * d * l * V0;
    float b = -3.0f * G * m * m + 3.0f * d * l * m * V0;
    float c = 6.0f * m * m * z0;
    return solveCubicTime(a, b, c);
}

// горизонтальна дальність польоту (розклад у ряд до t^5)
float computeHorizontalDistance(float t, float V0, float m, float d, float l) {
    float term1 = V0 * t;

    float term2 = -(powf(t, 2) * d * V0) / (2.0f * m);

    float term3 =
        powf(t, 3) * (6.0f * d * G * l * m - 6.0f * d * d * (powf(l, 2) - 1) * V0) /
        (36.0f * m * m);

    float term4 =
        (powf(t, 4) * (
            -6.0f * d * d * G * l * (1.0f + powf(l, 2) + powf(l, 4)) * m +
             3.0f * powf(d, 3) * powf(l, 2) * (1.0f + powf(l, 2)) * V0 +
             6.0f * powf(d, 3) * powf(l, 4) * (1.0f + powf(l, 2)) * V0
        )) /
        (36.0f * powf(1.0f + powf(l, 2), 2) * powf(m, 3));

    float term5 =
        (powf(t, 5) * (
             3.0f * powf(d, 3) * G * powf(l, 3) * m -
             3.0f * powf(d, 4) * powf(l, 2) * (1.0f + powf(l, 2)) * V0
        )) /
        (36.0f * (1.0f + powf(l, 2)) * powf(m, 4));

    return term1 + term2 + term3 + term4 + term5;
}

void interpolateTarget(int targetIdx, float t, float arrayTimeStep,
                       float& outTx, float& outTy) {
    int   idx  = static_cast<int>(std::floor(t / arrayTimeStep)) % TIME_STEPS;
    int   next = (idx + 1) % TIME_STEPS;
    float frac = (t - idx * arrayTimeStep) / arrayTimeStep;
    outTx = targetXInTime[targetIdx][idx] + (targetXInTime[targetIdx][next] - targetXInTime[targetIdx][idx]) * frac;
    outTy = targetYInTime[targetIdx][idx] + (targetYInTime[targetIdx][next] - targetYInTime[targetIdx][idx]) * frac;
}

// обчислює точку скиду та приблизний час польоту до неї
// повертає false якщо балістика не збіглась
bool computeFirePoint(float droneX, float droneY, float droneZ,
                      float tgtX, float tgtY,
                      float V0, float accPath,
                      float m, float d, float l,
                      float& fireX, float& fireY,
                      float& flightDist, float& flightTime) {
    try {
        const float EPS = 1e-6f;
        flightTime = computeFlightTime(droneZ, V0, m, d, l);
        flightDist = computeHorizontalDistance(flightTime, V0, m, d, l);
        if (fabs(flightTime) <= EPS || fabs(flightDist) <= EPS)
            return false;

        float dx = tgtX - droneX;
        float dy = tgtY - droneY;
        float D  = std::sqrt(dx*dx + dy*dy);

        float curX = droneX, curY = droneY;

        if (flightDist + accPath > D) {
            // треба відлетіти далі щоб набрати відстань для скиду
            if (fabs(D) < EPS) {
                curX = droneX + flightDist + accPath;
                curY = droneY;
            } else {
                float r = (flightDist + accPath) / D;
                curX = tgtX - (tgtX - droneX) * r;
                curY = tgtY - (tgtY - droneY) * r;
            }
            dx = tgtX - curX;
            dy = tgtY - curY;
            D  = std::sqrt(dx*dx + dy*dy);
        }

        if (fabs(D) < EPS) {
            fireX = curX;
            fireY = curY;
            return true;
        }
        float ratio = (D - flightDist) / D;
        fireX = curX + (tgtX - curX) * ratio;
        fireY = curY + (tgtY - curY) * ratio;
    } catch (...) {
        return false;
    }
    return true;
}

// різниця кутів у діапазоні [-pi, pi]
float angleDiff(float from, float to) {
    float diff = to - from;
    diff = fmodf(diff + M_PI, 2.0f * M_PI);
    if (diff < 0.0f) diff += 2.0f * M_PI;
    return diff - M_PI;
}

// ============================================================
// Введення / виведення
// ============================================================

bool readInput(DroneConfig& cfg) {
    #ifdef DEBUG_TEST_FOLDER
    std::ifstream in( DEBUG_TEST_FOLDER "/input.txt");
    if (!in.is_open()) {
        std::cerr << "Error: cannot open " << DEBUG_TEST_FOLDER << "/input.txt\n";
        return false;
    }
    #else
    std::ifstream in("input.txt");
    if (!in.is_open()) {
        std::cerr << "Error: cannot open input.txt\n";
        return false;
    }
    #endif

    char ammo_name[64];
    in >> cfg.x >> cfg.y >> cfg.z
       >> cfg.initialDir
       >> cfg.attackSpeed >> cfg.accelerationPath
       >> ammo_name
       >> cfg.arrayTimeStep >> cfg.simTimeStep
       >> cfg.hitRadius >> cfg.angularSpeed >> cfg.turnThreshold;

    if (in.fail()) {
        std::cerr << "Error: invalid input.txt format\n";
        return false;
    }
    in.close();

    int ammoIdx = findAmmoIndex(ammo_name);
    if (ammoIdx < 0) {
        std::cerr << "Error: unknown ammo type: " << ammo_name << "\n";
        return false;
    }
    cfg.m = bombM[ammoIdx];
    cfg.d = bombD[ammoIdx];
    cfg.l = bombL[ammoIdx];
    return true;
}

bool loadTargets() {
    #ifdef DEBUG_TEST_FOLDER
    std::ifstream tf( DEBUG_TEST_FOLDER "/targets.txt");
    if (!tf.is_open()) {
        std::cerr << "Error: cannot open " << DEBUG_TEST_FOLDER << "/targets.txt\n";
        return false;
    }
    #else
    std::ifstream tf("targets.txt");
    if (!tf.is_open()) {
        std::cerr << "Error: cannot open targets.txt\n";
        return false;
    }
    #endif

    for (int i = 0; i < NUM_TARGETS; i++)
        for (int j = 0; j < TIME_STEPS; j++)
            tf >> targetXInTime[i][j];

    for (int i = 0; i < NUM_TARGETS; i++)
        for (int j = 0; j < TIME_STEPS; j++)
            tf >> targetYInTime[i][j];

    if (tf.fail()) {
        std::cerr << "Error: invalid targets.txt format\n";
        return false;
    }
    tf.close();
    return true;
}

bool writeOutput(int stepCount) {
    #ifdef DEBUG_TEST_FOLDER
    std::ofstream out( DEBUG_TEST_FOLDER "/simulation.txt");
    if (!out.is_open()) {
        std::cerr << "Error: cannot open " << DEBUG_TEST_FOLDER << "/simulation.txt\n";
        return false;
    }
    #else
    std::ofstream out("simulation.txt");
    if (!out.is_open()) {
        std::cerr << "Error: cannot open simulation.txt\n";
        return false;
    }
    #endif

    out << std::fixed << std::setprecision(3);
    out << stepCount - 1 << "\n";

    for (int i = 0; i < stepCount; ++i)
        out << outX[i] << " " << outY[i] << " ";
    out << "\n";

    for (int i = 0; i < stepCount; ++i)
        out << outDir[i] << " ";
    out << "\n";

    for (int i = 0; i < stepCount; ++i)
        out << outState[i] << " ";
    out << "\n";

    for (int i = 0; i < stepCount; ++i)
        out << outTarget[i] << " ";
    out << "\n";

    out.close();

    #ifdef DEBUG_TEST_FOLDER
    // debug.txt для зручного перегляду результатів
    // N
    // масив для обраної точки скиду в поточний момент часу
    // масив для екстрапольованої позиції цілі у майбутнє, куди цілиться дрон
    std::ofstream dbg(DEBUG_TEST_FOLDER "/debug.txt");

    dbg << std::fixed << std::setprecision(3);
    dbg << stepCount - 1 << "\n";
    
    for (int i = 0; i < stepCount; ++i)
        dbg << outFireX[i] << " " << outFireY[i] << " ";
    dbg << "\n";

    for (int i = 0; i < stepCount; ++i)
        dbg << outPredTargetX[i] << " " << outPredTargetY[i] << " ";
    dbg << "\n";

    dbg.close();
    #endif


    return true;
}

// ============================================================
// Логіка симуляції
// ============================================================

// вибір найкращої цілі з урахуванням lead targeting та штрафу за зміну
// повертає індекс цілі або -1 якщо жодна не досяжна
int selectBestTarget(const SimState& s, const DroneConfig& cfg,
                     float& bestFireX, float& bestFireY, float& bestPredX, float& bestPredY) {
    const float acceleration = cfg.attackSpeed * cfg.attackSpeed / (2.0f * cfg.accelerationPath);

    float tgtX[NUM_TARGETS], tgtY[NUM_TARGETS];
    for (int i = 0; i < NUM_TARGETS; i++)
        interpolateTarget(i, s.currentTime, cfg.arrayTimeStep, tgtX[i], tgtY[i]);

    float bestTotalTime = 1e18f;
    int   bestTarget    = -1;
    bestFireX = 0.0f;
    bestFireY = 0.0f;
    bestPredX = 0.0f;
    bestPredY = 0.0f;


    for (int i = 0; i < NUM_TARGETS; i++) {
        float tgtXNext, tgtYNext;
        interpolateTarget(i, s.currentTime + cfg.simTimeStep, cfg.arrayTimeStep, tgtXNext, tgtYNext);
        float tvx = (tgtXNext - tgtX[i]) / cfg.simTimeStep;
        float tvy = (tgtYNext - tgtY[i]) / cfg.simTimeStep;

        // перша оцінка часу — до поточної позиції цілі
        float fx, fy, hDist, tFlight;
        if (!computeFirePoint(s.cx, s.cy, cfg.z, tgtX[i], tgtY[i],
                              cfg.attackSpeed, cfg.accelerationPath, cfg.m, cfg.d, cfg.l,
                              fx, fy, hDist, tFlight))
            continue;

        float dxFire = fx - s.cx, dyFire = fy - s.cy;
        float distFire = std::sqrt(dxFire*dxFire + dyFire*dyFire);
        float droneFlightTime = distFire / cfg.attackSpeed + cfg.accelerationPath / cfg.attackSpeed;
        float totalTime = droneFlightTime + tFlight;

        // прогнозована позиція і повторний розрахунок
        float predX = tgtX[i] + tvx * totalTime;
        float predY = tgtY[i] + tvy * totalTime;

        float fx2, fy2, hDist2, tFlight2;
        if (!computeFirePoint(s.cx, s.cy, cfg.z, predX, predY,
                              cfg.attackSpeed, cfg.accelerationPath, cfg.m, cfg.d, cfg.l,
                              fx2, fy2, hDist2, tFlight2))
            continue;

        dxFire = fx2 - s.cx;
        dyFire = fy2 - s.cy;
        distFire = std::sqrt(dxFire*dxFire + dyFire*dyFire);
        droneFlightTime = distFire / cfg.attackSpeed + cfg.accelerationPath / cfg.attackSpeed;
        totalTime = droneFlightTime + tFlight2;

        // час розвороту у точці скиду: напрямок підльоту vs напрямок до цілі
        {
            float dirApproach  = std::atan2(dyFire, dxFire);
            float dirFireToTgt = std::atan2(predY - fy2, predX - fx2);
            float turnAtFire   = std::fabs(angleDiff(dirApproach, dirFireToTgt)) / cfg.angularSpeed;
            totalTime += turnAtFire;
        }

        // штраф за зміну цілі
        if (i != s.chosenTarget) {
            float timeToStop = 0.0f;
            switch (s.state) {
                case STOPPED:
                case TURNING:      timeToStop = s.turnRemain / cfg.angularSpeed; break;
                case ACCELERATING: timeToStop = s.vel / acceleration; break;
                case MOVING:       timeToStop = cfg.attackSpeed / acceleration; break;
                case DECELERATING: timeToStop = s.vel / acceleration; break;
            }
            totalTime += timeToStop;
        }

        if (totalTime < bestTotalTime) {
            bestTotalTime = totalTime;
            bestTarget    = i;
            bestFireX     = fx2;
            bestFireY     = fy2;
            bestPredX     = predX;
            bestPredY     = predY;
        }
    }
    return bestTarget;
}

// один крок стейт-машини дрона
void updateDroneState(SimState& s, float fireX, float fireY,
                      float predTargetX, float predTargetY,
                      const DroneConfig& cfg) {
    const float acceleration = cfg.attackSpeed * cfg.attackSpeed / (2.0f * cfg.accelerationPath);

    float toFireX = fireX - s.cx, toFireY = fireY - s.cy;
    float distToFire = std::sqrt(toFireX*toFireX + toFireY*toFireY);

    // В точці скиду: зупиняємось і повертаємось до прогнозованої цілі перед скидом
    if (distToFire <= cfg.hitRadius * HIT_RADIUS_KOEF) {
        if (s.vel > 0.0f) {
            s.vel -= acceleration * cfg.simTimeStep;
            if (s.vel > 0.0f) {
                s.cx += s.vel * cfg.simTimeStep * std::cos(s.dir);
                s.cy += s.vel * cfg.simTimeStep * std::sin(s.dir);
                s.state = DECELERATING;
                return;
            }
            s.vel   = 0.0f;
            s.state = STOPPED;
        }
        float toPredX = predTargetX - s.cx, toPredY = predTargetY - s.cy;
        float desiredDir = std::atan2(toPredY, toPredX);
        float delta = angleDiff(s.dir, desiredDir);
        if (std::fabs(delta) > cfg.turnThreshold) {
            s.state = TURNING;
            float turnStep = cfg.angularSpeed * cfg.simTimeStep;
            float sign = (delta >= 0.0f) ? 1.0f : -1.0f;
            if (std::fabs(delta) <= turnStep)
                s.dir = desiredDir;
            else
                s.dir += sign * turnStep;
        } else {
            s.dir   = desiredDir;
            s.state = STOPPED;
        }
        return;
    }

    float desiredDir = std::atan2(toFireY, toFireX);
    float delta = angleDiff(s.dir, desiredDir);

    switch (s.state) {
        case STOPPED: {
            if (std::fabs(delta) > cfg.turnThreshold) {
                s.state      = TURNING;
                s.turnTarget = desiredDir;
                s.turnRemain = std::fabs(delta);
            } else {
                s.dir   = desiredDir;
                s.state = ACCELERATING;
            }
            break;
        }
        case TURNING: {
            float turnStep = cfg.angularSpeed * cfg.simTimeStep;
            if (s.turnRemain <= turnStep) {
                s.dir        = s.turnTarget;
                s.state      = ACCELERATING;
                s.turnRemain = 0.0f;
            } else {
                float sign = (delta >= 0.0f) ? 1.0f : -1.0f;
                s.dir        += sign * turnStep;
                s.turnRemain -= turnStep;
            }
            break;
        }
        case ACCELERATING: {
            if (std::fabs(delta) > cfg.turnThreshold) {
                s.state = DECELERATING;
                break;
            }
            s.dir = desiredDir;
            s.vel += acceleration * cfg.simTimeStep;
            if (s.vel >= cfg.attackSpeed) {
                s.vel   = cfg.attackSpeed;
                s.state = MOVING;
            }
            s.cx += s.vel * cfg.simTimeStep * std::cos(s.dir);
            s.cy += s.vel * cfg.simTimeStep * std::sin(s.dir);
            break;
        }
        case MOVING: {
            if (std::fabs(delta) > cfg.turnThreshold) {
                s.state = DECELERATING;
                break;
            }
            s.dir = desiredDir;
            s.cx += s.vel * cfg.simTimeStep * std::cos(s.dir);
            s.cy += s.vel * cfg.simTimeStep * std::sin(s.dir);
            break;
        }
        case DECELERATING: {
            s.vel -= acceleration * cfg.simTimeStep;
            if (s.vel <= 0.0f) {
                s.vel        = 0.0f;
                s.state      = TURNING;
                s.turnTarget = desiredDir;
                s.turnRemain = std::fabs(angleDiff(s.dir, desiredDir));
            } else {
                s.cx += s.vel * cfg.simTimeStep * std::cos(s.dir);
                s.cy += s.vel * cfg.simTimeStep * std::sin(s.dir);
            }
            break;
        }
    }
}

// основний цикл симуляції; повертає кількість кроків або -1 при помилці
int runSimulation(const DroneConfig& cfg) {
    SimState s;
    s.cx          = cfg.x;
    s.cy          = cfg.y;
    s.dir         = cfg.initialDir;
    s.vel         = 0.0f;
    s.state       = STOPPED;
    s.currentTime = 0.0f;
    s.turnTarget  = cfg.initialDir;
    s.turnRemain  = 0.0f;
    s.chosenTarget = 0;

    float fireX = 0.0f, fireY = 0.0f;
    int   stepCount = 0;

    while (stepCount < MAX_STEPS) {
        outX[stepCount]      = s.cx;
        outY[stepCount]      = s.cy;
        outDir[stepCount]    = s.dir;
        outState[stepCount]  = s.state;
        outTarget[stepCount] = s.chosenTarget;

        float bestFireX, bestFireY;
        float bestPredX, bestPredY;
        int bestTarget = selectBestTarget(s, cfg, bestFireX, bestFireY, bestPredX, bestPredY);
        if (bestTarget < 0) {
            std::cerr << "Error: no reachable target\n";
            return -1;
        }
        s.chosenTarget = bestTarget;
        fireX = bestFireX;
        fireY = bestFireY;
        outFireX[stepCount] = fireX;
        outFireY[stepCount] = fireY;
        outPredTargetX[stepCount] = bestPredX;
        outPredTargetY[stepCount] = bestPredY;
        float toFireX = fireX - s.cx;
        float toFireY = fireY - s.cy;
        float distToFire = std::sqrt(toFireX*toFireX + toFireY*toFireY);
        ++stepCount;

        if (distToFire <= cfg.hitRadius * HIT_RADIUS_KOEF) {
            float dirToTarget = std::atan2(bestPredY - s.cy, bestPredX - s.cx);
            if (std::fabs(angleDiff(s.dir, dirToTarget)) <= cfg.turnThreshold)
                break; // в точці скиду і повернуті до цілі — скидаємо
        }

        updateDroneState(s, fireX, fireY, bestPredX, bestPredY, cfg);
        s.currentTime += cfg.simTimeStep;
    }

    return stepCount;
}

// ============================================================
// main
// ============================================================

int main() {
    DroneConfig cfg;
    if (!readInput(cfg))   return 1;
    if (!loadTargets())    return 1;

    int stepCount = runSimulation(cfg);
    if (stepCount < 0) return 1;

    if (!writeOutput(stepCount)) return 1;

    std::cout << "Done. Steps: " << stepCount - 1
              << ". Drop at: (" << outX[stepCount-1] << ", " << outY[stepCount-1] << ")\n";
    return 0;
}
