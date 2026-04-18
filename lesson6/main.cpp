/******************************************************************************
Homework for Lesson 6
Denys Korolenko
*******************************************************************************/
#include <iostream>
#include <fstream>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <iomanip>

// Підключення бібліотеки для роботи з JSON
#include "json.hpp"
using json = nlohmann::json;

#define _USE_MATH_DEFINES

// ============================================================
// Макроси логування
// ============================================================
#define ENABLE_LOG    1
#define ENABLE_DEBUG  0

#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

// ============================================================
// Константи
// ============================================================
static const int   MAX_STEPS       = 10000;
static const float G               = 9.81f;
static const float HIT_RADIUS_KOEF = 0.2f;

enum DroneState {
    STOPPED      = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING      = 3,
    MOVING       = 4
};

// ============================================================
// Структури
// ============================================================

struct Coord {
    float x;
    float y;

    Coord operator+(const Coord& other) const {
        return {x + other.x, y + other.y};
    }
    
    Coord operator-(const Coord& other) const {
        return {x - other.x, y - other.y};
    }
    
    Coord operator*(float s) const {
        return {x * s, y * s};
    }
    
    Coord operator/(float s) const {
        return {x / s, y / s};
    }
    
    bool operator==(const Coord& other) const {
        return (std::fabs(x - other.x) < 1e-6f && std::fabs(y - other.y) < 1e-6f);
    }
};

struct AmmoParams {
    char name[32];
    float mass;
    float drag;
    float lift;
};

struct DroneConfig {
    Coord startPos;
    float altitude;
    float initialDir;
    float attackSpeed;
    float accelPath;
    char ammoName[32];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

struct SimStep {
    Coord pos;
    float direction;
    int state;
    int targetIdx;
};

// Внутрішній стан симуляції для стейт-машини
struct SimState {
    Coord pos;
    float dir;
    float vel;
    int state;
    float currentTime;
    float turnTarget;
    float turnRemain;
    int chosenTarget;
};

// ============================================================
// Допоміжні та математичні функції
// ============================================================

float length(Coord c) {
    return std::hypot(c.x, c.y);
}

Coord normalize(Coord c) {
    float l = length(c);
    if (l < 1e-6f) return {0.0f, 0.0f};
    return c / l;
}

float solveCubicTime(float a, float b, float c) {
    const float EPS = 1e-6f;
    if (std::fabs(a) < EPS)
        throw std::runtime_error("Coefficient 'a' is too close to zero");

    float p = -b * b / (3.0f * a * a);
    float q = (2.0f * b * b * b) / (27.0f * a * a * a) + c / a;
    float arg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);

    if (arg < -1.0f - EPS || arg > 1.0f + EPS)
        throw std::runtime_error("acos argument out of range");

    arg = std::max(-1.0f, std::min(1.0f, arg));
    float phi = std::acos(arg);
    float t = 2.0f * std::sqrt(-p / 3.0f) * std::cos((phi + 4.0f * M_PI) / 3.0f) - b / (3.0f * a);
    return t;
}

float computeFlightTime(float z0, float V0, float m, float d, float l) {
    float a = d * G * m - 2.0f * d * d * l * V0;
    float b = -3.0f * G * m * m + 3.0f * d * l * m * V0;
    float c = 6.0f * m * m * z0;
    return solveCubicTime(a, b, c);
}

float computeHorizontalDistance(float t, float V0, float m, float d, float l) {
    float term1 = V0 * t;
    float term2 = -(std::pow(t, 2.0f) * d * V0) / (2.0f * m);
    float term3 = std::pow(t, 3.0f) * (6.0f * d * G * l * m - 6.0f * d * d * (std::pow(l, 2.0f) - 1.0f) * V0) / (36.0f * m * m);
    
    float term4 = (std::pow(t, 4.0f) * (
            -6.0f * d * d * G * l * (1.0f + std::pow(l, 2.0f) + std::pow(l, 4.0f)) * m +
             3.0f * std::pow(d, 3.0f) * std::pow(l, 2.0f) * (1.0f + std::pow(l, 2.0f)) * V0 +
             6.0f * std::pow(d, 3.0f) * std::pow(l, 4.0f) * (1.0f + std::pow(l, 2.0f)) * V0
        )) / (36.0f * std::pow(1.0f + std::pow(l, 2.0f), 2.0f) * std::pow(m, 3.0f));
        
    float term5 = (std::pow(t, 5.0f) * (
             3.0f * std::pow(d, 3.0f) * G * std::pow(l, 3.0f) * m -
             3.0f * std::pow(d, 4.0f) * std::pow(l, 2.0f) * (1.0f + std::pow(l, 2.0f)) * V0
        )) / (36.0f * (1.0f + std::pow(l, 2.0f)) * std::pow(m, 4.0f));

    return term1 + term2 + term3 + term4 + term5;
}

Coord interpolateTarget(Coord** targets, int timeSteps, int targetIdx, float t, float arrayTimeStep) {
    int idx = static_cast<int>(std::floor(t / arrayTimeStep)) % timeSteps;
    int next = (idx + 1) % timeSteps;
    float frac = (t - idx * arrayTimeStep) / arrayTimeStep;
    
    // Використання перевантажених операторів
    return targets[targetIdx][idx] + (targets[targetIdx][next] - targets[targetIdx][idx]) * frac;
}

bool computeFirePoint(Coord dronePos, float droneZ, Coord tgtPos,
                      float V0, float accPath, float m, float d, float l,
                      Coord& firePos, float& flightDist, float& flightTime) {
    try {
        const float EPS = 1e-6f;
        flightTime = computeFlightTime(droneZ, V0, m, d, l);
        flightDist = computeHorizontalDistance(flightTime, V0, m, d, l);
        if (std::fabs(flightTime) <= EPS || std::fabs(flightDist) <= EPS)
            return false;

        Coord delta = tgtPos - dronePos;
        float D = length(delta);
        Coord curPos = dronePos;

        if (flightDist + accPath > D) {
            if (std::fabs(D) < EPS) {
                curPos.x = dronePos.x + flightDist + accPath;
                curPos.y = dronePos.y;
            } else {
                float r = (flightDist + accPath) / D;
                curPos = tgtPos - delta * r; // Перевантаження операторів
            }
            delta = tgtPos - curPos;
            D = length(delta);
        }

        if (std::fabs(D) < EPS) {
            firePos = curPos;
            return true;
        }
        float ratio = (D - flightDist) / D;
        firePos = curPos + (tgtPos - curPos) * ratio;
    } catch (...) {
        return false;
    }
    return true;
}

float angleDiff(float from, float to) {
    float diff = to - from;
    diff = std::fmod(diff + M_PI, 2.0f * M_PI);
    if (diff < 0.0f) diff += 2.0f * M_PI;
    return diff - M_PI;
}

// ============================================================
// Логіка симуляції
// ============================================================

int selectBestTarget(const SimState& s, const DroneConfig& cfg, const AmmoParams& ammo,
                     Coord** targets, int targetCount, int timeSteps,
                     Coord& bestFirePos, Coord& bestPredPos) {
                         
    const float acceleration = cfg.attackSpeed * cfg.attackSpeed / (2.0f * cfg.accelPath);
    float bestTotalTime = 1e18f;
    int bestTarget = -1;
    
    bestFirePos = {0.0f, 0.0f};
    bestPredPos = {0.0f, 0.0f};

    for (int i = 0; i < targetCount; i++) {
        Coord tgtPos  = interpolateTarget(targets, timeSteps, i, s.currentTime, cfg.arrayTimeStep);
        Coord tgtNext = interpolateTarget(targets, timeSteps, i, s.currentTime + cfg.simTimeStep, cfg.arrayTimeStep);

        Coord tVel = (tgtNext - tgtPos) / cfg.simTimeStep;

        Coord fPos1;
        float hDist1, tFlight1;
        if (!computeFirePoint(s.pos, cfg.altitude, tgtPos,
                              cfg.attackSpeed, cfg.accelPath, ammo.mass, ammo.drag, ammo.lift,
                              fPos1, hDist1, tFlight1)) continue;

        Coord fDelta1 = fPos1 - s.pos;
        float distFire1 = length(fDelta1);
        float droneFlightTime = distFire1 / cfg.attackSpeed + cfg.accelPath / cfg.attackSpeed;
        float totalTime = droneFlightTime + tFlight1;

        Coord predPos = tgtPos + tVel * totalTime;

        Coord fPos2;
        float hDist2, tFlight2;
        if (!computeFirePoint(s.pos, cfg.altitude, predPos,
                              cfg.attackSpeed, cfg.accelPath, ammo.mass, ammo.drag, ammo.lift,
                              fPos2, hDist2, tFlight2)) continue;

        Coord fDelta2 = fPos2 - s.pos;
        float distFire2 = length(fDelta2);
        droneFlightTime = distFire2 / cfg.attackSpeed + cfg.accelPath / cfg.attackSpeed;
        totalTime = droneFlightTime + tFlight2;

        float dirApproach  = std::atan2(fDelta2.y, fDelta2.x);
        float dirFireToTgt = std::atan2(predPos.y - fPos2.y, predPos.x - fPos2.x);
        float turnAtFire   = std::fabs(angleDiff(dirApproach, dirFireToTgt)) / cfg.angularSpeed;
        totalTime += turnAtFire;

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
            bestFirePos   = fPos2;
            bestPredPos   = predPos;
        }
    }
    return bestTarget;
}

void updateDroneState(SimState& s, Coord firePos, Coord predPos, const DroneConfig& cfg) {
    const float acceleration = cfg.attackSpeed * cfg.attackSpeed / (2.0f * cfg.accelPath);

    Coord toFire = firePos - s.pos;
    float distToFire = length(toFire);

    if (distToFire <= cfg.hitRadius * HIT_RADIUS_KOEF) {
        if (s.vel > 0.0f) {
            s.vel -= acceleration * cfg.simTimeStep;
            if (s.vel > 0.0f) {
                s.pos.x += s.vel * cfg.simTimeStep * std::cos(s.dir);
                s.pos.y += s.vel * cfg.simTimeStep * std::sin(s.dir);
                s.state = DECELERATING;
                return;
            }
            s.vel = 0.0f;
            s.state = STOPPED;
        }
        Coord toPred = predPos - s.pos;
        float desiredDir = std::atan2(toPred.y, toPred.x);
        float delta = angleDiff(s.dir, desiredDir);
        if (std::fabs(delta) <= cfg.turnThreshold) {
            s.dir = desiredDir;
            s.state = STOPPED;
            return;
        }
        s.state = TURNING;
        float turnStep = cfg.angularSpeed * cfg.simTimeStep;
        float sign = (delta >= 0.0f) ? 1.0f : -1.0f;
        if (std::fabs(delta) <= turnStep)
            s.dir = desiredDir;
        else
            s.dir += sign * turnStep;
        return;
    }

    float desiredDir = std::atan2(toFire.y, toFire.x);
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
            s.pos.x += s.vel * cfg.simTimeStep * std::cos(s.dir);
            s.pos.y += s.vel * cfg.simTimeStep * std::sin(s.dir);
            break;
        }
        case MOVING: {
            if (std::fabs(delta) > cfg.turnThreshold) {
                s.state = DECELERATING;
                break;
            }
            s.dir = desiredDir;
            s.pos.x += s.vel * cfg.simTimeStep * std::cos(s.dir);
            s.pos.y += s.vel * cfg.simTimeStep * std::sin(s.dir);
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
                s.pos.x += s.vel * cfg.simTimeStep * std::cos(s.dir);
                s.pos.y += s.vel * cfg.simTimeStep * std::sin(s.dir);
            }
            break;
        }
    }
}

// ============================================================
// Функції введення / виведення / очищення
// ============================================================

DroneConfig readConfig(const char* filename) {
    std::ifstream f(filename);
    if (!f.is_open())
        throw std::runtime_error(std::string("Cannot open ") + filename);
    json j; f >> j;

    DroneConfig cfg;
    cfg.startPos.x    = j["drone"]["position"]["x"];
    cfg.startPos.y    = j["drone"]["position"]["y"];
    cfg.altitude      = j["drone"]["altitude"];
    cfg.initialDir    = j["drone"]["initialDirection"];
    cfg.attackSpeed   = j["drone"]["attackSpeed"];
    cfg.accelPath     = j["drone"]["accelerationPath"];
    cfg.angularSpeed  = j["drone"]["angularSpeed"];
    cfg.turnThreshold = j["drone"]["turnThreshold"];

    // Обходимо std::string, копіюючи безпосередньо масив символів
    std::strncpy(cfg.ammoName, j["ammo"].get<std::string>().c_str(), sizeof(cfg.ammoName)-1);
    cfg.ammoName[sizeof(cfg.ammoName)-1] = '\0';

    cfg.simTimeStep   = j["simulation"]["timeStep"];
    cfg.hitRadius     = j["simulation"]["hitRadius"];
    cfg.arrayTimeStep = j["targetArrayTimeStep"];

    LOG("Config loaded: speed=" << cfg.attackSpeed);
    return cfg;
}

AmmoParams* readAmmo(const char* filename, const char* ammoName, int& ammoCount, int& selectedIdx) {
    std::ifstream f(filename);
    if (!f.is_open())
        throw std::runtime_error(std::string("Cannot open ") + filename);
    json j; f >> j;

    ammoCount = static_cast<int>(j.size());
    AmmoParams* ammos = new AmmoParams[ammoCount];
    selectedIdx = -1;

    for (int i = 0; i < ammoCount; i++) {
        std::strncpy(ammos[i].name, j[i]["name"].get<std::string>().c_str(), sizeof(ammos[i].name)-1);
        ammos[i].name[sizeof(ammos[i].name)-1] = '\0';
        ammos[i].mass = j[i]["mass"];
        ammos[i].drag = j[i]["drag"];
        ammos[i].lift = j[i]["lift"];

        if (std::strcmp(ammos[i].name, ammoName) == 0)
            selectedIdx = i;
    }

    if (selectedIdx == -1) {
        delete[] ammos;
        throw std::runtime_error("Selected ammo not found in ammo.json");
    }

    LOG("Ammo found: " << ammos[selectedIdx].name);
    return ammos;
}

Coord** readTargets(const char* filename, int& targetCount, int& timeSteps) {
    std::ifstream f(filename);
    if (!f.is_open())
        throw std::runtime_error(std::string("Cannot open ") + filename);
    json j; f >> j;

    targetCount = j["targetCount"];
    timeSteps   = j["timeSteps"];

    Coord** targets = new Coord*[targetCount];
    for (int i = 0; i < targetCount; i++) {
        targets[i] = new Coord[timeSteps];
        for (int k = 0; k < timeSteps; k++) {
            targets[i][k].x = j["targets"][i]["positions"][k]["x"];
            targets[i][k].y = j["targets"][i]["positions"][k]["y"];
        }
    }
    return targets;
}

int runSimulation(const DroneConfig& cfg, const AmmoParams& ammo,
                  Coord** targets, int targetCount, int timeSteps,
                  SimStep* steps) {
    int stepCount = 0;

    SimState s;
    s.pos          = cfg.startPos;
    s.dir          = cfg.initialDir;
    s.vel          = 0.0f;
    s.state        = STOPPED;
    s.currentTime  = 0.0f;
    s.turnTarget   = cfg.initialDir;
    s.turnRemain   = 0.0f;
    s.chosenTarget = 0;

    while (stepCount < MAX_STEPS) {
        steps[stepCount].pos       = s.pos;
        steps[stepCount].direction = s.dir;
        steps[stepCount].state     = s.state;
        steps[stepCount].targetIdx = s.chosenTarget;

        Coord bestFirePos, bestPredPos;
        int bestTarget = selectBestTarget(s, cfg, ammo, targets, targetCount, timeSteps, bestFirePos, bestPredPos);

        if (bestTarget < 0) {
            std::cerr << "Error: no reachable target\n";
            break;
        }

        s.chosenTarget = bestTarget;
        Coord toFire   = bestFirePos - s.pos;
        float distToFire = length(toFire);

        DEBUG("Step " << stepCount << " pos=(" << s.pos.x << "," << s.pos.y << ")");
        DEBUG("  target=" << s.chosenTarget << " state=" << s.state);

        stepCount++;

        if (distToFire <= cfg.hitRadius * HIT_RADIUS_KOEF) {
            Coord toPred = bestPredPos - s.pos;
            float dirToTarget = std::atan2(toPred.y, toPred.x);
            if (std::fabs(angleDiff(s.dir, dirToTarget)) <= cfg.turnThreshold)
                break;
        }

        updateDroneState(s, bestFirePos, bestPredPos, cfg);
        s.currentTime += cfg.simTimeStep;
    }

    LOG("Simulation complete. Steps: " << stepCount);
    return stepCount;
}

void writeOutput(const char* filename, const SimStep* steps, int stepCount) {
    json j;
    j["totalSteps"] = stepCount;
    j["steps"]      = json::array();

    for (int i = 0; i < stepCount; i++) {
        json stepObj;
        stepObj["position"]    = { {"x", steps[i].pos.x}, {"y", steps[i].pos.y} };
        stepObj["direction"]   = steps[i].direction;
        stepObj["state"]       = steps[i].state;
        stepObj["targetIndex"] = steps[i].targetIdx;
        j["steps"].push_back(stepObj);
    }

    std::ofstream f(filename);
    f << j.dump(2);
}

void clean(AmmoParams*& ammos, Coord**& targets, int targetCount, SimStep*& steps) {
    for (int i = 0; i < targetCount; i++)
        delete[] targets[i];
    delete[] targets;  targets = nullptr;
    delete[] ammos;    ammos   = nullptr;
    delete[] steps;    steps   = nullptr;
}

// ============================================================
// Точка входу
// ============================================================

int main() {
    int ammoCount = 0, selectedAmmoIdx = 0;
    int targetCount = 0, timeSteps = 0;

    AmmoParams* ammos   = nullptr;
    Coord**     targets = nullptr;
    SimStep*    steps   = nullptr;

    try {
        DroneConfig cfg = readConfig("config.json");
        ammos   = readAmmo("ammo.json", cfg.ammoName, ammoCount, selectedAmmoIdx);
        targets = readTargets("targets.json", targetCount, timeSteps);

        steps = new SimStep[MAX_STEPS];
        int stepCount = runSimulation(cfg, ammos[selectedAmmoIdx], targets, targetCount, timeSteps, steps);

        writeOutput("simulation.json", steps, stepCount);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        clean(ammos, targets, targetCount, steps);
        return 1;
    }

    clean(ammos, targets, targetCount, steps);
    std::cout << "Done. Output saved to simulation.json.\n";
    return 0;
}
