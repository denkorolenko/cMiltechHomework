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

// ─── Константи ───────────────────────────────────────────────────────────────
static const int   NUM_TARGETS   = 5;
static const int   TIME_STEPS    = 60;
static const int   MAX_STEPS     = 10000;
static const float G            = 9.81f;

// ─── Стани дрона ─────────────────────────────────────────────────────────────
enum DroneState {
    STOPPED      = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING      = 3,
    MOVING       = 4
};

// ─── Параметри боєприпасів (масиви) ──────────────────────────────────────────
static const char  bombNames[][15] = {"VOG-17","M67","RKG-3","GLIDING-VOG","GLIDING-RKG"};
static const float bombM[]         = {0.35f, 0.6f, 1.2f, 0.45f, 1.4f};
static const float bombD[]         = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f};
static const float bombL[]         = {0.0f,  0.0f,  0.0f,  1.0f,  1.0f};

// ─── Масиви координат цілей ───────────────────────────────────────────────────
static float targetXInTime[NUM_TARGETS][TIME_STEPS];
static float targetYInTime[NUM_TARGETS][TIME_STEPS];

// ─── Буфери виводу симуляції ──────────────────────────────────────────────────
static float outX[MAX_STEPS+1];
static float outY[MAX_STEPS+1];
static float outDir[MAX_STEPS+1];
static int   outState[MAX_STEPS+1];
static int   outTarget[MAX_STEPS+1];

// ─────────────────────────────────────────────────────────────────────────────
// Пошук боєприпасу за назвою (цикл for)
// ─────────────────────────────────────────────────────────────────────────────
int findAmmoIndex(const char* name) {
    for (int i = 0; i < 5; i++) {
        if (std::strcmp(bombNames[i], name) == 0) return i;
    }
    return -1;
}

// ─────────────────────────────────────────────────────────────────────────────
// Балістика: розв'язок кубічного рівняння (метод Кардано)
// a*t^3 + b*t^2 + c = 0
// ─────────────────────────────────────────────────────────────────────────────
float solveCubicTime(float a, float b, float c) {
    const float EPS = 1e-6f;

    if (fabs(a) < EPS) {
        throw std::runtime_error("Coefficient 'a' is too close to zero");
    }

    float p = -b * b / (3.0f * a * a);
    float q = (2.0f * b * b * b) / (27.0f * a * a * a) + c / a;
    float arg = 3.0f * q / (2.0f * p) * sqrtf(-3.0f / p);

    // Large deviation = real error; tiny deviation = float rounding
    if (arg < -1.0f - EPS || arg > 1.0f + EPS) {
        throw std::runtime_error("acos argument out of range: " + std::to_string(arg));
    }
    arg = std::max(-1.0f, std::min(1.0f, arg)); // clamp float rounding noise
    float phi = acosf(arg);
    
    float t = 2.0f * sqrtf(-p / 3.0f) * cosf((phi + 4.0f * M_PI) / 3.0f) - b / (3.0f * a);
    return t;
}

// ─────────────────────────────────────────────────────────────────────────────
// Час польоту боєприпасу
// ─────────────────────────────────────────────────────────────────────────────
float computeFlightTime(float z0, float V0, float m, float d, float l) {
    float a = d * G * m - 2.0f * d * d * l * V0;
    float b = -3.0f * G * m * m + 3.0f * d * l * m * V0;
    float c = 6.0f * m * m * z0;
    return solveCubicTime(a, b, c);
}

// ─────────────────────────────────────────────────────────────────────────────
// Горизонтальна дистанція польоту (степеневий ряд до t^5)
// ─────────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
// Інтерполяція позиції цілі в момент часу t
// ─────────────────────────────────────────────────────────────────────────────
void interpolateTarget(int targetIdx, float t, float arrayTimeStep,
                       float& outTx, float& outTy) {
    int   idx  = static_cast<int>(std::floor(t / arrayTimeStep)) % TIME_STEPS;
    int   next = (idx + 1) % TIME_STEPS;
    float frac = (t - idx * arrayTimeStep) / arrayTimeStep;
    outTx = targetXInTime[targetIdx][idx] + (targetXInTime[targetIdx][next] - targetXInTime[targetIdx][idx]) * frac;
    outTy = targetYInTime[targetIdx][idx] + (targetYInTime[targetIdx][next] - targetYInTime[targetIdx][idx]) * frac;
}

// ─────────────────────────────────────────────────────────────────────────────
// Обчислити точку скиду та орієнтовний час польоту дрона до неї.
// Повертає false, якщо балістика не вдалась.
// ─────────────────────────────────────────────────────────────────────────────
bool computeFirePoint(float droneX, float droneY, float droneZ,
                      float tgtX, float tgtY,
                      float V0, float accPath,
                      float m, float d, float l,
                      float& fireX, float& fireY,
                      float& flightDist, float& flightTime) {
    try {
        const float EPS = 1e-6f;
        flightTime  = computeFlightTime(droneZ, V0, m, d, l);
        flightDist  = computeHorizontalDistance(flightTime, V0, m, d, l);
        if (fabs(flightTime) <= EPS || fabs(flightDist) <= EPS) {
            return false;
        }

        float dx = tgtX - droneX;
        float dy = tgtY - droneY;
        float D  = std::sqrt(dx*dx + dy*dy);

        float curX = droneX, curY = droneY;

        if (flightDist + accPath > D) {
            // Потрібен маневр відльоту
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

// ─────────────────────────────────────────────────────────────────────────────
// Кут між двома векторами напрямку (різниця кутів, [-π, π])
// ─────────────────────────────────────────────────────────────────────────────
float angleDiff(float from, float to) {
    float diff = to - from;
    diff = fmodf(diff + M_PI, 2.0f * M_PI);
    if (diff < 0.0f) diff += 2.0f * M_PI;
    return diff - M_PI;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    // ── Читання input.txt ────────────────────────────────────────────────────
    std::ifstream in("input.txt");
    if (!in.is_open()) {
        std::cerr << "Error: cannot open input.txt\n";
        return 1;
    }

    float droneX, droneY, droneZ;
    float initialDir;
    float attackSpeed;
    float accelerationPath;
    char  ammo_name[64];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;

    in >> droneX >> droneY >> droneZ
       >> initialDir
       >> attackSpeed >> accelerationPath
       >> ammo_name
       >> arrayTimeStep >> simTimeStep
       >> hitRadius >> angularSpeed >> turnThreshold;

    if (in.fail()) {
        std::cerr << "Error: invalid input.txt format\n";
        return 1;
    }
    in.close();

    // ── Знайти боєприпас ─────────────────────────────────────────────────────
    int ammoIdx = findAmmoIndex(ammo_name);
    if (ammoIdx < 0) {
        std::cerr << "Error: unknown ammo type: " << ammo_name << "\n";
        return 1;
    }
    float m = bombM[ammoIdx];
    float d = bombD[ammoIdx];
    float l = bombL[ammoIdx];

    // ── Читання targets.txt ──────────────────────────────────────────────────
    std::ifstream tf("targets.txt");
    if (!tf.is_open()) {
        std::cerr << "Error: cannot open targets.txt\n";
        return 1;
    }

    for (int i = 0; i < NUM_TARGETS; i++) {
        for (int j = 0; j < TIME_STEPS; j++) {
            tf >> targetXInTime[i][j];
        }
    }

    for (int i = 0; i < NUM_TARGETS; i++) {
        for (int j = 0; j < TIME_STEPS; j++) {
            tf >> targetYInTime[i][j];
        }
    }

    if (tf.fail()) {
        std::cerr << "Error: invalid targets.txt format\n";
        return 1;
    }
    tf.close();

    // ── Прискорення ──────────────────────────────────────────────────────────
    float acceleration = attackSpeed * attackSpeed / (2.0f * accelerationPath);

    // ── Стан дрона ───────────────────────────────────────────────────────────
    float    cx  = droneX, cy = droneY;   // поточна позиція
    float    dir = initialDir;            // поточний напрямок (рад)
    float    vel = 0.0f;                  // поточна швидкість
    DroneState state = STOPPED;
    float    currentTime = 0.0f;

    // Поворот
    float turnTarget  = dir;  // напрямок, до якого повертаємось
    float turnRemain  = 0.0f;  // залишок кута повороту (>0)

    // Поточна обрана ціль і точка скиду
    int   chosenTarget = 0;
    float fireX = 0.0f, fireY = 0.0f;

    int   stepCount = 0;
    bool  dropped   = false;

    // ── Головний цикл симуляції ───────────────────────────────────────────────
    while (stepCount < MAX_STEPS && !dropped) {

        // 1. Зберігаємо поточний стан
        outX[stepCount]      = cx;
        outY[stepCount]      = cy;
        outDir[stepCount]    = dir;
        outState[stepCount]  = state;
        outTarget[stepCount] = chosenTarget;

        // 2. Інтерполюємо позиції всіх цілей
        float tgtX[NUM_TARGETS], tgtY[NUM_TARGETS];
        for (int i = 0; i < NUM_TARGETS; i++)
            interpolateTarget(i, currentTime, arrayTimeStep, tgtX[i], tgtY[i]);

        // 3. Для кожної цілі: розрахунок lead targeting + точки скиду + оцінка часу
        float bestTotalTime = 1e18f;
        int    bestTarget    = -1;
        float bestFireX = 0.0f, bestFireY = 0.0f;

        for (int i = 0; i < NUM_TARGETS; i++) {
            // Швидкість цілі (кінцеві різниці)
            float tgtXNext, tgtYNext;
            interpolateTarget(i, currentTime + simTimeStep, arrayTimeStep, tgtXNext, tgtYNext);
            float tvx = (tgtXNext - tgtX[i]) / simTimeStep;
            float tvy = (tgtYNext - tgtY[i]) / simTimeStep;

            // Перша оцінка: балістика до поточної позиції
            float fx, fy, hDist, tFlight;
            if (!computeFirePoint(cx, cy, droneZ, tgtX[i], tgtY[i],
                                  attackSpeed, accelerationPath, m, d, l,
                                  fx, fy, hDist, tFlight))
                continue;

            // Оцінка часу польоту дрона до точки скиду
            float dxFire = fx - cx, dyFire = fy - cy;
            float distFire = std::sqrt(dxFire*dxFire + dyFire*dyFire);
            float droneFlightTime = distFire / attackSpeed + accelerationPath / attackSpeed;

            float totalTime = droneFlightTime + tFlight;

            // Lead targeting: перерахунок до прогнозованої позиції
            float predX = tgtX[i] + tvx * totalTime;
            float predY = tgtY[i] + tvy * totalTime;

            float fx2, fy2, hDist2, tFlight2;
            if (!computeFirePoint(cx, cy, droneZ, predX, predY,
                                  attackSpeed, accelerationPath, m, d, l,
                                  fx2, fy2, hDist2, tFlight2)) {
                continue;
            }

            dxFire = fx2 - cx; dyFire = fy2 - cy;
            distFire = std::sqrt(dxFire*dxFire + dyFire*dyFire);
            droneFlightTime = distFire / attackSpeed + accelerationPath / attackSpeed;
            totalTime = droneFlightTime + tFlight2;

            // Додаємо штраф при зміні цілі
            if (i != chosenTarget) {
                float timeToStop = 0.0f;
                switch (state) {
                    case STOPPED:
                    case TURNING:
                        timeToStop = turnRemain / angularSpeed; break;
                    case ACCELERATING:
                        timeToStop = vel / acceleration; break;
                    case MOVING:
                        timeToStop = attackSpeed / acceleration; break;
                    case DECELERATING:
                        timeToStop = vel / acceleration; break;
                }
                totalTime += timeToStop;
            }

            if (totalTime < bestTotalTime) {
                bestTotalTime = totalTime;
                bestTarget    = i;
                bestFireX     = fx2;
                bestFireY     = fy2;
            }
        }

        if (bestTarget < 0) {
            std::cerr << "Error: no reachable target\n";
            return 1;
        }

        chosenTarget = bestTarget;
        fireX = bestFireX;
        fireY = bestFireY;

        // 4. Перевірка досягнення точки скиду
        float toFireX = fireX - cx, toFireY = fireY - cy;
        float distToFire = std::sqrt(toFireX*toFireX + toFireY*toFireY);
        if (distToFire <= hitRadius) {
            dropped = true;
            stepCount++;
            break;
        }

        // 5. Визначаємо потрібний напрямок до точки скиду
        float desiredDir = std::atan2(toFireY, toFireX);
        float delta = angleDiff(dir, desiredDir);

        // 6. Оновлюємо стан і позицію дрона
        switch (state) {
            case STOPPED: {
                if (std::fabs(delta) > turnThreshold) {
                    state = TURNING;
                    turnTarget = desiredDir;
                    turnRemain = std::fabs(delta);
                } else {
                    dir = desiredDir;
                    state = ACCELERATING;
                }
                break;
            }
            case TURNING: {
                float turnStep = angularSpeed * simTimeStep;
                if (turnRemain <= turnStep) {
                    dir   = turnTarget;
                    state = ACCELERATING;
                    turnRemain = 0.0f;
                } else {
                    float sign = (delta >= 0.0f) ? 1.0f : -1.0f;
                    dir        += sign * turnStep;
                    turnRemain -= turnStep;
                }
                break;
            }
            case ACCELERATING: {
                // Перевірити, чи треба різко перенацілитись
                if (std::fabs(delta) > turnThreshold) {
                    state = DECELERATING;
                    break;
                }
                // Повільно підкоректувати напрямок якщо кут малий
                dir = desiredDir;

                vel += acceleration * simTimeStep;
                if (vel >= attackSpeed) {
                    vel = attackSpeed;
                    state = MOVING;
                }
                cx += vel * simTimeStep * std::cos(dir);
                cy += vel * simTimeStep * std::sin(dir);
                break;
            }
            case MOVING: {
                if (std::fabs(delta) > turnThreshold) {
                    state = DECELERATING;
                    break;
                }
                dir = desiredDir;
                cx += vel * simTimeStep * std::cos(dir);
                cy += vel * simTimeStep * std::sin(dir);
                break;
            }
            case DECELERATING: {
                vel -= acceleration * simTimeStep;
                if (vel <= 0.0) {
                    vel   = 0.0;
                    state = TURNING;
                    turnTarget = desiredDir;
                    turnRemain = std::fabs(angleDiff(dir, desiredDir));
                } else {
                    cx += vel * simTimeStep * std::cos(dir);
                    cy += vel * simTimeStep * std::sin(dir);
                }
                break;
            }
        }

        currentTime += simTimeStep;
        stepCount++;
    }

    // ── Запис у simulation.txt ────────────────────────────────────────────────
    std::ofstream out("simulation.txt");
    if (!out.is_open()) {
        std::cerr << "Error: cannot open simulation.txt\n";
        return 1;
    }

    out << std::fixed << std::setprecision(3);
    out << stepCount << "\n";

    // Рядок 2: координати
    for (int i = 0; i < stepCount; i++) {
        out << outX[i] << " " << outY[i] << " ";
    }
    out << "\n";

    // Рядок 3: напрямки
    for (int i = 0; i < stepCount; i++) {
        out << outDir[i] << " ";
    }
    out << "\n";

    // Рядок 4: стани
    for (int i = 0; i < stepCount; i++) {
        out << outState[i] << " ";
    }
    out << "\n";

    // Рядок 5: індекси поточної цілі
    for (int i = 0; i < stepCount; i++) {
        out << outTarget[i] << " ";
    }
    out << "\n";

    out.close();

    std::cout << "Done. Steps: " << stepCount
              << ". Drop at: (" << cx << ", " << cy << ")\n";
    return 0;
}