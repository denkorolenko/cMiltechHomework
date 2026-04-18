// convert.cpp
// Утиліта для конвертації файлів ДЗ 2 у формат JSON для ДЗ 3:
//   input.txt   -> config.json   (конфігурація дрона та симуляції)
//   targets.txt -> targets.json (координати цілей у часі)
//
// Запуск (файли input.txt та targets.txt мають бути в тій самій папці):
//   ./convert
//
// Або з аргументами:
//   ./convert input.txt targets.txt config.json targets.json
// Якщо аргументи не вказані, використовуються файли з поточної директорії.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include "json.hpp"

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Конвертація input.txt -> config.json
// ---------------------------------------------------------------------------
//
// Формат input.txt (порядок рядків відповідає таблиці з ДЗ 2):
//   xd yd zd          // координати дрона (zd — висота)
//   initialDir        // початковий напрямок (рад)
//   attackSpeed       // швидкість атаки (м/с)
//   accelerationPath  // шлях розгону/гальмування (м)
//   ammo_name         // назва боєприпасу (VOG-17, M67, RKG-3, ...)
//   arrayTimeStep     // крок часу масиву цілей (с)
//   simTimeStep       // крок симуляції (с)
//   hitRadius         // радіус влучення (м)
//   angularSpeed      // кутова швидкість (рад/с)
//   turnThreshold     // поріг повороту (рад)
//
// Якщо у вашому input.txt інший порядок — підкоригуйте зчитування.
static bool convertInput(const std::string& inPath, const std::string& outPath) {
    std::ifstream fin(inPath);
    if (!fin) {
        std::cerr << "[ERROR] Не вдалося відкрити " << inPath << "\n";
        return false;
    }

    // Використовуємо double при читанні, щоб у JSON потрапили чисті числа
    // (наприклад, 0.1 замість 0.10000000149... як було б з float).
    double xd, yd, zd;
    double initialDir;
    double attackSpeed;
    double accelerationPath;
    std::string ammoName;
    double arrayTimeStep;
    double simTimeStep;
    double hitRadius;
    double angularSpeed;
    double turnThreshold;

    if (!(fin >> xd >> yd >> zd)) {
        std::cerr << "[ERROR] input.txt: не вдалося прочитати xd yd zd\n";
        return false;
    }
    if (!(fin >> initialDir))       { std::cerr << "[ERROR] input.txt: initialDir\n";       return false; }
    if (!(fin >> attackSpeed))      { std::cerr << "[ERROR] input.txt: attackSpeed\n";      return false; }
    if (!(fin >> accelerationPath)) { std::cerr << "[ERROR] input.txt: accelerationPath\n"; return false; }
    if (!(fin >> ammoName))         { std::cerr << "[ERROR] input.txt: ammo_name\n";        return false; }
    if (!(fin >> arrayTimeStep))    { std::cerr << "[ERROR] input.txt: arrayTimeStep\n";    return false; }
    if (!(fin >> simTimeStep))      { std::cerr << "[ERROR] input.txt: simTimeStep\n";      return false; }
    if (!(fin >> hitRadius))        { std::cerr << "[ERROR] input.txt: hitRadius\n";        return false; }
    if (!(fin >> angularSpeed))     { std::cerr << "[ERROR] input.txt: angularSpeed\n";     return false; }
    if (!(fin >> turnThreshold))    { std::cerr << "[ERROR] input.txt: turnThreshold\n";    return false; }

    // Формуємо JSON згідно з розділом 5.1 ДЗ 3 (config.json)
    json out;
    out["drone"] = {
        {"position",         {{"x", xd}, {"y", yd}}},
        {"altitude",         zd},
        {"initialDirection", initialDir},
        {"attackSpeed",      attackSpeed},
        {"accelerationPath", accelerationPath},
        {"angularSpeed",     angularSpeed},
        {"turnThreshold",    turnThreshold}
    };
    out["ammo"] = ammoName;
    out["simulation"] = {
        {"timeStep",  simTimeStep},
        {"hitRadius", hitRadius}
    };
    out["targetArrayTimeStep"] = arrayTimeStep;

    std::ofstream fout(outPath);
    if (!fout) {
        std::cerr << "[ERROR] Не вдалося відкрити для запису " << outPath << "\n";
        return false;
    }
    fout << out.dump(2) << "\n";

    std::cout << "[OK] " << inPath << " -> " << outPath << "\n";
    return true;
}

// ---------------------------------------------------------------------------
// Конвертація targets.txt -> targets.json
// ---------------------------------------------------------------------------
//
// Формат targets.txt (з ДЗ 2): 10 рядків, по 60 значень через пробіл у кожному.
//   Рядки 1..5  — X-координати цілей 1..5 для кроків часу 0..59
//   Рядки 6..10 — Y-координати цілей 1..5 для кроків часу 0..59
//
// Функція сама визначає кількість цілей та часових кроків (timeSteps)
// за фактичним вмістом файлу — без жорстко зашитих 5 та 60.
static bool convertTargets(const std::string& inPath, const std::string& outPath) {
    std::ifstream fin(inPath);
    if (!fin) {
        std::cerr << "[ERROR] Не вдалося відкрити " << inPath << "\n";
        return false;
    }

    // Зчитуємо всі непорожні рядки; кожен рядок — вектор значень (double,
    // щоб у JSON були чисті числа без float-артефактів).
    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(fin, line)) {
        std::istringstream iss(line);
        std::vector<double> vals;
        double v;
        while (iss >> v) vals.push_back(v);
        if (!vals.empty()) rows.push_back(std::move(vals));
    }

    if (rows.size() < 2 || rows.size() % 2 != 0) {
        std::cerr << "[ERROR] targets.txt: очікується парна кількість рядків "
                  << "(перша половина — X, друга — Y). Знайдено: " << rows.size() << "\n";
        return false;
    }

    const size_t targetCount = rows.size() / 2;
    const size_t timeSteps   = rows[0].size();

    // Перевіряємо узгодженість довжин рядків.
    for (size_t i = 0; i < rows.size(); ++i) {
        if (rows[i].size() != timeSteps) {
            std::cerr << "[ERROR] targets.txt: рядок " << (i + 1)
                      << " містить " << rows[i].size()
                      << " значень, очікувалось " << timeSteps << "\n";
            return false;
        }
    }

    // Будуємо JSON згідно з розділом 5.3 ДЗ 3 (targets.json)
    json out;
    out["targetCount"] = targetCount;
    out["timeSteps"]   = timeSteps;
    out["targets"]     = json::array();

    for (size_t i = 0; i < targetCount; ++i) {
        const auto& xs = rows[i];
        const auto& ys = rows[i + targetCount];

        json positions = json::array();
        for (size_t t = 0; t < timeSteps; ++t) {
            positions.push_back({{"x", xs[t]}, {"y", ys[t]}});
        }

        out["targets"].push_back({{"positions", positions}});
    }

    std::ofstream fout(outPath);
    if (!fout) {
        std::cerr << "[ERROR] Не вдалося відкрити для запису " << outPath << "\n";
        return false;
    }
    fout << out.dump(2) << "\n";

    std::cout << "[OK] " << inPath << " -> " << outPath
              << "  (targetCount=" << targetCount
              << ", timeSteps=" << timeSteps << ")\n";
    return true;
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    // За замовчуванням беремо файли з поточної директорії.
    std::string inputTxt   = "input.txt";
    std::string targetsTxt = "targets.txt";
    std::string configJson  = "config.json";
    std::string targetsJson = "targets.json";

    // Опційно: ./convert input.txt targets.txt config.json targets.json
    if (argc >= 2) inputTxt    = argv[1];
    if (argc >= 3) targetsTxt  = argv[2];
    if (argc >= 4) configJson   = argv[3];
    if (argc >= 5) targetsJson = argv[4];

    bool ok = true;
    ok &= convertInput(inputTxt, configJson);
    ok &= convertTargets(targetsTxt, targetsJson);

    return ok ? 0 : 1;
}