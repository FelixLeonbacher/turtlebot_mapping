#include <iostream>      // Für std::cout
#include <string>        // Für std::string
#include <regex>         // Für std::regex zum Extrahieren der Zahlen
#include <cmath>

#define _USE_MATH_DEFINES



struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

bool extractOrientation(const std::string& input,
                        double& x, double& y, double& z, double& w)
{
    // Der Regex sucht nach:
    //  "orientation": { "x": ZAHL , "y": ZAHL , "z": ZAHL, "w": ZAHL }
    //
    // [-0-9\\.eE]+ erlaubt:
    //   • negative Zahlen
    //   • Dezimalpunkte
    //   • wissenschaftliche Notation (z.B. 1.23e-4)
    //
    // match[1] = x
    // match[2] = y
    // match[3] = z
    // match[4] = w
    std::regex re(
        "\"orientation\"\\s*:\\s*\\{\\s*"
        "\"x\"\\s*:\\s*([-0-9\\.eE]+)\\s*,\\s*"
        "\"y\"\\s*:\\s*([-0-9\\.eE]+)\\s*,\\s*"
        "\"z\"\\s*:\\s*([-0-9\\.eE]+)\\s*,\\s*"
        "\"w\"\\s*:\\s*([-0-9\\.eE]+)"
    );

    std::smatch match;               // Speichert die Treffergruppen

    // regex_search durchsucht den gesamten Input nach dem Muster
    if (std::regex_search(input, match, re) && match.size() == 5) {
        // match[1]..match[4] sind die gefundenen Strings
        // std::stod wandelt den String → double
        x = std::stod(match[1]);
        y = std::stod(match[2]);
        z = std::stod(match[3]);
        w = std::stod(match[4]);
        return true;                 // Extraktion erfolgreich
    }

    // Wenn kein orientation-Block gefunden wurde
    return false;
}

int main()
{
    // Beispiel-String mit Quaternion-Daten
    std::string msg = R"(---START---
                        {"header": {"seq": 76468, "stamp": {"secs": 1706294598, "nsecs": 362875926}, "frame_id": "odom"},
                        "child_frame_id": "base_footprint",
                        "pose": {"pose": {"position": {"x": -0.6689528226852417, "y": -2.5571823120117188, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.9987409114837646, "w": 0.050165168941020966}},
                        "covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                        "twist": {"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.011920928955078125}},
                        "covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}
                        ___END___)";

    double ox, oy, oz, ow;           // Variablen zum Speichern der Ergebnisse

    // Versuche Orientierung zu extrahieren
    if (extractOrientation(msg, ox, oy, oz, ow)) {
        std::cout << "Extracted Quaternion:\n";
        std::cout << "x = " << ox << "\n";
        std::cout << "y = " << oy << "\n";
        std::cout << "z = " << oz << "\n";
        std::cout << "w = " << ow << "\n";
    }
    else {
        std::cout << "Orientation block NOT found!\n";
    }


	//Beispiel Aufruf

    // =======================================================
    // === 4) Test Euler Conversion ==========================
    // =======================================================

    Quaternion q;

    q.w = ow;
    q.x = ox;
    q.y = oy;
    q.z = oz;


    EulerAngles angles = ToEulerAngles(q);

    std::cout << "Roll:  " << angles.roll  << "\n";
    std::cout << "Pitch: " << angles.pitch << "\n";
    std::cout << "Yaw:   " << angles.yaw   << "\n";








    return 0;
}
