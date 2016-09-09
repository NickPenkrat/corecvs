#include "gcodeLoader.h"

#include <sstream>
#include "utils.h"

namespace corecvs {

using std::istringstream;
using std::string;
using std::vector;
using std::pair;

GcodeLoader::GcodeLoader()
{}

vector<pair<char, double>> parseLine(string gline)
{
    vector<pair<char, double>> result;
    vector<string> split = HelperUtils::stringSplit(gline, ' ');
    for (string s: split)
    {
        pair<char, double> p;
        if (!s.empty() && std::isalpha(s[0])) {
            p.first = s[0];
        }
        s = s.substr(1);

        /*G CODE is locale independant so we need to parse double in C locale*/
        if (!s.empty()) {
            std::locale mylocale("C");
            istringstream ss(s);
            ss.imbue(mylocale);
            ss >> p.second;
            result.push_back(p);
        }
    }

    return result;
}



int GcodeLoader::loadGcode(istream &input, Mesh3D &mesh)
{
    string line;

    Vector3dd currentPosition = Vector3dd::Zero();
    mesh.switchColor(true);

    while (!input.eof())
    {
        HelperUtils::getlineSafe (input, line);

        if (line.empty())
            continue;

        if (trace) {
            cout << "Parsing line <" << line << ">" << endl;
        }

        std::transform(line.begin(), line.end(), line.begin(), ::tolower);

        /* Removing comments */
        line = HelperUtils::stringSplit(line, ';').front();
        if (line.empty())
            continue;
        line = HelperUtils::stringSplit(line, '(').front();
        if (line.empty())
            continue;

        if (trace) {
            cout << "Without comments <" << line << ">" << endl;
        }
        vector<pair<char, double>> tokens = parseLine(line);
        for (auto token : tokens)
        {
            cout << " " << token.first << " -> " << token.second << endl;
        }
        if (tokens.empty())
            continue;

        const auto &token = tokens[0];

        switch(token.first)
        {
            case 'g':
            {
                Vector3dd target = currentPosition;
                for (int i = 1; i < (int)tokens.size(); i++)
                {
                    switch (tokens[i].first) {
                        case 'x': target.x() = tokens[i].second; break;
                        case 'y': target.y() = tokens[i].second; break;
                        case 'z': target.z() = tokens[i].second; break;
                        default: break;
                    }
                }

                if (token.second == 0) {
                    cout << "G0 move" << endl;
                    mesh.setColor(RGBColor::Gray());
                    mesh.addLine(currentPosition, target);
                } else if (token.second == 1) {
                    cout << "G1 move" << endl;
                    mesh.setColor(RGBColor::Blue());
                    mesh.addLine(currentPosition, target);
                } else if (token.second == 2 || token.second == 3) {
                    Vector3dd center = currentPosition;
                    mesh.setColor(RGBColor::Yellow());

                    for (size_t i = 1; i < tokens.size(); i++)
                    {
                        switch (tokens[i].first) {
                            case 'i': center.x() += tokens[i].second; break;
                            case 'j': center.y() += tokens[i].second; break;
                            case 'k': center.z() += tokens[i].second; break;
                            default: break;
                        }
                    }

                    Vector3dd normal = (currentPosition - center) ^ (target - center);
                    normal.normalise();
                    Vector3dd axis1 = (currentPosition - center);
                    double r = (currentPosition - center).l2Metric();
                    axis1.normalise();;
                    Vector3dd axis2 = (axis1 ^ normal).normalised();
                    if (token.second == 3)
                        axis2 = -axis2;

                    Vector2dd proj;
                    proj.x() = (target-center) & axis1;
                    proj.y() = (target-center) & axis2;
                    double maxArg = proj.argument();

                    double arg = 0;
                    for (int i =0; i <= arcSteps; i++)
                    {
                        arg += maxArg / arcSteps;
                        Vector3dd subTarget = center + axis1 * r * cos(arg) + axis2 * r * sin(arg);
                        mesh.addLine(currentPosition, subTarget);
                        currentPosition = subTarget;
                    }

                    mesh.setColor(RGBColor::Pink());
                    mesh.addLine(currentPosition, target);

                } else {
                    mesh.setColor(RGBColor::Green());
                    mesh.addLine(currentPosition, target);
                }

                currentPosition = target;
                break;
            }
            case 'm': break;
        }

    }
    return 0;
}

GcodeLoader::~GcodeLoader()
{}

} // namespace corecvs

