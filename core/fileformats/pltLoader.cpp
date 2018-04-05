#include "pltLoader.h"
#include "core/utils/utils.h"

namespace corecvs {

using namespace std;

#if 0

HPGLProgram::Record parseLine(const string& gline)
{
    vector<pair<char, double>> result;
    vector<string> split = HelperUtils::stringSplit(gline, ' ');

    // cout << "parseLine("<< gline << "):called" << endl;
    for (string s: split)
    {
        // cout << "parseLine(): processing <" << s << ">" << endl;
        pair<char, double> p;

        if (s.empty())
            continue;

        if (isalpha(s[0])) {
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

HPGLLoader::HPGLLoader()
{

}


int HPGLLoader::loadHPGLcode(std::istream &input, HPGLProgram &program)
{
    string line;

    program.program.clear();

    while (!input.eof())
    {
        HelperUtils::getlineSafe (input, line);

        if (line.empty())
            continue;

        if (trace) {
            cout << "Parsing line <" << line << ">" << endl;
        }

        GCodeProgram::Code code;
        /* Processing comments */
        line = HelperUtils::removeLeading(line, " \t");

        std::vector<string> parts;
        parts = HelperUtils::stringSplit(line, ';');
        line = parts.front();
        parts.erase(parts.begin() + 0, parts.begin() + 1); // Stupid C++;
        if (parts.size() > 0)
        {
            code.comment += HelperUtils::stringCombine(parts, ';');
        }

        if (line.empty())
        {
            program.program.push_back(code);
            continue;
        }

        parts = HelperUtils::stringSplit(line, '(');
        line = parts.front();
        parts.erase(parts.begin() + 0, parts.begin() + 1); // Stupid C++;
        if (parts.size() > 0)
        {
            code.comment += HelperUtils::stringCombine(parts, '(');
        }

        if (line.empty())
        {
            program.program.push_back(code);
            continue;
        }

        std::transform(line.begin(), line.end(), line.begin(), ::tolower);

        if (trace) {
            cout << "Without comments <" << line << ">" << endl;
        }

        vector<pair<char, double>> tokens = parseLine(line);
        /*for (auto token : tokens)
        {
            cout << " " << token.first << " -> " << token.second << endl;
        }*/
        if (tokens.empty())
            continue;

        const auto &token = tokens[0];
        code.area = token.first;
        code.number = token.second;

        for (int i = 1; i < (int)tokens.size(); i++)
        {
            GCodeProgram::Record r = {tokens[i].first, tokens[i].second};
            code.parameters.push_back(r);
        }

        program.program.push_back(code);
    }
    return 0;
}

#endif

} // namespace corecvs
