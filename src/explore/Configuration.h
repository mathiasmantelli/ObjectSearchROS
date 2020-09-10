#ifndef UTILCONFIGURATIONS_H
#define UTILCONFIGURATIONS_H

#include <iostream>
#include <map>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

class Configuration
{
public:
    Configuration();
    Configuration(string adress);

    const string RemoveComment(string original);
    const string RemoveSpace(string original);

    bool Load(string adress);

    bool GetString(string key, string& dst);
    bool GetInt(string key, int &dst);
    bool GetFloat(string key, float &dst);
    bool GetDouble(string key, double &dst);
    bool GetBool(string key, bool &dst);

    string GetString(string key);
    int GetInt(string key);
    float GetFloat(string key);
    double GetDouble(string key);
    bool GetBool(string key);

private:

    map<string,string> values_;

};

#endif // UTILCONFIGURATIONS_H
