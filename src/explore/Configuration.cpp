#include "Configuration.h"

Configuration::Configuration()
{

}

Configuration::Configuration(string adress)
{
    this->Load(adress);
}

bool Configuration::Load(string adress)
{
    ifstream file;
    file.open("/home/mathias/PioneerWorkspace/src/Phi-Exploration/config_sample_file.ini");

    if(!file.is_open()){
        cout << endl << "Cannot read configuration file: " << adress << endl;
        return false;
    }

    string line;
    while(getline(file,line))
    {
        line = RemoveComment(line);
        istringstream is_line(line);
        string key;
        if(getline(is_line, key, '='))
        {
            string value;
            if(getline(is_line, value) )
            {
                key = RemoveSpace(key);
                value = RemoveSpace(value);
                values_[key] = value;
            }

        }

    }

    return true;
}

bool Configuration::GetString(string key, string& dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = it->second;
        return true;
    }

    return false;
}

bool Configuration::GetInt(string key, int& dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = stoi(it->second);
        return true;
    }

    return false;
}

bool Configuration::GetFloat(string key, float &dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = stof(it->second);
        return true;
    }

    return false;
}

bool Configuration::GetDouble(string key, double &dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = stod(it->second);
        return true;
    }

    return false;
}

bool Configuration::GetBool(string key, bool &dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        if(it->second.compare("true") == 0 || it->second.compare("True") == 0 || it->second.compare("TRUE") == 0)
        {
            dst = true;
            return true;
        }
        else if(it->second.compare("false") == 0 || it->second.compare("False") == 0 || it->second.compare("FALSE") == 0)
        {
            dst = false;
            return true;
        }
        else{
            cout << endl << "Parameter " << key << " in configurations is not a correct bool expression. (" << it->second << ")" << endl;
            return false;
        }
    }

    return false;
}

string Configuration::GetString(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
        return it->second;

    cout << endl << "Cannot find parameter " << key << " in configurations." << endl;
    return "";
}

int Configuration::GetInt(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
        return stoi(it->second);

    cout << endl << "Cannot find parameter " << key << " in configurations." << endl;
    return -1;
}

float Configuration::GetFloat(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        istringstream buffer(it->second);
        float number;
        buffer >> number;
        return number;
    }

    cout << endl << "Cannot find parameter " << key << " in configurations." << endl;
    return -1;
}

double Configuration::GetDouble(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        istringstream buffer(it->second);
        double number;
        buffer >> number;
        return number;
    }

    cout << endl << "Cannot find parameter " << key << " in configurations." << endl;
    return -1;
}

bool Configuration::GetBool(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        if(it->second.compare("true") == 0 || it->second.compare("True") == 0 || it->second.compare("TRUE") == 0)
        {
            return true;
        }
        else if(it->second.compare("false") == 0 || it->second.compare("False") == 0 || it->second.compare("FALSE") == 0)
        {
            return false;
        }
        else{
            cout << endl << "Parameter " << key << " in configurations is not a correct bool expression.(" << it->second << ")" << endl;
            return false;
        }
    }

    cout << endl << "Cannot find parameter " << key << " in configurations." << endl;
    return false;
}

const string Configuration::RemoveSpace(string original)
{
    for (size_t i = 0; i < original.length(); i++)
    {
        if(original[i] == ' ' || original[i] == '\n' || original[i] == '\t') {
            original.erase(i, 1);
            i--;
        }
    }
    return original;
}

const string Configuration::RemoveComment(string original)
{
    string comment_marker = "#";
    size_t pos = 0;
    string token;

    while ((pos = original.find(comment_marker)) != string::npos) {
        token = original.substr(0, pos);
        original.erase(pos, string::npos);
    }
    return original;
}
