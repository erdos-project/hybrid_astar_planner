#include <fstream>
#include <iostream>
#include <map>
#include <vector>

class Logger
{
public:
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    static Logger* getInstance(){
        if(!log){
            log = new Logger();
        }
        return log;
    }

    template<typename T>
    void saveData(std::string variable, T value)
    {
        std::map<std::string,std::vector<T>>* data;

        if(std::is_same<T, int>::value)
        {
            data = &data_i;
        }
        else if (std::is_same<T, float>::value)
        {
            data = &data_f;
        }
        else if (std::is_same<T, double>::value)
        {
            data = &data_d;
        }
        else{
            std::cout << "logger.cpp: undefined datatype"<< std::endl;
        }

        typename std::map<std::string,std::vector<T>>::iterator it=data->find(variable);

        if(it != data->end())
            it->second.push_back(value);
        else       
            data->insert(std::pair<std::string, std::vector<T>>(variable,std::vector<T>{value}.at(0)));
    }

    // template<typename T>
    // std::string getData(std::string variable, int n)
    // {
    //     std::string ans="";
    //     std::map<std::string,std::vector<T>>* data;

    //     if(std::is_same<T, int>::value)
    //     {
    //         data = &data_i;
    //     }
    //     else if (std::is_same<T, float>::value)
    //     {
    //         data = &data_f;
    //     }
    //     else if (std::is_same<T, double>::value)
    //     {
    //         data = &data_d;
    //     }
    //     else{
    //         std::cout << "logger.cpp: undefined datatype"<< std::endl;
    //     }
        
    //     typename std::map<std::string,std::vector<T>>::iterator it=data->find(variable);
    //     if(it != data->end())
    //         ans = std::to_string(data[variable].at(n));
    //     return ans;
    // }
    
    //cant write data to csv, why?
    void writeDataToCSV()
   {
    fout.open("variableData.csv", std::ios::out | std::ios::app); 
    fout << "1" << "," << "2" <<"/n";
   }

private:
    Logger() = default;
    static Logger* log;
    std::fstream fout;
    std::map<std::string,std::vector<int>> data_i;
    std::map<std::string,std::vector<float>> data_f;
    std::map<std::string,std::vector<double>> data_d;
};

Logger* Logger::log = nullptr;

int main()
{
    Logger* myLogger=Logger::getInstance();
    // myLogger->writeDataToCSV();
    int i =3;
    double d=1.234;
    float f=2.34;
    myLogger->saveData<int>("varaible_1", i);
    // myLogger->saveData<double>("v_2", d);
    // myLogger->saveData<float>("anyT", f);
    myLogger->saveData<int>("variable_1", 505);

    // std::cout << myLogger->getData<int>("variable_1",0);
    // std::cout << myLogger->getData<int>("variable_1",1);
    // std::cout << myLogger->getData<int>("variable_1",2);
}