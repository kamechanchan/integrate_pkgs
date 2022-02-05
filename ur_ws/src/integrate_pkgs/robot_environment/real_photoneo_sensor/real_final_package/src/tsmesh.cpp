#include <iostream>
#include <iomanip>
#include <string>

using namespace std;

string getDatetimeStr() {
    time_t t = time(nullptr);
    const tm* localTime = localtime(&t);
    std::stringstream s;
    s << "20" << localTime->tm_year - 100 << " ";
    // setw(),setfill()で0詰め
    s << setw(2) << setfill('0') << localTime->tm_mon + 1 << " ";
    s << setw(2) << setfill('0') << localTime->tm_mday << " ";
    s << setw(2) << setfill('0') << localTime->tm_hour << " ";
    s << setw(2) << setfill('0') << localTime->tm_min << " ";
    s << setw(2) << setfill('0') << localTime->tm_sec;
    // std::stringにして値を返す
    return s.str();
}

void get_extension()
{
    std::string filepath = "/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/cloud_practice/package.xml";
    int path_i = filepath.find_last_of("/") + 1;
    int ext_i = filepath.find_last_of(".");
    std::string pathname = filepath.substr(0, ext_i);
    std::string exname = filepath.substr(ext_i, filepath.size() - ext_i);
    std::cout << pathname << std::endl;
    std::cout << exname << std::endl;
    std::string hante = "/home/ericlab/fdf";
    int ext = hante.find_last_of(".");
    std::cout << ext << "   :   " << hante.size() << std::endl;
    if (ext == std::string::npos) {
        std::cout << '.' << "は見つかりませんでした。n";
    }else {
        std::cout << '.' << "は" << ext << "番目にあります。n";
    }

}

int main(int argc, char** argv)
{
    // std::cout << getDatetimeStr() <<std::endl;
    get_extension();
}