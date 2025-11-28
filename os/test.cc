#include <iostream>
#include <string>

/* g++ -std=c++11 test.cc
    g++ -std=c++20 test.cc -o test.out       
    ./t.out
     output: 0123456789ABCDEF
            echo | g++ -dM -E -x c++ - | grep __cplusplus
                #define __cplusplus 199711L
*/
int main() {
    std::string hex = []{
        std::string s;
        for (int i = 0; i < 16; ++i)
            s += (i < 10 ? '0' + i : 'A' + i - 10);
        return s;
    }();

    std::cout << hex << '\n';  
    return 0;
}