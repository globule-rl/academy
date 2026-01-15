#include <string>
#include <fstream>

using namespace std;


int main(int, const char**) {
    ifstream inp("1.txt");
    // ifstream inp("11.txt");
    int sum = 0;
    int dial = 50;
    string line;
    while (getline(inp, line)) {
        const int step = stoi(line.substr(1));
        for (int i=0; i<step; i++) {
            if (line[0] == 'L') dial-=1;
            else dial += 1;
            if (dial % 100 == 0) sum++;
        }
    }
    printf("%d\n", sum);
}