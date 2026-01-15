#include <cstdint>
#include <iostream>

class TestFramework {
    public:
        int testsRun = 0;
        int testsPassed = 0;
        int testsFailed = 0;
        void assertEqual(uint32_t actual, uint32_t expected, const char* test) {
            testsRun++;
            if (actual == expected) {
                testsPassed++;
                printf("[PASS] %s: 0x%s == 0x%x\n", test, actual, expected);
            } else {
                testsFailed++;
                printf("[FAIL] %s: 0x%x != 0x%x\n", test, actual, expected);
            }
        }
        void assertTrue(bool condition, const char* test) {
            testsRun++;
            if (condition) {
                testsPassed++;
                printf("[PASS] %s\n", test);
            } else {
                testsFailed++;
                printf("[FAIL] %s\n", test);
            }
        }
        void printSummary() {
            printf("\n========== TEST SUMMARY =========\n");
            printf("Tests Ran: %d\n", testsRun);
            printf("Tests Passed: %d\n", testsPassed);
            printf("Test Failed: %d\n", testsFailed);
            if (testsFailed == 0) {
                printf("All passed \n");
            } else {
                printf("Some failed \n");
            }
            printf("====================================\n");
        }
        int getExit() {
            return testsFailed == 0 ? 0 : 1;
        }
};

TestFramework testFramework;

void testIpv4Addr() {
    uint8_t gip1=10, gip2=0, gip3=2, gip4=2;
    uint32_t gip_be = (uint32_t)gip1 | (uint32_t)gip2<<8 | (uint32_t)gip3<<16 | (uint32_t)gip4<<24; 
    uint8_t subnet1=255, subnet2=255, subnet3=255, subnet4=0;
    uint32_t subnet_be = (uint32_t)subnet1 | (uint32_t)subnet2<<8 | (uint32_t)subnet3<<16 | (uint32_t)subnet4<<24; 
    testFramework.assertEqual(gip_be, 0x0202000a, "be op: 10.0.2.2 ==> 0x0202000a 2.2.0.10");
    testFramework.assertEqual(subnet_be, 0x00fffff, "be op:  255.255.255.0 ==> 0x00fffff");
}
void testNetwork() {
    uint8_t gip1=192, gip2=168, gip3=1, gip4=100;
    uint32_t gip_be = (uint32_t)gip1 | (uint32_t)gip2<<8 | (uint32_t)gip3<<16 | (uint32_t)gip4<<24; 
    uint8_t subnet1=255, subnet2=255, subnet3=255, subnet4=0;
    uint32_t subnet_be = (uint32_t)subnet1 | (uint32_t)subnet2<<8 | (uint32_t)subnet3<<16 | (uint32_t)subnet4<<24; 
    uint32_t network = gip_be & subnet_be;
    uint32_t broadcast = network | (~subnet_be);
    testFramework.assertEqual(network, 0x01a8c0, "network mask be op: 192.168.1.0/24 1-254 ==> 0x01a8c0 0.1.168.192");
    testFramework.assertEqual(broadcast, 0xffa8c0, "network mask be op: 192.168.1.255 ==> 0xffa8c0 255.1.168.192");
}
/* sum >> 16: carry/1 from 32-bit to lower 16-bit/&0xffff
    checkSum 16 bit */
void testChecksum() {
    uint16_t data[] = {0x4500, 0x003c, 0x1c46, 0x4000, 0x4006, 0xb1e6};
    uint32_t sum = 0;
    for (int i=0; i<6; i++) { sum+=data[i]; if(sum>>16) {sum=(sum&0xffff)+(sum>>16);} }
    uint16_t checkSum = ~sum&0xffff;
    testFramework.assertTrue(checkSum == 0, "Packet Valid, sum 0xffff, Checksum == 0");
}
void runAllUnitTests() {
    testIpv4Addr();
    testNetwork();
    testChecksum();
    testFramework.printSummary();
}

int main() {
    runAllUnitTests();
    return testFramework.getExit();
}