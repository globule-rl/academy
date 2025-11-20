#include <cstdint>
#include <memory>

/* no hyperthread
        multithreads in one core -> cache 
        -> slower in hotpath
    assembly: call, jump expensive -> less
*/

/*
    one if one errFlags
        less hardware branch
        less instruction cache
*/
void sendOrdToExchange() {};
void handleErr(int64_t err) {};
void func1() {
    int64_t errFlags;

    if (!errFlags) sendOrdToExchange();
    else handleErr(errFlags);
}

/*
    more template-based config
        pure virtual func -> expensive runtime lookup table per call
        rm branches
    SendOrder(): some stage in the future
        mOrderSender: context for strategy/sendOrder()
    OrderManager<T>
    factory: compile time + runtieme
        config: no subclass
            pure virtual: inheritance
*/
struct OrderSenderA {
    void sendOrder() {}
};
struct OrderSenderB {
    void sendOrder() {}
};
struct IOrderManager { virtual void mainLoop() = 0; };
template <typename T>
struct OrderManager : public IOrderManager {
    void mainLoop() final {
        mOrderSender.sendOrder();
    }
    T mOrderSender;
};

struct Config {
    bool useOrderSenderA() const {return true;}
};
Config config;
std::unique_ptr<IOrderManager> Factory(const Config& config) {
    if (config.useOrderSenderA()) return std::make_unique<OrderManager<OrderSenderA>>();
    else return std::make_unique<OrderManager<OrderSenderB>>();
}
int main(int argc, char* argv[]) {
    auto manager = Factory(config);
    manager->mainLoop();
}