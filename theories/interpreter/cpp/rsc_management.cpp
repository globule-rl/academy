// RAII(Resource Acquisition Is Initialization)
// port wrapper
class Port
{
    PortHandle port;
public:
    Port(string_view destination) : port{open_port(destination)} {}
    ~Port() { close_port(port);}
    operator PortHandle() { return port; }
    // port handles cant be cloned, no copy assign
    Port(const Port&) = delete;
    Port& operator=(const Port&) = delete;
};
void send(unique_ptr<X> x, string_view destination)
{
    // port owns the PortHandle
    // auto port = open_port(destination); -> unlock close_port delete x
    Port port{destination};
    // guard owns the lock
    lock_guard<mutex> guard{my_mutex};
    send(port, x);
    // automatically unlocks my_mutex, closes port, deletes the pointer in x
}

// only use raw ptr to denote individual obj
// array: container vector(owning) span(non-owning)
void f(std::span<int> s)
{
    // span tracks the size
    if (s.size() > 2)
    {
        s[2] = 7;
    } else
    {}
}
// print *p using format #fmt
// use *p p[0] only single value, lifetime/ownership externally
void g(int* p, int fmt)
{
    printf("Format #%d: %d\n", fmt, *p)
}

// T* raw ptr non-owning T& raw ref non owning
void f()
{
    // dont: int* p1 = new int{7}; raw owning ptr 
    // unique_ptr guaranteers deletion of the obj
    // dont: int& r=*new int{7}; raw owning ref
    auto p2 = make_unique<int>(7);
    std::unique_ptr<int> p3= std::make_unique<int>(7);
    int& r = *p3;
}
// explicit ownership
template<typename T>
class X2
{
public:
    // p owning q not owning
    owner<T*> p;
    T* q;
};
// remember to delete ptr p
// prefer scoped obj than heap-allocate
Gadget make_gadget(int n)
{
    // unnecessary alloc and dealloc
    auto p = new Gadget{n};
    return p;
}
void caller(int n)
{
    auto p = make_gadget(n);
    // unnecessary alloc and dealloc
    delete p;
}
// return by value, not ptr if small/efficient move func
Gadget make_gadget(int n)
{
    // use local var
    Gadget g{n};
    return g;
}

// avoid non-const global var
// no support for construction/destruction: malloc() free()
class Record
{
    int id;
    string name;
};
void use()
{
    // dont: Record* p1 = static_cast<Record*>(malloc(sizeof(Record))); delete p1;
    // p1 nullptr *p1 not initialized, str is a str-sized bits cant delete obj allocated by malloc()
    // dont: auto p2 = new Record; free(p2); cant free obj allocated by new
    // instead: *p2 *p3 default initialized, *p3 could be nullptr
    auto p3 = new(nothrow) Record;
}

// explicit resource alloc to a manager obj
void func(const string& name)
{
    // dont: FILE* f = fopen(name, "r"); auto _ = finally([f] {fclose(f); });
    // remember to close the file, exception leads to leak
    // instead: ifstream open simple efficient safe
    ifstream f{name};
    vector<char> buf(1024);
}

// one explicit alloc in one expression statement
// dont: void fun(shared_ptr<Widget> sp1, shared_ptr<Widget> sp2);
// messy: shared_ptr<Widget> sp1(new Widget(a, b)); fun(sp1, new Widget(c, d));
// instead: avoid explicit alloc
// use factory wrapper func(wrap new, returns a smart ptr/owning obj)
fun(make_shared<Widget>(a, b), make_shared<Widget>(c, d));

// prefer span over []
// dont: void f(int[]); void f(int*);
void g(gsl:span<int>);

//lambda
const widget x=[&]
{
    widget val;
    for (auto i=2; i<=N; ++i)
    {
        val+=obj.sth(i);
    }
    // val should be/is const
    return val;
};

// avoid macro bugs #define C
enum E { a, b};
template<int x>
constexpr const char* stringify()
{
    switch (x)
    {
        case a: return "a";
        case b: return "b";
    }
}
void f()
{
    string s1=stringify<a>();
    string s2=stringify<b>();
}
constexpr double pi=3.14;
template<typename T> T square(T a, T b) { return a*b;}