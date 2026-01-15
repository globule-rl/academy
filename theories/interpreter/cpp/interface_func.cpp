auto p=find(begin(v), end(v), val);
change_speed(23_m / 10s);
for (const auto& x : v)
for (auto& x : v)
for_each(v, [](int x))
for_each(par, v, [](int x))
draw_line(Point, Point);
static_assert(sizeof(Int) >= 4);

void read(int* p, int n); 
int a[100];

void read(span<int> r);
int a[100];
read(a);

extern void f4(vector<int>&);
extern void f4(span<int>);
void g3(int n) 
{
    vector<int> v(n);
    f4(v);
    f4(span<int>{v});
}

vector<int> f5(int n)
{
    vector<int> v(n);
    return v;
}

void increment2(span<int> p)
{
    for (int& x : p) ++x;
}
void use3(int m)
{
    const int n = 10;
    int a[n] = {};
    increment2(a);
}

void f(char* name)
{
    ifstream input {name};
    if (something) return;
}

vector<int> v;
v.reserve(100);
for (int x; cin >> x;) 
{
    v.push_back (x);
}

std::sort(begin(v), end(v), std::greater<>());

X& myX()
{
    static X my_x {3};
    return my_x;
}

void draw_rectangle(Point top_left, Point bottom_right);
void draw_rectangle(Point top_left, Size height_width);
draw_rectangle(p, Point{10, 20});
draw_rectangle(p, Size{10, 20});

alarm_settings s{};
s.enabled = true;
s.displayMode = alarm_settings::mode::spinning_light;
s.frequency = alarm_settings::every_10_seconds;
set_setting(s);

enable_lamp_options(lamp_option::on | lamp_option::animate_state_transitions);

void blink_led(milliseconds time_to_blink)
{
}
void use()
{
    blink_led(1500ms);
}

template<class rep, class period>
void blink_led(duration<rep, period> time_to_blink)
{
    auto milliseconds_to_blink = duration_cast<milliseconds>(time_to_blink);
}
void use()
{
    blink_led(2s);
    blink_led(1500ms);
}

double sqrt(double x) { Expects(x >= 0);}

int area(int height, int width)
{
    Expects(height > 0 && width > 0);
}

int area(int height, int width)
{
    auto res = height * width;
    Ensures(res > 0);
    return res;
}

void f() 
{
    char buffer [MAX];
    memset(buffer, 0, sizeof(buffer)); // memset(buffer, 0, MAX);
    Ensures(buffer[0] == 0);
}

void manipulate(Record& r)
{
    lock_guard<mutex> _ {m};
}

template<typename Iter, typename Val>
    requires input_iterator<Iter> && equality_comparable_with<iter_value_t<Iter>, Val>
Iter find(Iter first, Iter last, Val v)
{
}

template<class F, class ...Args>
explicit thread(F&& f, Args&&... args);

auto [val, error_code] = do_somthing();
if (error_code) 
{
}

vector<double> compute(args)
{
    vector<double> res(10000);
    return res;
}

owner<X*> compute(args)
{
    owner<X*> res = new X {};
    return res;
}

int length(not_null<czstring> P);
void copy(span<const T> r, span<T> r2);

void draw2(span<Circle>(arr));
Circle arr[10];
draw2(span<Circle>(arr));
draw2(arr);

template<class InputRange1, class InputRange2, class OutputIterator>
OutputIterator merge(InputRange1 r1, InputRange2 r2, OutputIterator result);

template<class In1, class In2, class Out>
    requires mergeable<In1, In2, Out>
Out merge(In1 r1, In2 r2, Out result);

void f(gsl::span<int> some_ints);
void copy_n(const T* p, T* q, int n);
void copy_n(span<const T>p, span<T> q);

struct SystemParams {
    string config_file;
    string output_path;
    seconds timeout;
}
void initialize(SystemParams p);

class Shape 
{
    public:
        virtual Point center() const = 0;
        virtual void draw() const = 0;
        virtual void rotate(int) = 0;

        virtual ~Shape() = default;
}

 class widget
 {
    class impl;
    // unique_ptr 
    // sole ownership of a dynamically allocated object
    // move-only, cant be copied
    // if out of scope, auto delete the object it manages
    std::unique_ptr<impl> pimpl;
 public:
    void draw();
    widget(int);
    ~widget();
    widget(widget&&) noexcept;
    widget(const widget&) = delete;
    widget& operator=(widget&&) noexcept;
    widget& operator=(const widget&) = delete;
 };
 class widget::impl
 {
    int n;
  public:
    void draw(const widget& w) {}
    impl(int n): n(n) {}
 };
 void widget::draw(){piml->draw(*this);}
 widget::widget(int n) : pimpl {std::make_unique<impl>(n)} {}
 widget::widget(widget&&) noexcept = default;
 widget::~widget() = default;
 widget& widget::operator={widget&&} noexcept = default;

class Istream { 
// [[gsl::suppress("lifetime)]]
public:
    enum Opt { from_line = 1};
    Istream() {}
    Istream(czstring p) : owned{true}, inp{new ifstream{p}} {}
    Istream(czstring p, Opt) : owned{true}, inp{new istringstream{p}} {}
    ~Istream() { if (owned) delete inp; }
    operator istream&() { return *inp; }
private:
    bool owned = false;
    istream* inp = &cin;
};

auto lessT = [](T x, T y) {return x.rank() < y.rank() && x.value() < y.value(); }
sort(a, b, lessT);

int read(istream& is)
{
    int x;
    is >> x;
    //check for errors
    return x;
}
void print(ostream& os, int x)
{
    os << x << "\n";
}

void read_and_print()
{
    auto x = read(cin);
    print(cout, x);
}

auto read=[](auto& input, auto& value)
{
    input >> value;
    //check errors
};
void print(auto& output, const auto& value)
{
    output << value << "\n";
}

double func1_muon(double val, int flag)
{
}
double func1_tau(double val, int flag1, int flag2)
{
}
double simple_func(double val, int flag1, int flag2)
{
    if (flag1 > 0)
        return func1_muon(val, flag2);
    if (flag1 == -1)
        return func1_tau(-val, flag1, flag2);
    return 0;
}

constexpr int min(int x, int y) {return x < y ? x : y; >}
void test(int v)
{
    int m1 = min(-1, 2); // probably compile-time
    constexpr int m2 = min(-1, 2); // compile-time
    int m3 = min(-1, v); // run-time
}

inline string cat(const string& s, const string& s2) { return s + s2; }

vector<string> collect(istream& is) noexcept
{
    vector<string> res;
    for (string s; is >> s;)
        res.push_back(s);
    return res;
}

void f(int*); // any int*
void g(unique_ptr<int>); //only transfer ownership
void g(shared_ptr<int>); // only share
void h(const unique_ptr<int>&); // receive ownership
void h(int&); // any int

void f(widge& w)
{
    use(w);
}
shared_ptr<widget> my_widget = /* ... */;
f(*my_widget);
widget stack_widget;
f(stack_widget);

template<class T>
auto square(T t) { return t * t; }

widget* find(const set<widget>& s, const widget& w, Hint); // unused parameter

template <typename Value>
Value* find(const set<Value>& s, const Value& v, [[maybe_unused]] Hint h)
{
    if constexpr (sizeof(Value) > CacheSize)
    {
        // Hint used
    }
}

struct Rec 
{
    string name;
    string addr;
    int id; // unique identifier
}
bool same(const Rec& a, const Rec& b)
{
    return a.id == b.id;
}
vector<Rec*> find_id(const string& name);
auto x = find_id(vr.begin(), vr.end(), 
    [&](Rec& r) {
        if (r.name.size() != n.size()) return false;
        for (int i=0; i<r.name.size(); ++i)
            if (tolower(r.name[i]) != tolower(n[i])) return false;
        return true;
    }
)

bool compare_insensitive(const string& a, const string& b)
{
    if (a.size() != b.size()) return false;
    for(int i=0; i<a.size(); ++i) if (tolower(a[i]) != tolower(b[i])) return false;
    return true;
}
auto x = find_if(vr.begin(), vr.end(),
    [&](Rec& r) { return compare_insensitive(r.name, n); }
);

auto cmp_to_n = [&n](const string& a) { return compare_insensitive(a, n); };
auto x = find_if(vr.begin(), vr.end(),
    [](const Rec& r) { return cmp_to_n(r.name); }
)

auto earlyUsersEnd = std::remove_if(users.begin(), users.end(),
                        [](const User &a) { return a.id > 100; });

lval: var, array ele, class, func caller/return by ref
    int a = 42;
    int* p =&a;
    *p = 20; ->lval(refer to 'a')
        struct Foo { int x; };
        Foo f;
        f.x = 42;
            int& getRef() {
                static int a = 42;
                return a;
            }
            getRef() = 5;
rval: literal 5, str, temp res, func return val, casting
    int a = 10; a->lval->prval
    int&& r = std::move(a); a->rval->xval(expiring val)

f(X x) local caller can modify, but f(x) cant change caller val, can copy move, lval+rval->small
f(X& x) reference to an lval(persistent), no copy, can modify, cant rval->in/out paras
f(const X& x) rval+lval, no copy, read-only, can rval, cant modify->no modifying
f(X&& x) rval(temporary), can modify, can move, cant copy, cant lval->move

template <typename T>
void f(T&& t);
T->X& T&&->X& lval, modifies
T->X T&&->X&& rval ref, move

void f1(const string& s); //cheap
void f2(int x); //small

int multiply(int, int);
string& concatenate(string&, const string& suffix);
// input only, transfers ownership of the widget to the sink func
// after calling sink, the caller no longer owns/uses the resource
void sink(unique_ptr<widget>);

void update(Record& r); //in-out, to be modified

void increment_all(span<int> a) //cheap to copy and passed by val
{
    from (auto&& e : a) //mutable(in-out) ref semantics
        ++e;
}

template<class T>
// new owner -> sink
void sink(std::unique_ptr<T> p) 
{
    // std::move(p) -> onward to somewhere else
    // unique_ptr empty, sink should destroy it when out of scope
    // destroy p (del the obj)
}

void foo(std::string&& s)
{
    // not use(s);
    // s->lval, needs to be casted to rval thru move()
    use(std::move(s));
    // after moved, the state is valid but unspecified cant std::cout << s;
    // need to explicitly reinitialize it or dont touch it again
}

// use TP&& to forward
// its temp obj that lives the duration of func calls
// pass to codes that dont care about the const-ness/rval-ness
template<class F, class... Args>
inline decltype(auto) invoke(F&& f, Args&&... args)
{
    return forward<F>(f)(forward<Args>(args)...);
}

// forward a composite parameter piecewise
template<class PairLike>
inline auto test(PairLike&& pairlike)
{
    // forward .first
    f1(some, args, and, forward<PairLike>.first);
    // forward .second
    f2(and, forward<PairLike>(pairlike).second, in, another, call);
}

// return output val not parameters
// dont: void find_all(const vector<int>&, vector<const int*>& out, int x); // x in-out
// instead: return out val
vector<const int*> find_all(const vector<int>&, int x);

// return result/unique_ptr if non-concrete types
std::vector<const int*> find_all(const std::vector<int>& v, int x)
{
    std::vector<const int*> result;
    for (const int& i : v)
    {
        if (i == x)
            result.push_back(&i);
    }
    return result;
}

//move operation
class Matrix 
{
    public:
        Matrix() = default;
        Matrix(const Matrix&) = default;
        Matrix(Matrix&&) noexcept = default;
        Matrix& operator = (const Matrix&) = default;
        Matrix& operator = (Matrix&&) noexcept = default;
    private:
        // vector has move op
        std::vector<double> elements;
};
Matrix operator+(const Matrix& a, const Matrix& b)
{
    Matrix res;
    // sum res
    // return value optimiztion(rvo)
    // construct val directly in it s final loc
    return res;
}
// rvo else call Matrix(Matrix&&) move constructor
// transfer ownershiop of resources(elements vector), no copying
Matrix x = m1 + m2;
// y existing obj -> complier calls Matrix& operator=(Matrix&&) move assignment
// release resource y, take temp resource
y = m3 + m3;

// small cheap-to-ret types
// dont: void val(int&); 
// Is it reading/writing from/to its arg/res
int val();

// prefer return-by-value over non-const reference output parameters
// except:
// expensive-to-move obj
struct Package
{
    char header[16];
    cahr load[2024 - 16];
};
// dont: Packge fill(); // large val
// instead: non-const ref as out, constuct Package in-place, no moves/copies
void fill(Package& p); 

// return multiple as a struct
struct min_res { int status; string data; };
min_res f(const string& inp)
{
    return { status, sth()};
}

// c++98
pair<set::iterator, bool> result = my_set.insert("Hello");
if (result.second)
    do_sth(result.first)
//c++17
if (auto [ iter, success ] = my_set.insert("hello"); success)
    do_sth(iter);

//T&
// std::operator >> ()
istream& operator>>(istream& in, string& s);

// dont pass all val as ret 
/*
struct get_str_res {istream& in; string s; }
{
    string s;
    in >> s;
    return { in move(s)};
} 
// less elegant/perfomance
for (auto [in, s] = get_str(cin); in; s=get_str(in).s)
{
}
*/

// both s/in -> in-out param
// in->manipulate state
// s->avoid repeated alloc, reuse, alloc new mem only when need to expand s
for (string s; in >> s; ) 
{
}

// ret spec, user-defined type
struct Distance {
    int val;
    int unit=1;
}
Distance d1 = measure(obj1);
auto d2 = measure(obj2);
// dont: pair tuple used not when abstracted, confusing
// auto [val, unit] = measure(obj3);
// auto [x, y] = measure(obj4);
// instead: when represent independent entities/ or optional<T>, expected<T, error_code>
std::pair<int, int> dimensions() { return {640, 480}; }

// move->avoid copying
pair<LargeObj, LargeObj> f(const string& input)
{
    LargeObj large1 = g(inp);
    LargeObj large2 = h(inp);
    return { move(large1), move(large2) };
}

// no copies, no move
pair<LargeObj, LargeObj> f(const string& inp)
{
    return { g(inp), h(inp) };
}

// T* over T& when 'no arg' is valid
// T* can be nullptr, T& cant
// zstring->char*, can be nullptr
string zstr_to_str(zstring p)
{
    if (!p) return str{};
    return str{p};
}
// vector& cant be nullptr, code is simpler
void print(const vector<int>& r)
{
    // r->vector<int>, no need to check
}

// T* owner<T*>
// dont: range error, 0-terminated array of char, *q free store
// void use(int* p, int n, char* s, int* q)
/*
{
    p[n-1]=666;
    cout << s;
    delete q;
} 
*/

void use(span<int> p, zstring s, owner<int*> q)
{
    p[p.size() - 1] = 666;
    cout << s;
    delete q;
}

// not_null<T>
int length(not_null<Record*> p);
int length(Record* p);

// span<T> span_p<T>
// find v in r, expects a span(explicit range)
X* find(span<X> r, const X& v);

// find x{} in vec
vector<X> vec;
// dont: auto p = find({vec.begin(), vec.end()}, x{});
auto p = find(std::span(vec), x{});

// span->a range of elements
// dont: c traversal/random access/extract pointer
// span knows its size->safer than raw pointer, but potentially need bound-checked
void f(span<int> s)
{
    // dont: for (gsl::index i=0; i<s.size(); ++i) cout << s[i] << '\n';
    // dont: s[7] = 9;
    for (int x : s) cout<< x << '\n';
    // dont: std::sort(&s[0], &s[s.size() / 2]);
    std::ranges::sort(s.subspan(0, s.size() / 2));
}

// zstring not_null<zstring>->null termination C-style string
// string_view for no null termination
int length(zstring p);
int length(not_null<zstring> p);

// unique_ptr<T>->transfer ownership from T to ptr dynamically
// cheapest way to pass a pointer safely, cant be copied, only moved
// no need to delete, no leaks if exception
unique_ptr<Shape> get_shape(istream& is)
{
    auto kind = read_header(is);
    switch (kind) 
    {
        case kCircle:
            // dynamically allocate Circle(constructed from istream& is) on the heap
            // and return std::unique_ptr<Shape> ptr
            return make_unique<Circle>(is);
        case kTriangle:
            return make_unique<Triangle>(is);
        default:
            throw std::runtime_error("Unknown Shape Kind");
    }
}

// shared_ptr<T>->shared, the last owner deletes the obj
// ensure the last thread to finish safely deletes the image
{
    shared_ptr<const Image> im {read_image(somewhere)};
    std::thread t0 {shade, args0, top_left, im};
    std::thread t1 {shade, args1, top_right, im};
    std::thread t2 {shade, args2, bottom_left, im};
    std::thread t3 {shade, args3, bottom_right, im};
    // detaching threads(join before main ends)
}

// return T* only to indicate a position, not tranfering ownership
// dont delete, its owned by the binary tree
Node* find(Node* t, const string& s)
{
    // ->: (*pointer).member
    if (!t || t->name == s) return t;
    if ((auto p = find(t->left, s))) return p;
    if ((auto p = find(t->right, s))) return p;
    return nullptr;
} 

// dont ret a ptr, ref to a local obj->dangling ptr
// when returns, its destroyed, yet the address is handed back to the caller
// access to the ptr afterwards is undefined
/*
int* f()
{
    int fx = 9;
    // address of local obj
    return &fx;
}
void g(int* p)
{
    // gx == *p, *p refer to the space occupied by gx, reads/writes to gx
    int gx;
    *p = 999;
}
void h()
{
    int* p = f();
    int z = *p;
    // gx reuses the abandoned stack mem by calling f(), as fx in f()
    g(p);
}
// ref ret addr refer to destroyed stack mem
int& f()
{
    int x = 7;
    return x;
}
*/
// static local var ok, except for multi-threaded
int* f()
{
    static int x = 7;
    return &x;
}
/*
// lambdas->less obvious leak
int* glob;
template<class T>
// T->anything callable with x() lambda, ptr, func
void steal(T x)
{
    // ptr stored globally, bad
    glob = x();
}
void f()
{
    int i = 99;
    // i address captured by a lambda and pass out to steal()
    // [&]->captures all local var by ref(i)
    // return the address of i
    steal([&] {return &i;});
}
int main()
{
    // once f() returns, i mem gone, glob holds a ptr to dead mem
    f();
    cout << *glob << '\n';
}
*/
void f()
{
    int i = 99;
    // capture by val, return val
    steal([=] {return i;})
}

// ret T& when no copy, no need for 'ret no obj'/theres obj to return
// no need to check for nullptr
class Car
{
    array<wheel, 4> w;
public:
    // expects->index valid
    wheel& get_wheel(int i) {Expects(i<w.sizes()); return w[i];}
}
void use()
{
    // obj c and ref w0 have the same lifetime
    // ref to the wheel, no copy
    Car c;
    wheel w0=c.get_wheel(0);
}

// dont return T&&
// &&->temp
// only std::move std::forward return &&, and ref passed and got destroyed vore temp destroyed
/*
auto&& x = max(0, 1);
foo(x);
*/
template<class F>
// dont auto&& wrapper(F f)
auto wrapper(F f)
{
    log_call(typeid(f));
    return f();
}

// int->ret for main()
int main()
{
    std::cout << "int return main\n";
}

// ret T& from assignment operators
class Foo
{
    public:
        Foo& operator=(const Foo& rhs)j
        {
            // *this->ret lval ref 
            // this-> ptr of type Foo*
            // deref: *this->Foo& ref
            // non-constant->chain a = b = c
            return *this;
        }
};

// no ret std::move(local)
S good()
{
    S result;
    // dont: return std::move(result);
    // prevents return val optimization, eliminate the move
    // instead: ret moves the local var
    return result;
}

// lambda->capture local var or be defined at local scope when func cant
// dont overload
void f(int);
void f(const string&);
vector<work> v = lots_of_work();
for (int tasknum = 0; tasknum < max; ++tasknum)
{
    pool.run([=, &v]
    {
        // process 1/max-th of v, tasknum-th chunk
    });
}
pool.join();

// default arg over overloading
// avoid code replication
void print(const string& s, format f = {});

// capture by ref
/*
// dont: large obj(network msg)
// not efficient/correct to copy
std::for_each(begin(sockets), end(sockets), [&message](auto& socket))
{
    socket.send(message);
}
*/
class stage
{
    // func->data member  where data gets stored
    std::function<void(buffer&)> func;
public:
    // std::function->callable wrapper(hold lambda, func ptr obj)
    // buffer&->params
    // : func(f)->initializing, copy f val into constructor stage()
    stage(std::function<void(buffer&)> f) : func(f) {}
    void process(buffer& b) {func(b);}
};
// bufs->ref to a buffers obj
// pipline: decorate->compress->encrypt
void send_packets(buffers& bufs)
{
    // stage->obj class
    // encryptor->var
    // b->param list for lambda, ref to buffer 
    // [&]->lambda capture
    // auto& b->ref to ele in bufs
    stage encryptor([](buffer& b){encrypt(b);});
    stage compressor([&](buffer& b) {compress(b); encryptor.process(b);});
    stage decorator([&](buffer& b){decorate(b); compressor.process(b);});
    for (auto& b : bufs) {decorator.process(b);}
}
buffer buf;
s.process(buf);

// no ref in lambdas in non-local(returned, heap, passed to another thread)
// outlive the scope of local obj
int local = 42;
// dont: thread_pool.queue_work([&] {process{local};});
// ref->after program exits, local no long exists
// instead: make a copy/unique_ptr/[*this] ptr whole copy of the obj
thread_pool.queue_work([=] {process(local);});

// no [=] in lambdas
class my_class
{
    int x = 0;
    void f()
    {
        int i = 0;
        x = 42;
        lambda();
        x = 42;
        lambda();
        // dont: auto lambda = [=] {use(i, x);};
        // looks like copy/val capture when it actually captures this ptr by val
        // instead: this [*this]->copy of all class data members
        auto lambda = [i, this] {use(i, x);};
    }
}

// va_arg arg
/*
dont:
int sum(...)
{
    while ()
        // assuming it will be int
        result += va_arg(list, int);
}
*/
// Args->match any types of arg int/double catchall
template<class ...Args>
// Args->expands into args, Args-><int, int>, args-><int arg1, int arg2>
auto sum(Args... args)
{
    // fold expression: use binary operator to reduce pack into one single val 
    // adds up all ele in args using +
    return (... + args);
}
sum(3, 2);
sum(3.14159, 2.71828);

// shallow nesting, avoid unnecessary condition nesting, return early
/*
dont:
void foo()
{
    if (x)
    {
        compute(x);
    }
}
void food()
{
    if (!x)
        return;
    else
    {
        compute(x);
    }
}
*/
void foo()
{
    if (!x)
        return;
    compute(x);
}
void foo()
{
    if (!(x&&y))
        return;
    compute(x);
}