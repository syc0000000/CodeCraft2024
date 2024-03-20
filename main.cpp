#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <utility>
#include <vector>
#include <limits>
#include <algorithm>
#include <string>
#include <iostream>
#include <complex>
#include <thread>
#include <csignal>
#include <mutex>

#define MOVEBLOCK 1
#define STICKBlLOCK 2
#define UNBLOCK 0

#define ACTION_GET -1
#define ACTION_PULL -2
#define MOVE_FONT 2
#define MOVE_BACK 3
#define MOVE_LEFT 1
#define MOVE_RIGHT 4
#define NO_ACTION 0

const int n = 200;        // 地图大小
const int robot_num = 10; // 机器人数量
const int berth_num = 10; // 港口数量
const int N = 210;        // 地图大小++
// 坐标节点
struct Node {
public:
    int x;
    int y;
    int block;    // 是否阻塞
    bool canPull; // 是否可以放下
    bool canGet;  // 是否有货物

    // 自定义复制构造函数
    Node(const Node& other)
            : x(other.x), y(other.y), block(other.block), canPull(other.canPull), canGet(other.canGet) {
    }

    // 自定义复制赋值运算符
    Node& operator=(const Node& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            block = other.block;
            canPull = other.canPull;
            canGet = other.canGet;
        }
        return *this;
    }

    Node(int x, int y, int block, bool canPull, bool canGet)
            : x(x), y(y), block(block), canPull(canPull), canGet(canGet) {
    }

    Node() {};
};
Node chNode[N][N];
std::mutex chNodeMutex;
// 到港口路线缓存
struct PathCache {
    int distance;          // 距离
    std::queue<Node> path; // 路径
} pathCache[N][N][berth_num];

// 再极端一点，这里可以改成任意两点间距离的缓存
// Node对应的hash函数
struct NodeHash {
    std::size_t operator()(const Node &node) const {
        return std::hash<int>()(node.x) ^
               (std::hash<int>()(node.y) << 1) ^
               (std::hash<bool>()(node.block) << 2) ^
               (std::hash<bool>()(node.canPull) << 3) ^
               (std::hash<bool>()(node.canGet) << 4); // 使用位移来组合不同成员的哈希值
    }
};

// Node对应的相等函数
struct NodeEqual {
    bool operator()(const Node &lhs, const Node &rhs) const {
        return lhs.x == rhs.x &&
               lhs.y == rhs.y &&
               lhs.block == rhs.block &&
               lhs.canPull == rhs.canPull &&
               lhs.canGet == rhs.canGet;
    }
};


//货物
struct Good {
    int x;
    int y;
    int price;
    double row;//价值密度
    int berth;//绑定的港口
    bool selected = false;//被选中
    Good() {}

    Good(int x, int y, int price) {
        this->x = x;
        this->y = y;
        this->price = price;
    }

};

struct GoodHash {
    size_t operator()(const Good &good) const {
        size_t x_hash = std::hash<int>()(good.x);
        size_t y_hash = std::hash<int>()(good.y) << 1;
        size_t price_hash = std::hash<int>()(good.price) << 2;
        size_t row_hash = std::hash<double>()(good.row) << 3;
        size_t berth_hash = std::hash<int>()(good.berth) << 4;
        size_t selected_hash = std::hash<bool>()(good.selected) << 5;
        return x_hash ^ y_hash ^ price_hash ^ row_hash ^ berth_hash ^ selected_hash;
    }
};

struct GoodEqual {
    bool operator()(const Good &lhs, const Good &rhs) const {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.price == rhs.price
               && lhs.row == rhs.row && lhs.berth == rhs.berth && lhs.selected == rhs.selected;
    }
};

// struct NodeSmaller
// {
//     bool operator()(const pair<Node, int> left,const pair<Node,int> right) const {
//         return left.second<right.second;
//     }
// };
//struct ARAStar;
// 机器人类
struct Robot {
    int x /*X坐标*/, y /*Y坐标*/;
    int status;            // 状态
    std::vector<Node> path; // 路径
    bool isBusy;            //是否已占用
    int actionBefore;      // 移动前动作
    int actionMove;        // 移动动作
    int actionAfter;       // 移动后动作
    Good goodOn;

//    ARAStar* araStar;        //规划器
    Robot() {}

    Robot(int startX, int startY) {
        x = startX;
        y = startY;
    }

    void updateAction(); // 计算动作
    int goods;
} robot[robot_num + 10]; // 机器人数组

// 港口类
struct Berth {
    int x;              // 左上角X坐标
    int y;              // 左上角Y坐标
    int transport_time; // 泊位到虚拟点的时间
    int loading_speed;  // 装载速度
    Berth() {}

    Berth(int x, int y, int transport_time, int loading_speed) {
        this->x = x;
        this->y = y;
        this->transport_time = transport_time;
        this->loading_speed = loading_speed;
    }
} berth[berth_num + 10]; // 港口数组

// 船
struct Boat {
    int num;    // 货物数量
    int pos;    // 位置
    int status; // 状态
} boat[10];     // 船数组

int money;         /*当前钱*/
int boat_capacity; /*船只容量*/
int id;            /*帧序号*/
char ch[N][N];     /*完整地图*/
int gds[N][N];     /*货物位置图*/


#define INF 999

class ARAStar {
public:
    Node s_start;                                               // 起点
    Node s_goal;                                                // 终点
    double e;                                                   // 权重
    int heuristic_type;                                         // 启发式类型 两种取值：0欧几里得euclidean，1曼哈顿manhattan
    std::unordered_map<Node, int, NodeHash, NodeEqual> g;       // 到达起点的代价字典
    std::unordered_map<Node, double, NodeHash, NodeEqual> OPEN;                                // open node待探索列表
    std::unordered_set<Node, NodeHash, NodeEqual> CLOSED;       // 关闭集
    std::unordered_set<Node, NodeHash, NodeEqual> INCOMES;      // 除 OPEN 以外的 local inconsistency(变化集)
    std::unordered_map<Node, Node, NodeHash, NodeEqual> PARENT; // 节点间的父节点对应关系
    std::vector<Node> path;                                      // 规划路径
    std::unordered_set<Node, NodeHash, NodeEqual> VISITED;      // 已访问集

    ARAStar() {};

    // 构造函数
    ARAStar(Node start, Node goal, double weight, int heuristic);

//    ARAStar();
    ~ARAStar();

    // 通用搜索路径
    void search();

    // A*算法搜索修改路径
    void improvePath();

    // 计算总代价
    double f_value(Node node);

    // 计算h代价
    int h(Node node);

    // 利用父节点计算path
    void extract_path();

    // 计算真e
    double update_e();

    //计算最小的f
    std::pair<Node, double> calc_smallest_f();


};

std::queue<Node> nodeToFlush; // 下一轮开始时要刷新的节点
std::vector<Node> nodeNear(Node node);

std::unordered_set<Node, NodeHash, NodeEqual> obs; // 障碍物位置
std::unordered_set<Good, GoodHash, GoodEqual> goodsList;

#define FONT 2
#define BACK 3
#define LEFT 1
#define RIGHT 0
#define VITRUL_POINT -1

void ROB_MOVE(int id, int direction);

void ROB_GET(int id);

void ROB_PULL(int id);

void SHIP_MOVE(int id, int target);

void NextZhen();

void compileCommand();

std::vector<std::string> commandOut; // 输出队列

void updateAction() {

    for (std::size_t i = 0; i < 10; i++) {
        if (robot[i].isBusy) {
            if (!robot[i].path.empty()) {
                robot[i].path.erase(robot[i].path.begin());
                // 计算下一步移动动作
                if (robot[i].path.front().x - robot[i].x == 1)
                    robot[i].actionMove = MOVE_RIGHT;
                else if (robot[i].path.front().x - robot[i].x == -1)
                    robot[i].actionMove = MOVE_LEFT;
                else if (robot[i].path.front().y - robot[i].y == 1)
                    robot[i].actionMove = MOVE_FONT;
                else if (robot[i].path.front().y - robot[i].y == -1)
                    robot[i].actionMove = MOVE_BACK;
                else
                    robot[i].actionMove = NO_ACTION;
            } else {
                if (chNode[robot[i].x][robot[i].y].canPull)
                    robot[i].actionAfter = ACTION_PULL;
                else robot[i].actionBefore = ACTION_GET;
            }
        }

    }
}

void ROB_MOVE(int id, int direction) {
    // 每次移动加入待刷新状态列表
    nodeToFlush.push(chNode[robot[id].x][robot[id].y]);
    nodeToFlush.push(chNode[robot[id].x + 1][robot[id].y]);
    nodeToFlush.push(chNode[robot[id].x][robot[id].y + 1]);
    nodeToFlush.push(chNode[robot[id].x - 1][robot[id].y]);
    nodeToFlush.push(chNode[robot[id].x][robot[id].y - 1]);

    commandOut.emplace_back("move %d %d", id, direction);
}

/// @brief 拿起货物
/// @param id 机器人id
void ROB_GET(int id) {
    chNode[robot[id].x][robot[id].y].canGet = false;
    commandOut.emplace_back("get %d", id);
    // if (printf("get %d\n", id))
    //     return true;
    // else
    //     return false;
}

/// @brief 放下货物
/// @param id 机器人id
/// @return 执行结果
void ROB_PULL(int id) {
    commandOut.emplace_back("pull %d", id);
    // if (printf("pull %d\n", id))
    //     return true;
    // else
    //     return false;
}

/// @brief 移动船
/// @param id 船id
/// @param target 目标点id
void SHIP_MOVE(int id, int target) {
    if (target == -1)
        // printf("go %d\n", id);
        commandOut.emplace_back("go %d", id);
    else
        // printf("ship %d %d\n", id, target);
        commandOut.emplace_back("ship %d %d", id, target);
}

/// @brief 下一帧
void NextZhen() {
    updateAction();
    compileCommand();
    // TODO: 如果输出出现问题就写命令排序逻辑
    for (size_t i = 0; i < commandOut.size(); i++) {
        std::cout << commandOut[i];
        std::cout << "\n";
    }
    commandOut.clear();
    puts("OK\n");
    fflush(stdout);
}

void compileCommand() {
    //生成机器人移动前指令
    for (size_t i = 0; i < 10; i++) {
        if (robot[i].actionBefore == ACTION_PULL)
            ROB_PULL(i);
        else if (robot[i].actionBefore == ACTION_GET)
            ROB_GET(i);
    }

    // 生成机器人移动指令
    for (size_t i = 0; i < 10; i++) {
        switch (robot[i].actionMove) {
            case MOVE_FONT:
                ROB_MOVE(i, FONT);
                break;
            case MOVE_BACK:
                ROB_MOVE(i, BACK);
                break;

            case MOVE_LEFT:
                ROB_MOVE(i, LEFT);
                break;
            case MOVE_RIGHT:
                ROB_MOVE(i, RIGHT);

        }

    }

    //生成机器人移动后指令
    for (size_t i = 0; i < 10; i++) {
        if (robot[i].actionAfter == ACTION_PULL)
            ROB_PULL(i);
        else if (robot[i].actionAfter == ACTION_GET)
            ROB_GET(i);
    }
    //TODO: 生成船指令
}

ARAStar::ARAStar(Node start, Node goal, double weight, int heuristic)
        : s_start(start), s_goal(goal), e(weight), heuristic_type(heuristic) {
    // 初始化代价字典
    g[s_start] = 0;
    g[s_goal] = INF;
    // 将起点加入优先集合
    OPEN[s_start] = f_value(s_start);
    PARENT[s_start] = s_start;
    // 初始化e
    e = weight;
}

ARAStar::~ARAStar() {}

void ARAStar::search() {
fprintf(stderr,"ARAStar Start Search!!!!!!!\n");
    improvePath();
fprintf(stderr,"ARAStar Path improved!!!!!\n");
    extract_path();
fprintf(stderr,"ARAStar Path Result!!!!!\n");
    while (update_e() > 1) {
        e -= 0.4;
        for (auto &node: INCOMES)
            OPEN.emplace(node, INF);
        for (auto &open: OPEN)
            OPEN[open.first] = f_value(open.first);
        INCOMES.clear();
        CLOSED.clear();
        improvePath();
        extract_path();
fprintf(stderr,"ARAStar REREREREPath Finished!!!!!\n");
    }
}

void ARAStar::improvePath() {
    // A*算法搜索修改路径
    std::pair<Node, double> nowNode = calc_smallest_f();
fprintf(stderr,"calc_smallest_f()!!!!!\n");
    while (f_value(s_goal) > nowNode.second) {
fprintf(stderr,"While IN!!!!!\n");

//        Node smallest = calc_smallest_f();
        OPEN.erase(nowNode.first); // TODO:这样写没问题吗？
fprintf(stderr,"OPEN.erase(nowNode.first);!!!!!\n");

        CLOSED.emplace(nowNode.first);
fprintf(stderr,"CLOSED.emplace(nowNode.first);!!!!!\n");

        for (Node node: nodeNear(nowNode.first)) {
            if (node.block) continue;
            if (!VISITED.count(node))
                g[node] = INF;
            else if (g[node] > g[nowNode.first] + 1) {
                g[node] = g[nowNode.first] + 1;
                PARENT[node] = nowNode.first;
                VISITED.emplace(node);
                if (!CLOSED.count(node))
                    OPEN[node] = f_value(node);
                else
                    INCOMES.emplace(node);
            }
        }
        nowNode = calc_smallest_f();
    }
}

std::pair<Node, double> ARAStar::calc_smallest_f() {
    Node minNode;
    double minValue = std::numeric_limits<double>::max();
    std::pair<Node, double> minPair;

    for (const auto &pair: OPEN) {
        if (pair.second < minValue) {
            minValue = pair.second;
//            minNode = pair.first;
            minPair = pair;
        }
    }
    return minPair;
}

//返回周围的Node
std::vector<Node> nodeNear(Node node) {
//    chNodeMutex.lock();
    std::lock_guard<std::mutex> locker(chNodeMutex);
    std::vector<Node> nodenear;
//    Node node111 =  chNode[node.x - 1][node.y];
    nodenear.push_back(chNode[node.x + 1][node.y]);
    nodenear.push_back(chNode[node.x - 1][node.y]);
    nodenear.push_back(chNode[node.x][node.y + 1]);
    nodenear.push_back(chNode[node.x][node.y - 1]);
//    chNodeMutex.unlock();
    return nodenear;
}

// GPT转换，注意检查
double ARAStar::update_e() {
    double v = INF;

    for (auto const &s: OPEN) {
        double tentative_g = g[s.first];
        double tentative_h = h(s.first);
        v = std::min(v, tentative_g + tentative_h);
    }
    for (auto const &s: INCOMES) {
        double tentative_g = g[s];
        double tentative_h = h(s);
        v = std::min(v, tentative_g + tentative_h);
    }
    double goal_g = g[s_goal];
    return std::min(e, goal_g / v);
}

void ARAStar::extract_path() {

    path.push_back(s_goal);
    Node node = s_goal;
    while (node.x != s_start.x && node.y != s_start.y) {
        node = PARENT[node];
        path.push_back(node);
    }
    std::reverse(path.begin(), path.end());
}

double ARAStar::f_value(Node node) {
    return g[node] + e * (abs(node.x - s_goal.x) + abs(node.y - s_goal.y));
}

int ARAStar::h(Node node) {
    if (heuristic_type)
        return abs(node.x - s_goal.x) + abs(node.y - s_goal.y);
    else
        return sqrt(pow(node.x - s_goal.x, 2) + pow(node.y - s_goal.y, 2));
}


Node findGood(Robot robot);

Node findBerth(Robot robotNow);

using namespace std;

// 初始化
void Init() {
    for (int i = 1; i <= n; i++) {
        scanf("%s", ch[i] + 1);
        for (int ii = 1; ii <= n; ii++) {
            if (ch[i][ii] == '#' || ch[i][ii] == '*') {
                chNode[i][ii] = {i, ii, STICKBlLOCK, false, false};
            } else if (ch[i][ii] == 'B') {
                chNode[i][ii] = {i, ii, UNBLOCK, true, false};
            } else if (ch[i][ii] == 'A') {
                chNode[i][ii] = {i, ii, MOVEBLOCK, true, false};
            } else {
                chNode[i][ii] = {i, ii, UNBLOCK, false, false};
            }
        }
    }
    // 初始化港口信息
    for (int i = 0; i < berth_num; i++) {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
    }
    scanf("%d", &boat_capacity); // 读入船只容量
     char okk[100];
     scanf("%s", okk);
    printf("OK\n"); // 初始化完毕
    fflush(stdout);
}

// 每一帧读入判题器信息
int Input() {
    scanf("%d%d", &id, &money);
    //处理货物
    int num;//货物数量
    scanf("%d", &num);
    for (int i = 1; i <= num; i++) {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        Good good(x, y, val);
        //给货物绑定港口
        for (int j = 0; j < berth_num; ++j) {
            double rowNow = double(good.price) / (abs(berth[j].x - good.x) + abs(berth[j].y - good.y));
            if (rowNow > good.row) {
                good.row = rowNow;
                good.berth = j;
            }
        }
        goodsList.insert(good);
        //处理Node
        chNode[good.x][good.y].canGet = true;
    }
    for (int i = 0; i < robot_num; i++) {
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &robot[i].status);
    }
    for (int i = 0; i < 5; i++)
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);
     char okk[100];
     scanf("%s", okk);
    fflush(stdin);
fprintf(stderr,"INPUT OK!!!!!!!\n");
    return id;
}

ARAStar araStar[robot_num];
thread pathFinder[robot_num];

int main() {
    Init();
    // 读取地图、泊位和船的容积
    // PreprocessMap();
    for (int zhen = 1; zhen <= 15000; zhen++) {
        // 每一帧id的交互命令

        Input();
        // Input函数读取当前帧的所有信息

        for (int i = 0; i < robot_num; i++) {
            if (robot[i].status == 1) {
                // 正常情况下
                // 若机器人没有规划路径，则指定目标物品。
                if (!robot[i].isBusy) {
                    //开线程计算路径
                    araStar[i] = *new ARAStar(chNode[robot[i].x][robot[i].y], findGood(robot[i]), 2.0, 1);
                    pathFinder[i] = thread(&ARAStar::search, &araStar[i]);
                    pathFinder[i].detach();
                    fprintf(stderr,"PATH %d CAL!!!!!!!\n",i);

                } else if (robot[i].path.empty()) {
                    //开线程计算路径
                    araStar[i] = *new ARAStar(chNode[robot[i].x][robot[i].y], findBerth(robot[i]), 2.0, 1);
                    pathFinder[i] = thread(&ARAStar::search, &araStar[i]);
                    pathFinder[i].detach();
                    fprintf(stderr,"PATH %d CAL!!!!!!!\n",i);

                }
            }
        }
        sleep(10);
        for (int i = 0; i < robot_num; i++) {
            if (robot[i].status == 1) {
                // 正常情况下
                // 若机器人没有规划路径，则指定目标物品。
                if (!robot[i].isBusy) {
                    //开线程计算路径
                    robot[i].path = araStar[i].path;
                } else if (robot[i].path.empty()) {
                    //开线程计算路径
                    robot[i].path = araStar[i].path;
                }
            }
        }
        NextZhen();
    }
    return 0;
}

Node findBerth(Robot robotNow) {
    return chNode[berth[robotNow.goodOn.berth].x + 4][berth[robotNow.goodOn.berth].y + 4];
}

Node findGood(Robot robotNow) {
    double rowBefore;
    for (auto &good: goodsList) {
        double rowNow = double(good.price) / (abs(robotNow.x - good.x) + abs(robotNow.y - good.y));
        if (rowNow > rowBefore) {
            robotNow.goodOn = good;
            rowBefore = rowNow;
        }
    }
    return chNode[robotNow.goodOn.x][robotNow.goodOn.y];
}
