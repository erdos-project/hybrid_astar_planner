#include "DubinsPath.h"

#include <cmath>

vector<pair<char, double>> DubinsPath::getShortestPath() {
    vector<vector<pair<char, double>>> paths;
    vector<pair<char, double>> shortest_path;
    double cost, shortest_cost;
    shortest_cost = INFINITY;

    paths = calcPaths();
    for (const auto& path: paths) {
        cost = 0;
        for (auto p: path) {
            if (p.first == 's') cost += p.second;
            else cost += p.second * radius;
        }
        if (cost < shortest_cost) {
            shortest_path = path;
            shortest_cost = cost;
        }
    }
    return shortest_path;
}

vector<vector<double>>
DubinsPath::generatePath(vector<double> s, vector<pair<char, double>> path) {
    vector<double> r_x, r_y, r_yaw, cur, center;
    double yaw, ang, ang_start, ang_end, step;
    int tick;
    vector<vector<double>> ret;

    cur = s;
    yaw = s[2];
    for (auto p: path) {
        if (p.first == 's') {
            if (p.second > 0) {
                tick = 1;
            } else {
                tick = -1;
            }
            for (int i = 0; i < 10 * p.second; i+=tick) {
                r_x.push_back(cur[0] + cos(yaw) * i / 10.0);
                r_y.push_back(cur[1] + sin(yaw) * i / 10.0);
                r_yaw.push_back(yaw);
            }
            r_x.push_back(cur[0] + cos(yaw) * p.second);
            r_y.push_back(cur[1] + sin(yaw) * p.second);
            r_yaw.push_back(yaw);
        } else {
            center = calcTurnCenter(cur, p.first);
            ang_start = atan2(cur[1] - center[1], cur[0] - center[0]);
            if (p.first == 'l') ang_end = ang_start + p.second;
            else ang_end = ang_start - p.second;
            if (ang_start < ang_end) step = (0.1 / radius);
            else step = (-0.1 / radius);
            ang = ang_start;
            while (ang < ang_end) {
                r_x.push_back(center[0] + cos(ang) * radius);
                r_y.push_back(center[1] + sin(ang) * radius);
                r_yaw.push_back(yaw);
                yaw += step;
                ang += step;
            }
            r_x.push_back(center[0] + cos(ang_end) * radius);
            r_y.push_back(center[1] + sin(ang_end) * radius);
            if (p.first == 'l') yaw = cur[2] + p.second;
            else yaw = cur[2] - p.second;
            r_yaw.push_back(yaw);
        }
        cur.clear();
        cur.push_back(r_x.back());
        cur.push_back(r_y.back());
        cur.push_back(r_yaw.back());
    }
    ret.push_back(r_x);
    ret.push_back(r_y);
    ret.push_back(r_yaw);
    return ret;
}

double DubinsPath::mod2Pi(double theta) {
    return theta - 2.0 * M_PI * floor(theta / 2.0 / M_PI);
}

// Calculate the delta in x, y, yaw between the start and end
vector<double> DubinsPath::calcEnd() {
    double ex, ey, yaw, lex, ley, leyaw;
    vector<double> dend;

    ex = end[0] - start[0];
    ey = end[1] - start[1];
    yaw = start[2];

    lex = (cos(yaw) * ex + sin(yaw) * ey) / radius;
    ley = (-sin(yaw) * ex + cos(yaw) * ey) / radius;
    leyaw = end[2] - start[2];

    dend.push_back(lex);
    dend.push_back(ley);
    dend.push_back(leyaw);
    return dend;
}

// Calculate the turning center given a pose and direction (l = left, r = right)
vector<double> DubinsPath::calcTurnCenter(vector<double> point, char dir) {
    double x, y, ang;
    vector<double> turn_center;

    ang = point[2];
    if (dir == 'l') ang += M_PI / 2.0;
    else if (dir == 'r') ang -= M_PI / 2.0;

    x = point[0] + cos(ang) * radius;
    y = point[1] * sin(ang) * radius;
    turn_center.push_back(x);
    turn_center.push_back(y);
    return turn_center;
}

// Compute Dubin's path for L(eft), S(traight), L(eft) sequence
vector<pair<char, double>> DubinsPath::calcLSL(vector<double> e) {
    double x, y, yaw, u, t, v;
    vector<pair<char, double>> dp;

    yaw = e[2];
    x = e[0] - sin(yaw);
    y = e[1] - 1 + cos(yaw);
    u = sqrt(pow(x, 2) + pow(y, 2));
    t = mod2Pi(atan2(y, x));
    v = mod2Pi(yaw - t);

    pair<char, double> first ('l', t);
    pair<char, double> second ('s', u * radius);
    pair<char, double> third ('l', v);

    dp.push_back(first);
    dp.push_back(second);
    dp.push_back(third);
    return dp;
}

// Compute Dubin's path for R(ight), S(traight), R(ight) sequence
vector<pair<char, double>> DubinsPath::calcRSR(vector<double> e) {
    vector<double> e_prime(e);
    vector<pair<char, double>> path;

    e_prime[1] = -e[1];
    e_prime[2] = mod2Pi(-e[2]);
    path = calcLSL(e_prime);
    path[0].first = 'r';
    path[2].first = 'r';
    return path;
}

// Compute Dubin's path for L(eft), S(traight), R(ight) sequence
vector<pair<char, double>> DubinsPath::calcLSR(vector<double> e) {
    double x, y, yaw, u1_square, t1, u, theta, t, v;
    pair<char, double> first, second, third;
    vector<pair<char, double>> dp;

    yaw = e[2];
    x = e[0] + sin(yaw);
    y = e[1] - 1 - cos(yaw);
    u1_square = pow(x, 2) + pow(y, 2);
    if (u1_square <= 4.0) {
        return dp;
    }
    t1 = mod2Pi(atan2(y, x));
    u = sqrt(u1_square - 4);
    theta = mod2Pi(atan(2 / u));
    t = mod2Pi(t1 + theta);
    v = mod2Pi(t - yaw);
    first = make_pair('l', t);
    second = make_pair('s', u * radius);
    third = make_pair('r', v);

    dp.push_back(first);
    dp.push_back(second);
    dp.push_back(third);

    return dp;
}

// Compute Dubin's path for R(ight), S(traight), L(eft) sequence
vector<pair<char, double>>
DubinsPath::calcRSL(vector<double> e) {
    vector<double> e_prime(e);
    vector<pair<char, double>> path;

    e_prime[1] = -e[1];
    e_prime[2] = mod2Pi(-e[2]);
    path = calcLSR(e_prime);
    if (path.empty()) {
        return path;
    }
    path[0].first = 'r';
    path[2].first = 'l';
    return path;
}

// Compute Dubin's path for L(eft), R(ight), L(eft) sequence
vector<pair<char, double>>
DubinsPath::calcLRL(vector<double> e) {
    double x, y, yaw, u1, t1, theta, t, u, v;
    pair<char, double> first, second, third;
    vector<pair<char, double>> dp;

    yaw = e[2];
    x = e[0] - sin(yaw);
    y = e[1] - 1 + cos(yaw);
    u1 = sqrt(pow(x, 2) + pow(y, 2));
    if (u1 > 4.0) {
        return dp;
    }
    t1 = atan2(y, x);
    theta = acos(u1 / 4.0);
    t = mod2Pi(M_PI_2 + t1 + theta);
    u = mod2Pi(M_PI + 2 * theta);
    v = mod2Pi(M_PI_2 - t1 + theta + yaw);
    first = make_pair('l', t);
    second = make_pair('r', u);
    third = make_pair('l', v);

    dp.push_back(first);
    dp.push_back(second);
    dp.push_back(third);

    return dp;
}

// Compute Dubin's path for R(ight), L(eft), R(ight) sequence
vector<pair<char, double>>
DubinsPath::calcRLR(vector<double> e) {
    vector<double> e_prime(e);
    vector<pair<char, double>> path;

    e_prime[1] = -e[1];
    e_prime[2] = mod2Pi(-e[2]);
    path = calcLRL(e_prime);
    if (path.empty()) {
        return path;
    }
    path[0].first = 'r';
    path[1].first = 'l';
    path[2].first = 'r';
    return path;
}

vector<vector<pair<char, double>>>
DubinsPath::calcPaths() {
    vector<pair<char, double>>
        lsl, rsr, lsr, rsl, rlr, lrl;
    vector<vector<pair<char, double>>>
        all_paths;
    vector<double> e;

    e = calcEnd();
    lsl = calcLSL(e);
    rsr = calcRSR(e);
    lsr = calcLSR(e);
    rsl = calcRSL(e);
    rlr = calcRLR(e);
    lrl = calcLRL(e);

    all_paths.push_back(lsl);
    all_paths.push_back(rsr);
    if (!lsr.empty()) all_paths.push_back(lsr);
    if (!rsl.empty()) all_paths.push_back(rsl);
    if (!rlr.empty()) all_paths.push_back(rlr);
    if (!lrl.empty()) all_paths.push_back(lrl);

    return all_paths;
}