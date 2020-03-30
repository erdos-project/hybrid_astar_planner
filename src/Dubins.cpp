#include "include/Dubins.h"

#include <cmath>

// Find the shortest Dubin's path from the start to end
DubinsPath Dubins::getShortestPath() {
    vector<DubinsPath> paths;
    DubinsPath shortest_path;
    double cost, shortest_cost;
    shortest_cost = INFINITY;

    paths = calcPaths();
    for (const auto& path: paths) {
        cost = 0;
        for (auto p: path) {
            if (p.first == direction_t::straight) {
                cost += p.second;
            }
            else {
                cost += p.second * radius;
            }
        }
        if (cost < shortest_cost) {
            shortest_path = path;
            shortest_cost = cost;
        }
    }

    return shortest_path;
}

// Construct the x, y, yaw path given a starting pose, Dubin's path and radius
// Arguments:
//      s: initial pose x, y, yaw
//      path: Dubin's path to construct
//      radius: turning radius of vehicle
// Returns:
//      vector of pose x, y, yaw
vector<Pose> Dubins::generatePath(Pose s, DubinsPath path, double radius) {
    vector<double> r_x, r_y, r_yaw, cur, center;
    double yaw, ang, ang_start, ang_end, step;
    int tick;
    vector<Pose> ret;

    cur = s;
    yaw = s[2];
    for (auto p: path) {
        if (p.first == direction_t::straight) {
            if (p.second > 0) {
                tick = 1;
            } else {
                tick = -1;
            }
            for (int i = 0; i < p.second; i+=tick) {
                r_x.push_back(cur[0] + cos(yaw) * i);
                r_y.push_back(cur[1] + sin(yaw) * i);
                r_yaw.push_back(yaw);
            }
            r_x.push_back(cur[0] + cos(yaw) * p.second);
            r_y.push_back(cur[1] + sin(yaw) * p.second);
            r_yaw.push_back(yaw);
        } else {
            center = calcTurnCenter(cur, p.first, radius);
            ang_start = atan2(cur[1] - center[1], cur[0] - center[0]);
            if (p.first == direction_t::left) {
                ang_end = ang_start + p.second;
            }
            else {
                ang_end = ang_start - p.second;
            }
            if (ang_start < ang_end) {
                step = (1 / radius);
            }
            else {
                step = (-1 / radius);
            }
            ang = ang_start;
            for (int i = 0; i < (ang_end - ang_start) / step; i+=1) {
                r_x.push_back(center[0] + cos(ang) * radius);
                r_y.push_back(center[1] + sin(ang) * radius);
                r_yaw.push_back(yaw);
                yaw += step;
                ang += step;
            }
            r_x.push_back(center[0] + cos(ang_end) * radius);
            r_y.push_back(center[1] + sin(ang_end) * radius);
            if (p.first == direction_t::left) {
                yaw = cur[2] + p.second;
            }
            else {
                yaw = cur[2] - p.second;
            }
            r_yaw.push_back(yaw);
        }
        cur.clear();
        cur.push_back(r_x.back());
        cur.push_back(r_y.back());
        cur.push_back(yaw);
    }
    for (size_t i = 0; i < r_x.size(); i++) {
        Pose p({r_x[i], r_y[i], r_yaw[i]});
        ret.push_back(p);
    }
    return ret;
}

// Mod theta (radians) to range [0, 2 PI)
double Dubins::mod2Pi(double theta) {
    return theta - 2.0 * M_PI * floor(theta / 2.0 / M_PI);
}

// Calculate the delta in x, y, yaw between the start and end
Pose Dubins::calcEnd() {
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

// Compute Dubin's path for L(eft), S(traight), L(eft) sequence
DubinsPath Dubins::calcLSL(Pose e) {
    double x, y, yaw, u, t, v;
    DubinsPath dp;

    yaw = e[2];
    x = e[0] - sin(yaw);
    y = e[1] - 1 + cos(yaw);
    u = sqrt(pow(x, 2) + pow(y, 2));
    t = mod2Pi(atan2(y, x));
    v = mod2Pi(yaw - t);

    DubinsPoint first (direction_t::left, t);
    DubinsPoint second (direction_t::straight, u * radius);
    DubinsPoint third (direction_t::left, v);

    dp.push_back(first);
    dp.push_back(second);
    dp.push_back(third);
    return dp;
}

// Compute Dubin's path for R(ight), S(traight), R(ight) sequence
DubinsPath Dubins::calcRSR(Pose e) {
    Pose e_prime(e);
    DubinsPath path;

    e_prime[1] = -e[1];
    e_prime[2] = mod2Pi(-e[2]);
    path = calcLSL(e_prime);
    path[0].first = direction_t::right;
    path[2].first = direction_t::right;
    return path;
}

// Compute Dubin's path for L(eft), S(traight), R(ight) sequence
DubinsPath Dubins::calcLSR(Pose e) {
    double x, y, yaw, u1_square, t1, u, theta, t, v;
    DubinsPoint first, second, third;
    DubinsPath dp;

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
    first = make_pair(direction_t::left, t);
    second = make_pair(direction_t::straight, u * radius);
    third = make_pair(direction_t::right, v);

    dp.push_back(first);
    dp.push_back(second);
    dp.push_back(third);

    return dp;
}

// Compute Dubin's path for R(ight), S(traight), L(eft) sequence
DubinsPath Dubins::calcRSL(Pose e) {
    Pose e_prime(e);
    DubinsPath path;

    e_prime[1] = -e[1];
    e_prime[2] = mod2Pi(-e[2]);
    path = calcLSR(e_prime);
    if (path.empty()) {
        return path;
    }
    path[0].first = direction_t::right;
    path[2].first = direction_t::left;
    return path;
}

// Compute Dubin's path for L(eft), R(ight), L(eft) sequence
DubinsPath Dubins::calcLRL(Pose e) {
    double x, y, yaw, u1, t1, theta, t, u, v;
    DubinsPoint first, second, third;
    DubinsPath dp;

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
    first = make_pair(direction_t::left, t);
    second = make_pair(direction_t::right, u);
    third = make_pair(direction_t::left, v);

    dp.push_back(first);
    dp.push_back(second);
    dp.push_back(third);

    return dp;
}

// Compute Dubin's path for R(ight), L(eft), R(ight) sequence
DubinsPath Dubins::calcRLR(Pose e) {
    Pose e_prime(e);
    DubinsPath path;

    e_prime[1] = -e[1];
    e_prime[2] = mod2Pi(-e[2]);
    path = calcLRL(e_prime);
    if (path.empty()) {
        return path;
    }
    path[0].first = direction_t::right;
    path[1].first = direction_t::left;
    path[2].first = direction_t::right;
    return path;
}

// Compute all possible Dubin's Paths from start to end
vector<DubinsPath> Dubins::calcPaths() {
    DubinsPath
        lsl, rsr, lsr, rsl, rlr, lrl;
    vector<DubinsPath>
        all_paths;
    Pose e;

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