#include "DubinsPath.h"

#include <cmath>

tuple<pair<char, double>, pair<char, double>, pair<char, double> >
        DubinsPath::getShortestPath() {

}

vector<tuple<double, double, double> > DubinsPath::generatePath() {

}

double DubinsPath::mod2Pi(double theta) {
    return theta - 2.0 * M_PI * floor(theta / 2.0 / M_PI);
}

// Calculate the delta in x, y, yaw between the start and end
tuple<double, double, double> DubinsPath::calcEnd() {
    double ex, ey, yaw, lex, ley, leyaw;

    ex = get<0>(end) - get<0>(start);
    ey = get<1>(end) - get<1>(start);
    yaw = get<2>(start);

    lex = (cos(yaw) * ex + sin(yaw) * ey) / radius;
    ley = (-sin(yaw) * ex + cos(yaw) * ey) / radius;
    leyaw = get<2>(end) - get<2>(start);
    return make_tuple(lex, ley, leyaw);
}

// Calculate the turning center given a pose and direction (l = left, r = right)
tuple<double, double> DubinsPath::calcTurnCenter(
        tuple<double, double, double> point, char dir) {
    double x, y;
    double ang = get<2>(point);

    if (dir == 'l') ang += M_PI / 2.0;
    else if (dir == 'r') ang -= M_PI / 2.0;

    x = get<0>(point) + cos(ang) * radius;
    y = get<1>(point) * sin(ang) * radius;
    return make_tuple(x, y);
}

// Compute Dubin's path for L(eft), S(traight), L(eft) sequence
tuple<pair<char, double>, pair<char, double>, pair<char, double>>
DubinsPath::calcLSL(tuple<double, double, double> e) {
    double x, y, yaw, u, t, v;

    yaw = get<2>(e);
    x = get<0>(e) - sin(yaw);
    y = get<1>(e) - 1 + cos(yaw);
    u = sqrt(exp2(x) + exp2(y));
    t = mod2Pi(atan2(y, x));
    v = mod2Pi(yaw - t);

    pair<char, double> first ('l', t);
    pair<char, double> second ('s', u * radius);
    pair<char, double> third ('l', v);

    return make_tuple(first, second, third);
}

// Compute Dubin's path for R(ight), S(traight), R(ight) sequence
tuple<pair<char, double>, pair<char, double>, pair<char, double>>
DubinsPath::calcRSR(tuple<double, double, double> e) {
    tuple<double, double, double> e_prime;
    tuple<pair<char, double>, pair<char, double>, pair<char, double>> path;

    e_prime = make_tuple(get<0>(e), -get<1>(e), mod2Pi(-get<2>(e)));
    path = calcLSL(e_prime);
    get<0>(path).first = 'r';
    get<2>(path).first = 'r';
    return path;
}

// Compute Dubin's path for L(eft), S(traight), R(ight) sequence
tuple<pair<char, double>, pair<char, double>, pair<char, double>>
DubinsPath::calcLSR(tuple<double, double, double> e) {
    double x, y, yaw, u1_square, t1, u, theta, t, v;

    yaw = get<2>(e);
    x = get<0>(e) + sin(yaw);
    y = get<1>(e) - 1 - cos(yaw);
    u1_square = exp2(x) + exp2(y);
    if (u1_square < 4.0) {
        pair<char, double> first ('x', 0);
        pair<char, double> second ('x', 0);
        pair<char, double> third ('x', 0);

        return make_tuple(first, second, third);
    }
    t1 = mod2Pi(atan2(y, x));
    u = sqrt(u1_square - 4);
    theta = mod2Pi(atan(2 / u));
    t = mod2Pi(t1 + theta);
    v = mod2Pi(t - yaw);

    pair<char, double> first ('l', t);
    pair<char, double> second ('s', u * radius);
    pair<char, double> third ('r', v);

    return make_tuple(first, second, third);
}

// Compute Dubin's path for R(ight), S(traight), L(eft) sequence
tuple<pair<char, double>, pair<char, double>, pair<char, double>>
DubinsPath::calcRSL(tuple<double, double, double> e) {
    tuple<double, double, double> e_prime;
    tuple<pair<char, double>, pair<char, double>, pair<char, double>> path;

    e_prime = make_tuple(get<0>(e), -get<1>(e), mod2Pi(-get<2>(e)));
    path = calcLSR(e_prime);
    if (get<0>(path).first == 'x') {
        return path;
    }
    get<0>(path).first = 'r';
    get<2>(path).first = 'l';
    return path;
}

// Compute Dubin's path for L(eft), R(ight), L(eft) sequence
tuple<pair<char, double>, pair<char, double>, pair<char, double>>
DubinsPath::calcLRL(tuple<double, double, double> e) {
    double x, y, yaw, u1, t1, theta, t, u, v;

    yaw = get<2>(e);
    x = get<0>(e) - sin(yaw);
    y = get<1>(e) - 1 + cos(yaw);
    u1 = sqrt(exp2(x) + exp2(y));
    if (u1 > 4.0) {
        pair<char, double> first ('x', 0);
        pair<char, double> second ('x', 0);
        pair<char, double> third ('x', 0);

        return make_tuple(first, second, third);
    }
    t1 = atan2(y, x);
    theta = acos(u1 / 4.0);
    t = mod2Pi(M_PI_2 + t1 + theta);
    u = mod2Pi(M_PI + 2 * theta);
    v = mod2Pi(M_PI_2 - t1 + theta + yaw);

    pair<char, double> first ('l', t);
    pair<char, double> second ('r', u * radius);
    pair<char, double> third ('l', v);

    return make_tuple(first, second, third);
}

// Compute Dubin's path for R(ight), L(eft), R(ight) sequence
tuple<pair<char, double>, pair<char, double>, pair<char, double>>
DubinsPath::calcRLR(tuple<double, double, double> e) {
    tuple<double, double, double> e_prime;
    tuple<pair<char, double>, pair<char, double>, pair<char, double>> path;

    e_prime = make_tuple(get<0>(e), -get<1>(e), mod2Pi(-get<2>(e)));
    path = calcLRL(e_prime);
    if (get<0>(path).first == 'x') {
        return path;
    }
    get<0>(path).first = 'r';
    get<1>(path).first = 'l';
    get<2>(path).first = 'r';
    return path;
}

vector<tuple<pair<char, double>, pair<char, double>, pair<char, double>>>
DubinsPath::calcPaths() {
    tuple<pair<char, double>, pair<char, double>, pair<char, double>>
        lsl, rsr, lsr, rsl, rlr, lrl;
    vector<tuple<pair<char, double>, pair<char, double>, pair<char, double>>>
        all_paths;
    tuple<double, double, double> e;

    e = calcEnd();
    lsl = calcLSL(e);
    rsr = calcRSR(e);
    lsr = calcLSR(e);
    rsl = calcRSL(e);
    rlr = calcRLR(e);
    lrl = calcLRL(e);

    all_paths.push_back(lsl);
    all_paths.push_back(rsr);
    if (get<0>(lsr).first != 'x') all_paths.push_back(lsr);
    if (get<0>(rsl).first != 'x') all_paths.push_back(lsr);
    if (get<0>(rlr).first != 'x') all_paths.push_back(lsr);
    if (get<0>(lrl).first != 'x') all_paths.push_back(lsr);

    return all_paths;
}