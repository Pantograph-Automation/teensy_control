#include <cmath>
#include <algorithm>
#include <stdexcept>

#define V_MAX 3.0f // rad/s
#define A_MAX 5.0f // ras/s^2

struct TrapezoidProfile
{
    double q0{0.0};      // start position
    double qf{0.0};      // final position
    double dq{0.0};      // signed displacement
    double dir{1.0};     // sign of motion (+1 / -1)

    double v_peak{0.0};  // positive peak velocity actually used
    double t_acc{0.0};   // accel phase duration
    double t_cruise{0.0};
    double t_total{0.0};

    bool triangular{false};
};

struct SynchronizedTrapezoids2D
{
    TrapezoidProfile axis1;
    TrapezoidProfile axis2;
    double T{0.0};       // shared final time
};

inline TrapezoidProfile computeMinimumTimeProfile(double q0,
                                                  double qf)
{

    TrapezoidProfile p;
    p.q0 = q0;
    p.qf = qf;
    p.dq = qf - q0;
    p.dir = (p.dq >= 0.0) ? 1.0 : -1.0;

    const double D = std::abs(p.dq);

    if (D == 0.0) {
        p.v_peak = 0.0;
        p.t_acc = 0.0;
        p.t_cruise = 0.0;
        p.t_total = 0.0;
        p.triangular = true;
        return p;
    }

    const double D_min_for_full_trap = (V_MAX * V_MAX) / A_MAX;

    if (D >= D_min_for_full_trap) {
        // Full trapezoid
        p.triangular = false;
        p.v_peak = V_MAX;
        p.t_acc = V_MAX / A_MAX;
        p.t_cruise = (D - D_min_for_full_trap) / V_MAX;
        p.t_total = 2.0 * p.t_acc + p.t_cruise;
    } else {
        // Triangular
        p.triangular = true;
        p.v_peak = std::sqrt(A_MAX * D);
        p.t_acc = p.v_peak / A_MAX;
        p.t_cruise = 0.0;
        p.t_total = 2.0 * p.t_acc;
    }

    return p;
}

inline TrapezoidProfile computeSynchronizedProfile(double q0,
                                                   double qf,
                                                   double T_sync)
{
    if (T_sync < 0.0) {
        throw std::invalid_argument("T_sync must be >= 0.");
    }

    TrapezoidProfile p;
    p.q0 = q0;
    p.qf = qf;
    p.dq = qf - q0;
    p.dir = (p.dq >= 0.0) ? 1.0 : -1.0;

    const double D = std::abs(p.dq);

    if (D == 0.0) {
        p.v_peak = 0.0;
        p.t_acc = 0.0;
        p.t_cruise = T_sync;
        p.t_total = T_sync;
        p.triangular = true;
        return p;
    }

    // Minimum feasible time for this axis
    const TrapezoidProfile p_min = computeMinimumTimeProfile(q0, qf);
    if (T_sync + 1e-12 < p_min.t_total) {
        throw std::runtime_error("Requested synchronized time is infeasible for this axis.");
    }

    // Solve D = v*T - v^2/a  for v, with fixed a and T.
    // Smaller root is the physically relevant one.
    const double disc = A_MAX * A_MAX * T_sync * T_sync - 4.0 * A_MAX * D;
    const double disc_clamped = std::max(0.0, disc);
    double v_peak = 0.5 * (A_MAX * T_sync - std::sqrt(disc_clamped));

    // Numerical cleanup
    v_peak = std::max(0.0, v_peak);

    // Respect nominal v_max. In theory if T_sync >= Tmin this should hold automatically.
    if (v_peak > V_MAX) {
        v_peak = V_MAX;
    }

    p.v_peak = v_peak;
    p.t_acc = (A_MAX > 0.0) ? (v_peak / A_MAX) : 0.0;
    p.t_cruise = std::max(0.0, T_sync - 2.0 * p.t_acc);
    p.t_total = T_sync;
    p.triangular = (p.t_cruise <= 1e-12);

    return p;
}

inline SynchronizedTrapezoids2D computeSynchronizedTrapezoids2D(double q1_0,
                                                                double q1_f,
                                                                double q2_0,
                                                                double q2_f)
{
    const TrapezoidProfile p1_min = computeMinimumTimeProfile(q1_0, q1_f);
    const TrapezoidProfile p2_min = computeMinimumTimeProfile(q2_0, q2_f);

    const double T_sync = std::max(p1_min.t_total, p2_min.t_total);

    SynchronizedTrapezoids2D out;
    out.T = T_sync;
    out.axis1 = computeSynchronizedProfile(q1_0, q1_f, T_sync);
    out.axis2 = computeSynchronizedProfile(q2_0, q2_f, T_sync);
    return out;
}

struct TrajectorySample
{
    double q{0.0};
    double v{0.0};
    double a{0.0};
};

inline TrajectorySample sampleProfile(const TrapezoidProfile& p, double t)
{
    TrajectorySample s{};

    if (t <= 0.0 || p.t_total <= 0.0) {
        s.q = p.q0;
        s.v = 0.0;
        s.a = 0.0;
        return s;
    }

    if (t >= p.t_total) {
        s.q = p.qf;
        s.v = 0.0;
        s.a = 0.0;
        return s;
    }

    const double ta = p.t_acc;
    const double tc = p.t_cruise;
    const double T  = p.t_total;
    const double a  = A_MAX;
    const double v  = p.v_peak;
    const double sgn = p.dir;

    if (t < ta) {
        // Acceleration
        s.q = p.q0 + sgn * (0.5 * a * t * t);
        s.v = sgn * (a * t);
        s.a = sgn * a;
    }
    else if (t < ta + tc) {
        // Cruise
        const double q_acc = 0.5 * a * ta * ta;
        s.q = p.q0 + sgn * (q_acc + v * (t - ta));
        s.v = sgn * v;
        s.a = 0.0;
    }
    else {
        // Deceleration
        const double dt = T - t;
        s.q = p.qf - sgn * (0.5 * a * dt * dt);
        s.v = sgn * (a * dt);
        s.a = -sgn * a;
    }

    return s;
}