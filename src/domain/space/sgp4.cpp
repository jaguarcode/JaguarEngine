/**
 * @file sgp4.cpp
 * @brief SGP4/SDP4 orbital propagator implementation
 *
 * Based on NORAD SGP4 model (Hoots & Roehrich, 1980)
 * with improvements from Vallado et al.
 *
 * Reference: Revisiting Spacetrack Report #3 (Vallado, Crawford, Hujsak, 2006)
 */

#include "jaguar/domain/space.h"
#include <cmath>

namespace jaguar::domain::space {

// ============================================================================
// Constants
// ============================================================================

namespace {

// WGS-72 Earth constants (used by SGP4)
constexpr Real EARTH_RADIUS = 6378.135;    // km (WGS-72)
constexpr Real XKE = 0.0743669161331734;   // sqrt(GM) in units of (earth radii)^1.5 / min
constexpr Real AE = 1.0;                   // Distance unit (earth radii)
constexpr Real DE2RA = 0.0174532925199433; // Degrees to radians
constexpr Real PI = 3.14159265358979323846;
constexpr Real TWOPI = 2.0 * PI;
constexpr Real MIN_PER_DAY = 1440.0;
constexpr Real XJ2 = 1.082616e-3;
constexpr Real XJ3 = -2.53881e-6;
constexpr Real XJ4 = -1.65597e-6;
constexpr Real CK2 = 0.5 * XJ2 * AE * AE;
constexpr Real CK4 = -0.375 * XJ4 * AE * AE * AE * AE;
constexpr Real S = AE * (1.0 + 78.0 / EARTH_RADIUS);
constexpr Real QOMS2T = 1.880279e-09;

// Deep space threshold
constexpr Real DEEP_SPACE_PERIOD_MIN = 225.0; // minutes

inline Real fmod2p(Real x) {
    Real result = std::fmod(x, TWOPI);
    if (result < 0.0) result += TWOPI;
    return result;
}

} // anonymous namespace

// ============================================================================
// SGP4 Internal Data Structure
// ============================================================================

struct SGP4Propagator::SGP4Data {
    // TLE epoch data
    Real epoch{0.0};      // Minutes from 0h Jan 1 1950

    // SGP4 initialization values
    Real bstar{0.0};
    Real inclo{0.0};
    Real nodeo{0.0};
    Real ecco{0.0};
    Real argpo{0.0};
    Real mo{0.0};
    Real no{0.0};         // Mean motion (rad/min)

    // Derived quantities
    Real a{0.0};
    Real alta{0.0};
    Real altp{0.0};
    Real con41{0.0};
    Real cc1{0.0};
    Real cc4{0.0};
    Real cc5{0.0};
    Real d2{0.0};
    Real d3{0.0};
    Real d4{0.0};
    Real delmo{0.0};
    Real eta{0.0};
    Real argpdot{0.0};
    Real omgcof{0.0};
    Real sinmao{0.0};
    Real t2cof{0.0};
    Real t3cof{0.0};
    Real t4cof{0.0};
    Real t5cof{0.0};
    Real x1mth2{0.0};
    Real x7thm1{0.0};
    Real mdot{0.0};
    Real nodedot{0.0};
    Real xlcof{0.0};
    Real xmcof{0.0};
    Real nodecf{0.0};
    Real irez{0};
    Real aycof{0.0};
    Real cosio{0.0};
    Real cosio2{0.0};
    Real sinio{0.0};

    // Deep space only
    Real gsto{0.0};

    // Output state
    Vec3 position{0, 0, 0};
    Vec3 velocity{0, 0, 0};
};

// ============================================================================
// TLE Parser Implementation
// ============================================================================

TLE TLE::parse(const std::string& line1, const std::string& line2)
{
    TLE tle;
    tle.line1 = line1;
    tle.line2 = line2;

    // Line 1 format:
    // 1 NNNNN U NNNNNAAA NNNNN.NNNNNNNN +.NNNNNNNN +NNNNN-N +NNNNN-N N NNNNN

    if (line1.length() < 69 || line2.length() < 69) {
        return tle;
    }

    // Parse line 1
    tle.satellite_number = std::stoi(line1.substr(2, 5));

    // Epoch year (column 19-20)
    int epoch_year_short = std::stoi(line1.substr(18, 2));
    tle.epoch_year = (epoch_year_short >= 57) ? (1900 + epoch_year_short) : (2000 + epoch_year_short);

    // Epoch day (column 21-32)
    tle.epoch_day = std::stod(line1.substr(20, 12));

    // B* drag term (column 54-61)
    std::string bstar_str = line1.substr(53, 8);
    // Format: +NNNNN-N or -NNNNN-N (implicit decimal)
    Real bstar_mantissa = std::stod(bstar_str.substr(0, 6)) * 1e-5;
    int bstar_exp = std::stoi(bstar_str.substr(6, 2));
    tle.bstar = bstar_mantissa * std::pow(10.0, bstar_exp);

    // Line 2 format:
    // 2 NNNNN NNN.NNNN NNN.NNNN NNNNNNN NNN.NNNN NNN.NNNN NN.NNNNNNNNNNNNNN

    // Inclination (column 9-16) - degrees
    tle.inclination = std::stod(line2.substr(8, 8)) * DE2RA;

    // RAAN (column 18-25) - degrees
    tle.raan = std::stod(line2.substr(17, 8)) * DE2RA;

    // Eccentricity (column 27-33) - implicit decimal point
    tle.eccentricity = std::stod("0." + line2.substr(26, 7));

    // Argument of perigee (column 35-42) - degrees
    tle.arg_of_perigee = std::stod(line2.substr(34, 8)) * DE2RA;

    // Mean anomaly (column 44-51) - degrees
    tle.mean_anomaly = std::stod(line2.substr(43, 8)) * DE2RA;

    // Mean motion (column 53-63) - revolutions per day
    tle.mean_motion = std::stod(line2.substr(52, 11));

    return tle;
}

// ============================================================================
// SGP4 Propagator Implementation
// ============================================================================

SGP4Propagator::SGP4Propagator()
    : data_(std::make_unique<SGP4Data>())
{
}

SGP4Propagator::~SGP4Propagator() = default;

bool SGP4Propagator::initialize(const TLE& tle)
{
    if (tle.line1.empty() || tle.line2.empty()) {
        return false;
    }

    // Convert TLE to SGP4 internal format
    data_->bstar = tle.bstar;
    data_->inclo = tle.inclination;
    data_->nodeo = tle.raan;
    data_->ecco = tle.eccentricity;
    data_->argpo = tle.arg_of_perigee;
    data_->mo = tle.mean_anomaly;

    // Mean motion from rev/day to rad/min
    data_->no = tle.mean_motion * TWOPI / MIN_PER_DAY;

    // Calculate epoch in minutes from 0 Jan 1950
    Real year = tle.epoch_year;
    Real days = tle.epoch_day;

    // Julian date of epoch
    Real jd = 2433281.5 + (year - 1950.0) * 365.0 + std::floor((year - 1949.0) / 4.0) + days;
    data_->epoch = (jd - 2433281.5) * MIN_PER_DAY;

    // Greenwich Sidereal Time at epoch
    Real t_ut1 = (jd - 2451545.0) / 36525.0;
    data_->gsto = std::fmod(67310.54841 + t_ut1 * (876600.0 * 3600.0 + 8640184.812866 +
                            t_ut1 * (0.093104 - t_ut1 * 6.2e-6)), 86400.0) * DE2RA / 240.0;
    data_->gsto = fmod2p(data_->gsto);

    // Determine deep space or near earth
    Real a1 = std::pow(XKE / data_->no, 2.0/3.0);
    data_->cosio = std::cos(data_->inclo);
    data_->cosio2 = data_->cosio * data_->cosio;
    data_->sinio = std::sin(data_->inclo);
    Real temp = 1.5 * CK2 * (3.0 * data_->cosio2 - 1.0) /
                std::pow(1.0 - data_->ecco * data_->ecco, 1.5);
    Real del1 = temp / (a1 * a1);
    Real a0 = a1 * (1.0 - del1 * (1.0/3.0 + del1 * (1.0 + 134.0/81.0 * del1)));
    Real del0 = temp / (a0 * a0);
    data_->no = data_->no / (1.0 + del0);

    // Semi-major axis
    data_->a = std::pow(XKE / data_->no, 2.0/3.0);
    data_->alta = data_->a * (1.0 + data_->ecco) - 1.0;
    data_->altp = data_->a * (1.0 - data_->ecco) - 1.0;

    // Check for deep space (period > 225 minutes)
    Real period_min = TWOPI / data_->no;
    deep_space_ = (period_min >= DEEP_SPACE_PERIOD_MIN);

    // SGP4 initialization
    Real ao = data_->a;
    Real con42 = 1.0 - 5.0 * data_->cosio2;
    Real con41 = -con42 - data_->cosio2 - data_->cosio2;
    data_->con41 = con41;
    Real x1mth2 = 1.0 - data_->cosio2;
    data_->x1mth2 = x1mth2;
    data_->x7thm1 = 7.0 * data_->cosio2 - 1.0;

    Real posq = ao * ao;
    Real rp = ao * (1.0 - data_->ecco);
    Real perige = (rp - 1.0) * EARTH_RADIUS;

    // For perigee below 156 km, use different s and qoms2t
    Real sfour = S;
    Real qzms24 = QOMS2T;
    if (perige < 156.0) {
        sfour = perige - 78.0;
        if (perige < 98.0) {
            sfour = 20.0;
        }
        Real qzms24_temp = std::pow((120.0 - sfour) / EARTH_RADIUS, 4.0);
        qzms24 = qzms24_temp;
        sfour = sfour / EARTH_RADIUS + 1.0;
    }

    Real pinvsq = 1.0 / posq;
    Real tsi = 1.0 / (ao - sfour);
    data_->eta = ao * data_->ecco * tsi;
    Real etasq = data_->eta * data_->eta;
    Real eeta = data_->ecco * data_->eta;
    Real psisq = std::abs(1.0 - etasq);
    Real coef = qzms24 * std::pow(tsi, 4.0);
    Real coef1 = coef / std::pow(psisq, 3.5);
    Real cc2 = coef1 * data_->no * (ao * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq)) +
               0.375 * CK2 * tsi / psisq * con41 * (8.0 + 3.0 * etasq * (8.0 + etasq)));
    data_->cc1 = data_->bstar * cc2;
    Real cc3 = 0.0;
    if (data_->ecco > 1.0e-4) {
        cc3 = -2.0 * coef * tsi * XJ3 * AE * data_->no * data_->sinio / data_->ecco;
    }
    data_->x1mth2 = x1mth2;
    data_->cc4 = 2.0 * data_->no * coef1 * ao * (
        ao * (0.5 * etasq + 2.0 * eeta + 0.5 * data_->ecco * etasq) +
        0.25 * CK2 * tsi / psisq * (-3.0 * con41 * (1.0 - 2.0 * eeta + etasq * (1.5 - 0.5 * eeta)) +
        0.75 * x1mth2 * (2.0 * etasq - eeta * (1.0 + etasq)) * std::cos(2.0 * data_->argpo)));
    data_->cc5 = 2.0 * coef1 * ao * ao * (1.0 + 2.75 * (etasq + eeta) + eeta * etasq);

    Real cosio4 = data_->cosio2 * data_->cosio2;
    Real temp1 = 1.5 * CK2 * pinvsq * data_->no;
    Real temp2 = 0.5 * temp1 * CK2 * pinvsq;
    Real temp3 = -0.46875 * CK4 * pinvsq * pinvsq * data_->no;

    data_->mdot = data_->no + 0.5 * temp1 * (1.0 - etasq) * con41 +
                  0.0625 * temp2 * (1.0 - etasq) * (13.0 - 78.0 * data_->cosio2 + 137.0 * cosio4);
    data_->argpdot = -0.5 * temp1 * con42 + 0.0625 * temp2 *
                     (7.0 - 114.0 * data_->cosio2 + 395.0 * cosio4) +
                     temp3 * (3.0 - 36.0 * data_->cosio2 + 49.0 * cosio4);
    Real xhdot1 = -temp1 * data_->cosio;
    data_->nodedot = xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * data_->cosio2) +
                     2.0 * temp3 * (3.0 - 7.0 * data_->cosio2)) * data_->cosio;
    data_->omgcof = data_->bstar * cc3 * std::cos(data_->argpo);
    data_->xmcof = 0.0;
    if (data_->ecco > 1.0e-4) {
        data_->xmcof = -TWOPI / MIN_PER_DAY * coef * data_->bstar * AE / eeta;
    }
    data_->nodecf = 3.5 * (ao * ao) * xhdot1 * data_->cc1;
    data_->t2cof = 1.5 * data_->cc1;

    // For near earth, compute additional terms
    if (!deep_space_) {
        data_->xlcof = 0.125 * XJ3 * AE * data_->sinio * (3.0 + 5.0 * data_->cosio) /
                       (1.0 + data_->cosio);
        data_->aycof = 0.25 * XJ3 * AE * data_->sinio;
        data_->delmo = std::pow(1.0 + data_->eta * std::cos(data_->mo), 3);
        data_->sinmao = std::sin(data_->mo);
        data_->x7thm1 = 7.0 * data_->cosio2 - 1.0;

        Real cc1sq = data_->cc1 * data_->cc1;
        data_->d2 = 4.0 * ao * tsi * cc1sq;
        Real temp_d3 = data_->d2 * tsi * data_->cc1 / 3.0;
        data_->d3 = (17.0 * ao + sfour) * temp_d3;
        data_->d4 = 0.5 * temp_d3 * ao * tsi * (221.0 * ao + 31.0 * sfour) * data_->cc1;
        data_->t3cof = data_->d2 + 2.0 * cc1sq;
        data_->t4cof = 0.25 * (3.0 * data_->d3 + data_->cc1 * (12.0 * data_->d2 + 10.0 * cc1sq));
        data_->t5cof = 0.2 * (3.0 * data_->d4 + 12.0 * data_->cc1 * data_->d3 +
                       6.0 * data_->d2 * data_->d2 + 15.0 * cc1sq * (2.0 * data_->d2 + cc1sq));
    }

    initialized_ = true;
    return true;
}

bool SGP4Propagator::propagate(Real minutes_since_epoch, Vec3& pos_eci, Vec3& vel_eci)
{
    if (!initialized_) {
        return false;
    }

    Real tsince = minutes_since_epoch;

    // Update for secular gravity and atmospheric drag
    Real xmdf = data_->mo + data_->mdot * tsince;
    Real argpdf = data_->argpo + data_->argpdot * tsince;
    Real nodedf = data_->nodeo + data_->nodedot * tsince;
    Real argpm = argpdf;
    Real mm = xmdf;
    Real t2 = tsince * tsince;
    Real nodem = nodedf + data_->nodecf * t2;
    Real tempa = 1.0 - data_->cc1 * tsince;
    Real tempe = data_->bstar * data_->cc4 * tsince;
    Real templ = data_->t2cof * t2;

    if (!deep_space_) {
        Real delomg = data_->omgcof * tsince;
        Real delmtemp = 1.0 + data_->eta * std::cos(xmdf);
        Real delm = data_->xmcof * (delmtemp * delmtemp * delmtemp - data_->delmo);
        Real temp = delomg + delm;
        mm = xmdf + temp;
        argpm = argpdf - temp;
        Real t3 = t2 * tsince;
        Real t4 = t3 * tsince;
        tempa = tempa - data_->d2 * t2 - data_->d3 * t3 - data_->d4 * t4;
        tempe = tempe + data_->bstar * data_->cc5 * (std::sin(mm) - data_->sinmao);
        templ = templ + data_->t3cof * t3 + t4 * (data_->t4cof + tsince * data_->t5cof);
    }

    Real nm = data_->no;
    Real em = data_->ecco;
    Real inclm = data_->inclo;

    // Note: For deep space, additional secular terms would be added here

    Real am = std::pow(XKE / nm, 2.0/3.0) * tempa * tempa;
    nm = XKE / std::pow(am, 1.5);
    em = em - tempe;

    // Clamp eccentricity
    if (em < 1.0e-6) em = 1.0e-6;
    if (em > 1.0 - 1.0e-6) em = 1.0 - 1.0e-6;

    mm = mm + data_->no * templ;
    Real xlm = mm + argpm + nodem;

    nodem = fmod2p(nodem);
    argpm = fmod2p(argpm);
    xlm = fmod2p(xlm);
    mm = fmod2p(xlm - argpm - nodem);

    // Compute extra terms for short-period periodics
    // Note: cosim/sinim would be used in deep space mode
    (void)inclm;

    Real axnl = em * std::cos(argpm);
    Real temp_aynl = 1.0 / (am * (1.0 - em * em));
    Real aynl = em * std::sin(argpm) + temp_aynl * data_->aycof;
    Real xl = mm + argpm + nodem + temp_aynl * data_->xlcof * axnl;

    // Solve Kepler's equation
    Real u = fmod2p(xl - nodem);
    Real eo1 = u;
    Real tem5 = 1.0;
    int iter = 0;
    Real sineo1, coseo1;
    while (std::abs(tem5) > 1.0e-12 && iter < 10) {
        sineo1 = std::sin(eo1);
        coseo1 = std::cos(eo1);
        tem5 = 1.0 - coseo1 * axnl - sineo1 * aynl;
        tem5 = (u - aynl * coseo1 + axnl * sineo1 - eo1) / tem5;
        if (std::abs(tem5) > 0.95) {
            tem5 = (tem5 > 0.0) ? 0.95 : -0.95;
        }
        eo1 = eo1 + tem5;
        iter++;
    }

    // Short period preliminary quantities
    sineo1 = std::sin(eo1);
    coseo1 = std::cos(eo1);
    Real ecose = axnl * coseo1 + aynl * sineo1;
    Real esine = axnl * sineo1 - aynl * coseo1;
    Real el2 = axnl * axnl + aynl * aynl;
    Real pl = am * (1.0 - el2);

    if (pl < 0.0) {
        return false;
    }

    Real rl = am * (1.0 - ecose);
    Real rdotl = std::sqrt(am) * esine / rl;
    Real rvdotl = std::sqrt(pl) / rl;
    Real betal = std::sqrt(1.0 - el2);
    Real temp_sinu = am / rl * (sineo1 - aynl - axnl * esine / (1.0 + betal));
    Real sinu = temp_sinu;
    Real cosu = am / rl * (coseo1 - axnl + aynl * esine / (1.0 + betal));
    Real su = std::atan2(sinu, cosu);
    Real sin2u = (cosu + cosu) * sinu;
    Real cos2u = 1.0 - 2.0 * sinu * sinu;
    Real temp_mrt = 1.0 / pl;
    Real mrt = rl * (1.0 - 1.5 * CK2 * temp_mrt * (3.0 * data_->cosio2 - 1.0)) +
               0.5 * CK2 * temp_mrt * data_->x1mth2 * cos2u;
    Real su_dot = su;
    Real xnode = nodem + 1.5 * CK2 * temp_mrt * data_->cosio * sin2u;
    Real xinc = inclm + 1.5 * CK2 * temp_mrt * data_->cosio * data_->sinio * cos2u;
    Real mvt = rdotl - nm * temp_mrt * (data_->x1mth2 * sin2u) * CK2;
    Real rvdot = rvdotl + nm * temp_mrt * (data_->x1mth2 * cos2u + 1.5 * data_->con41) * CK2;

    // Orientation vectors
    Real sinsu = std::sin(su_dot);
    Real cossu = std::cos(su_dot);
    Real snod = std::sin(xnode);
    Real cnod = std::cos(xnode);
    Real sini = std::sin(xinc);
    Real cosi = std::cos(xinc);
    Real xmx = -snod * cosi;
    Real xmy = cnod * cosi;
    Real ux = xmx * sinsu + cnod * cossu;
    Real uy = xmy * sinsu + snod * cossu;
    Real uz = sini * sinsu;
    Real vx = xmx * cossu - cnod * sinsu;
    Real vy = xmy * cossu - snod * sinsu;
    Real vz = sini * cossu;

    // Position and velocity (earth radii and earth radii/min)
    Real r_er = mrt;
    Real rdot_er = mvt;
    Real rvdot_er = rvdot;

    pos_eci.x = r_er * ux * EARTH_RADIUS;
    pos_eci.y = r_er * uy * EARTH_RADIUS;
    pos_eci.z = r_er * uz * EARTH_RADIUS;

    vel_eci.x = (rdot_er * ux + rvdot_er * vx) * EARTH_RADIUS / 60.0;
    vel_eci.y = (rdot_er * uy + rvdot_er * vy) * EARTH_RADIUS / 60.0;
    vel_eci.z = (rdot_er * uz + rvdot_er * vz) * EARTH_RADIUS / 60.0;

    data_->position = pos_eci;
    data_->velocity = vel_eci;

    return true;
}

OrbitalElements SGP4Propagator::get_elements() const
{
    OrbitalElements elements;
    if (!initialized_) {
        return elements;
    }

    // Convert current state to orbital elements
    elements = OrbitalElements::from_state_vector(data_->position, data_->velocity);
    return elements;
}

// ============================================================================
// Orbital Elements Implementation
// ============================================================================

void OrbitalElements::to_state_vector(Vec3& pos_eci, Vec3& vel_eci) const
{
    constexpr Real MU = 398600.4418e9;  // m³/s²

    // Calculate specific angular momentum
    Real p = semi_major_axis * (1.0 - eccentricity * eccentricity);
    Real r = p / (1.0 + eccentricity * std::cos(true_anomaly));

    // Position in orbital frame
    Real r_pqw_p = r * std::cos(true_anomaly);
    Real r_pqw_q = r * std::sin(true_anomaly);

    // Velocity in orbital frame
    Real sqrt_mu_p = std::sqrt(MU / p);
    Real v_pqw_p = -sqrt_mu_p * std::sin(true_anomaly);
    Real v_pqw_q = sqrt_mu_p * (eccentricity + std::cos(true_anomaly));

    // Rotation matrices
    Real cos_raan = std::cos(raan);
    Real sin_raan = std::sin(raan);
    Real cos_inc = std::cos(inclination);
    Real sin_inc = std::sin(inclination);
    Real cos_argp = std::cos(arg_of_perigee);
    Real sin_argp = std::sin(arg_of_perigee);

    // Combined rotation PQW -> ECI
    Real r11 = cos_raan * cos_argp - sin_raan * sin_argp * cos_inc;
    Real r12 = -cos_raan * sin_argp - sin_raan * cos_argp * cos_inc;
    Real r21 = sin_raan * cos_argp + cos_raan * sin_argp * cos_inc;
    Real r22 = -sin_raan * sin_argp + cos_raan * cos_argp * cos_inc;
    Real r31 = sin_argp * sin_inc;
    Real r32 = cos_argp * sin_inc;

    // Transform to ECI
    pos_eci.x = r11 * r_pqw_p + r12 * r_pqw_q;
    pos_eci.y = r21 * r_pqw_p + r22 * r_pqw_q;
    pos_eci.z = r31 * r_pqw_p + r32 * r_pqw_q;

    vel_eci.x = r11 * v_pqw_p + r12 * v_pqw_q;
    vel_eci.y = r21 * v_pqw_p + r22 * v_pqw_q;
    vel_eci.z = r31 * v_pqw_p + r32 * v_pqw_q;
}

OrbitalElements OrbitalElements::from_state_vector(const Vec3& pos_eci, const Vec3& vel_eci)
{
    constexpr Real MU = 398600.4418e9;  // m³/s² (for km, use 398600.4418)

    OrbitalElements elements;

    // Position and velocity magnitudes
    Real r = std::sqrt(pos_eci.x * pos_eci.x + pos_eci.y * pos_eci.y + pos_eci.z * pos_eci.z);
    Real v = std::sqrt(vel_eci.x * vel_eci.x + vel_eci.y * vel_eci.y + vel_eci.z * vel_eci.z);

    // Specific angular momentum
    Real hx = pos_eci.y * vel_eci.z - pos_eci.z * vel_eci.y;
    Real hy = pos_eci.z * vel_eci.x - pos_eci.x * vel_eci.z;
    Real hz = pos_eci.x * vel_eci.y - pos_eci.y * vel_eci.x;
    Real h = std::sqrt(hx * hx + hy * hy + hz * hz);

    // Node vector (K x h)
    Real nx = -hy;
    Real ny = hx;
    Real n = std::sqrt(nx * nx + ny * ny);

    // Eccentricity vector
    Real rdotv = pos_eci.x * vel_eci.x + pos_eci.y * vel_eci.y + pos_eci.z * vel_eci.z;
    Real ex = (v * v - MU / r) * pos_eci.x / MU - rdotv * vel_eci.x / MU;
    Real ey = (v * v - MU / r) * pos_eci.y / MU - rdotv * vel_eci.y / MU;
    Real ez = (v * v - MU / r) * pos_eci.z / MU - rdotv * vel_eci.z / MU;
    elements.eccentricity = std::sqrt(ex * ex + ey * ey + ez * ez);

    // Semi-major axis
    Real energy = v * v / 2.0 - MU / r;
    elements.semi_major_axis = -MU / (2.0 * energy);

    // Inclination
    elements.inclination = std::acos(hz / h);

    // RAAN
    if (n > 1e-10) {
        elements.raan = std::acos(nx / n);
        if (ny < 0) {
            elements.raan = TWOPI - elements.raan;
        }
    }

    // Argument of perigee
    if (n > 1e-10 && elements.eccentricity > 1e-10) {
        Real ndote = nx * ex + ny * ey;
        elements.arg_of_perigee = std::acos(ndote / (n * elements.eccentricity));
        if (ez < 0) {
            elements.arg_of_perigee = TWOPI - elements.arg_of_perigee;
        }
    }

    // True anomaly
    if (elements.eccentricity > 1e-10) {
        Real edotr = ex * pos_eci.x + ey * pos_eci.y + ez * pos_eci.z;
        elements.true_anomaly = std::acos(edotr / (elements.eccentricity * r));
        if (rdotv < 0) {
            elements.true_anomaly = TWOPI - elements.true_anomaly;
        }
    }

    return elements;
}

Real OrbitalElements::period() const
{
    constexpr Real MU = 398600.4418e9;  // m³/s²
    return TWOPI * std::sqrt(semi_major_axis * semi_major_axis * semi_major_axis / MU);
}

Real OrbitalElements::altitude() const
{
    constexpr Real EARTH_RADIUS_M = 6378137.0;  // WGS-84
    Real p = semi_major_axis * (1.0 - eccentricity * eccentricity);
    Real r = p / (1.0 + eccentricity * std::cos(true_anomaly));
    return r - EARTH_RADIUS_M;
}

} // namespace jaguar::domain::space
