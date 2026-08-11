//! Low-precision solar-system ephemeris: apparent RA/Dec of the Sun, Moon, and naked-eye
//! planets for "Go To" pointing.
//!
//! Self-contained (no external crate, no calendar library — time enters as a Unix timestamp)
//! implementation of Paul Schlyter's *Computing planetary positions* method. Accuracy is
//! ~1–2 arcmin for the planets and ~2 arcmin for the Moon (with the topocentric correction),
//! which is far finer than any mount's pointing accuracy — the object lands well inside the
//! field. Results are apparent coordinates referred to the equator/equinox **of date**, which
//! is exactly what INDI's `EQUATORIAL_EOD_COORD` expects (no separate precession step).
//!
//! Reference: <https://www.stjarnhimlen.se/comp/ppcomp.html> (public domain).

use std::f64::consts::PI;

/// A solar-system body the user can slew to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolarObject {
    Sun,
    Moon,
    Mercury,
    Venus,
    Mars,
    Jupiter,
    Saturn,
    Uranus,
    Neptune,
}

impl SolarObject {
    /// All targets, in display order (Sun, Moon, then planets outward).
    pub fn all() -> &'static [SolarObject] {
        use SolarObject::*;
        &[
            Sun, Moon, Mercury, Venus, Mars, Jupiter, Saturn, Uranus, Neptune,
        ]
    }

    /// Human-friendly name.
    pub fn label(&self) -> &'static str {
        use SolarObject::*;
        match self {
            Sun => "Sun",
            Moon => "Moon",
            Mercury => "Mercury",
            Venus => "Venus",
            Mars => "Mars",
            Jupiter => "Jupiter",
            Saturn => "Saturn",
            Uranus => "Uranus",
            Neptune => "Neptune",
        }
    }
}

/// Apparent equatorial coordinates of date.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Equatorial {
    /// Right ascension, hours [0, 24).
    pub ra_hours: f64,
    /// Declination, degrees [-90, 90].
    pub dec_deg: f64,
}

// --- degree-based trig helpers (Schlyter works throughout in degrees) ---
fn sind(x: f64) -> f64 {
    x.to_radians().sin()
}
fn cosd(x: f64) -> f64 {
    x.to_radians().cos()
}
fn tand(x: f64) -> f64 {
    x.to_radians().tan()
}
fn asind(x: f64) -> f64 {
    x.clamp(-1.0, 1.0).asin().to_degrees()
}
fn atan2d(y: f64, x: f64) -> f64 {
    y.atan2(x).to_degrees()
}

/// Normalize an angle to [0, 360) degrees.
fn rev(x: f64) -> f64 {
    x - (x / 360.0).floor() * 360.0
}

/// Solve Kepler's equation `E = M + e·sin(E)` (degrees) by Newton's method, returning the
/// eccentric anomaly `E`. The first approximation is Schlyter's; the loop refines it (needed
/// for the Moon and Mercury, whose eccentricities are non-trivial).
fn kepler(m_deg: f64, e: f64) -> f64 {
    let edeg = e * (180.0 / PI);
    let m = rev(m_deg);
    let mut ea = m + edeg * sind(m) * (1.0 + e * cosd(m));
    for _ in 0..30 {
        let delta = (ea - edeg * sind(ea) - m) / (1.0 - e * cosd(ea));
        ea -= delta;
        if delta.abs() < 1e-7 {
            break;
        }
    }
    ea
}

/// Day number since the epoch 2000 Jan 0.0 TT (Schlyter's `d`), from a Unix timestamp.
/// Derived from JD(1970-01-01) = 2440587.5 and JD(2000 Jan 0.0) = 2451543.5.
fn day_number(unix_secs: f64) -> f64 {
    unix_secs / 86400.0 - 10956.0
}

/// Mean obliquity of the ecliptic (degrees) at day `d`.
fn obliquity(d: f64) -> f64 {
    23.4393 - 3.563e-7 * d
}

/// The Sun's geocentric state, reused for the planets' geocentric conversion and for sidereal
/// time.
struct Sun {
    /// True ecliptic longitude (degrees).
    lon: f64,
    /// Distance (AU).
    r: f64,
    /// Mean anomaly (degrees).
    m: f64,
    /// Mean longitude (degrees) = argument of perihelion + mean anomaly.
    l: f64,
}

fn sun(d: f64) -> Sun {
    let w = 282.9404 + 4.70935e-5 * d;
    let e = 0.016709 - 1.151e-9 * d;
    let m = rev(356.0470 + 0.9856002585 * d);
    let ea = kepler(m, e);
    let xv = cosd(ea) - e;
    let yv = (1.0 - e * e).sqrt() * sind(ea);
    let v = atan2d(yv, xv);
    let r = (xv * xv + yv * yv).sqrt();
    Sun {
        lon: rev(v + w),
        r,
        m,
        l: rev(w + m),
    }
}

/// Rotate geocentric ecliptic rectangular coordinates to equatorial and package as RA/Dec.
fn ecl_to_equatorial(xg: f64, yg: f64, zg: f64, ecl: f64) -> Equatorial {
    let xe = xg;
    let ye = yg * cosd(ecl) - zg * sind(ecl);
    let ze = yg * sind(ecl) + zg * cosd(ecl);
    Equatorial {
        ra_hours: rev(atan2d(ye, xe)) / 15.0,
        dec_deg: atan2d(ze, (xe * xe + ye * ye).sqrt()),
    }
}

/// Keplerian orbital elements at a given day `d` (angles in degrees, `a` in AU).
struct Elements {
    n: f64,
    i: f64,
    w: f64,
    a: f64,
    e: f64,
    m: f64,
}

/// Schlyter's orbital elements for a planet at day `d`.
fn planet_elements(obj: SolarObject, d: f64) -> Elements {
    use SolarObject::*;
    match obj {
        Mercury => Elements {
            n: 48.3313 + 3.24587e-5 * d,
            i: 7.0047 + 5.00e-8 * d,
            w: 29.1241 + 1.01444e-5 * d,
            a: 0.387098,
            e: 0.205635 + 5.59e-10 * d,
            m: 168.6562 + 4.0923344368 * d,
        },
        Venus => Elements {
            n: 76.6799 + 2.46590e-5 * d,
            i: 3.3946 + 2.75e-8 * d,
            w: 54.8910 + 1.38374e-5 * d,
            a: 0.723330,
            e: 0.006773 - 1.302e-9 * d,
            m: 48.0052 + 1.6021302244 * d,
        },
        Mars => Elements {
            n: 49.5574 + 2.11081e-5 * d,
            i: 1.8497 - 1.78e-8 * d,
            w: 286.5016 + 2.92961e-5 * d,
            a: 1.523688,
            e: 0.093405 + 2.516e-9 * d,
            m: 18.6021 + 0.5240207766 * d,
        },
        Jupiter => Elements {
            n: 100.4542 + 2.76854e-5 * d,
            i: 1.3030 - 1.557e-7 * d,
            w: 273.8777 + 1.64505e-5 * d,
            a: 5.20256,
            e: 0.048498 + 4.469e-9 * d,
            m: 19.8950 + 0.0830853001 * d,
        },
        Saturn => Elements {
            n: 113.6634 + 2.38980e-5 * d,
            i: 2.4886 - 1.081e-7 * d,
            w: 339.3939 + 2.97661e-5 * d,
            a: 9.55475,
            e: 0.055546 - 9.499e-9 * d,
            m: 316.9670 + 0.0334442282 * d,
        },
        Uranus => Elements {
            n: 74.0005 + 1.3978e-5 * d,
            i: 0.7733 + 1.9e-8 * d,
            w: 96.6612 + 3.0565e-5 * d,
            a: 19.18171 - 1.55e-8 * d,
            e: 0.047318 + 7.45e-9 * d,
            m: 142.5905 + 0.011725806 * d,
        },
        Neptune => Elements {
            n: 131.7806 + 3.0173e-5 * d,
            i: 1.7700 - 2.55e-7 * d,
            w: 272.8461 - 6.027e-6 * d,
            a: 30.05826 + 3.313e-8 * d,
            e: 0.008606 + 2.15e-9 * d,
            m: 260.2471 + 0.005995147 * d,
        },
        Sun | Moon => unreachable!("planet_elements called for Sun/Moon"),
    }
}

/// Heliocentric ecliptic longitude perturbations (degrees) for the giant planets. Schlyter's
/// largest terms; each argument is a linear combination of the giant planets' mean anomalies.
fn giant_perturbations(obj: SolarObject, d: f64) -> (f64, f64) {
    let mj = rev(19.8950 + 0.0830853001 * d); // Jupiter mean anomaly
    let ms = rev(316.9670 + 0.0334442282 * d); // Saturn mean anomaly
    let mu = rev(142.5905 + 0.011725806 * d); // Uranus mean anomaly
    match obj {
        SolarObject::Jupiter => {
            let dlon = -0.332 * sind(2.0 * mj - 5.0 * ms - 67.6)
                - 0.056 * sind(2.0 * mj - 2.0 * ms + 21.0)
                + 0.042 * sind(3.0 * mj - 5.0 * ms + 21.0)
                - 0.036 * sind(mj - 2.0 * ms)
                + 0.022 * cosd(mj - ms)
                + 0.023 * sind(2.0 * mj - 3.0 * ms + 52.0)
                - 0.016 * sind(mj - 5.0 * ms - 69.0);
            (dlon, 0.0)
        }
        SolarObject::Saturn => {
            let dlon = 0.812 * sind(2.0 * mj - 5.0 * ms - 67.6)
                - 0.229 * cosd(2.0 * mj - 4.0 * ms - 2.0)
                + 0.119 * sind(mj - 2.0 * ms - 3.0)
                + 0.046 * sind(2.0 * mj - 6.0 * ms - 69.0)
                + 0.014 * sind(mj - 3.0 * ms + 32.0);
            let dlat = -0.020 * cosd(2.0 * mj - 4.0 * ms - 2.0)
                + 0.018 * sind(2.0 * mj - 6.0 * ms - 49.0);
            (dlon, dlat)
        }
        SolarObject::Uranus => {
            let dlon = 0.040 * sind(ms - 2.0 * mu + 6.0)
                + 0.035 * sind(ms - 3.0 * mu + 33.0)
                - 0.015 * sind(mj - mu + 20.0);
            (dlon, 0.0)
        }
        _ => (0.0, 0.0),
    }
}

/// Geocentric apparent RA/Dec of a planet.
fn planet_position(obj: SolarObject, d: f64, sun: &Sun, ecl: f64) -> Equatorial {
    let el = planet_elements(obj, d);
    let ea = kepler(el.m, el.e);
    let xv = el.a * (cosd(ea) - el.e);
    let yv = el.a * (1.0 - el.e * el.e).sqrt() * sind(ea);
    let v = atan2d(yv, xv);
    let r = (xv * xv + yv * yv).sqrt();
    let vw = v + el.w;

    // Heliocentric ecliptic rectangular → ecliptic lon/lat.
    let xh = r * (cosd(el.n) * cosd(vw) - sind(el.n) * sind(vw) * cosd(el.i));
    let yh = r * (sind(el.n) * cosd(vw) + cosd(el.n) * sind(vw) * cosd(el.i));
    let zh = r * (sind(vw) * sind(el.i));
    let mut lonecl = atan2d(yh, xh);
    let mut latecl = atan2d(zh, (xh * xh + yh * yh).sqrt());

    let (dlon, dlat) = giant_perturbations(obj, d);
    lonecl += dlon;
    latecl += dlat;

    // Back to heliocentric rectangular, then add the Sun to get geocentric.
    let xh = r * cosd(lonecl) * cosd(latecl);
    let yh = r * sind(lonecl) * cosd(latecl);
    let zh = r * sind(latecl);
    let xg = xh + sun.r * cosd(sun.lon);
    let yg = yh + sun.r * sind(sun.lon);
    ecl_to_equatorial(xg, yg, zh, ecl)
}

/// Geocentric, then topocentric, apparent RA/Dec of the Moon.
fn moon_position(d: f64, sun: &Sun, ecl: f64, lat_deg: f64, lst_deg: f64) -> Equatorial {
    let n = 125.1228 - 0.0529538083 * d;
    let i = 5.1454;
    let w = 318.0634 + 0.1643573223 * d;
    let a = 60.2666; // Earth radii
    let e = 0.054900;
    let m = rev(115.3654 + 13.0649929509 * d);

    let ea = kepler(m, e);
    let xv = a * (cosd(ea) - e);
    let yv = a * (1.0 - e * e).sqrt() * sind(ea);
    let v = atan2d(yv, xv);
    let r = (xv * xv + yv * yv).sqrt();
    let vw = v + w;

    let xh = r * (cosd(n) * cosd(vw) - sind(n) * sind(vw) * cosd(i));
    let yh = r * (sind(n) * cosd(vw) + cosd(n) * sind(vw) * cosd(i));
    let zh = r * (sind(vw) * sind(i));
    let mut lonecl = atan2d(yh, xh);
    let mut latecl = atan2d(zh, (xh * xh + yh * yh).sqrt());
    let mut r = r;

    // Perturbations (Schlyter). Arguments from the Moon's and Sun's mean elements.
    let ms = sun.m; // Sun's mean anomaly
    let mm = m; // Moon's mean anomaly
    let lm = rev(n + w + m); // Moon's mean longitude
    let dd = lm - sun.l; // mean elongation
    let f = lm - n; // argument of latitude

    lonecl += -1.274 * sind(mm - 2.0 * dd)
        + 0.658 * sind(2.0 * dd)
        - 0.186 * sind(ms)
        - 0.059 * sind(2.0 * mm - 2.0 * dd)
        - 0.057 * sind(mm - 2.0 * dd + ms)
        + 0.053 * sind(mm + 2.0 * dd)
        + 0.046 * sind(2.0 * dd - ms)
        + 0.041 * sind(mm - ms)
        - 0.035 * sind(dd)
        - 0.031 * sind(mm + ms)
        - 0.015 * sind(2.0 * f - 2.0 * dd)
        + 0.011 * sind(mm - 4.0 * dd);
    latecl += -0.173 * sind(f - 2.0 * dd)
        - 0.055 * sind(mm - f - 2.0 * dd)
        - 0.046 * sind(mm + f - 2.0 * dd)
        + 0.033 * sind(f + 2.0 * dd)
        + 0.017 * sind(2.0 * mm + f);
    r += -0.58 * cosd(mm - 2.0 * dd) - 0.46 * cosd(2.0 * dd);

    // Geocentric ecliptic rectangular (Earth radii) → equatorial RA/Dec.
    let xg = r * cosd(lonecl) * cosd(latecl);
    let yg = r * sind(lonecl) * cosd(latecl);
    let zg = r * sind(latecl);
    let geo = ecl_to_equatorial(xg, yg, zg, ecl);

    // Topocentric correction: the Moon is close enough that parallax (up to ~1°, larger than
    // its disk) matters for pointing. Shift the geocentric RA/Dec to the observer's location.
    let ra = geo.ra_hours * 15.0;
    let dec = geo.dec_deg;
    let mpar = asind(1.0 / r); // horizontal parallax (degrees)
    let gclat = lat_deg - 0.1924 * sind(2.0 * lat_deg); // geocentric latitude
    let rho = 0.99833 + 0.00167 * cosd(2.0 * lat_deg);
    let ha = lst_deg - ra; // hour angle
    let g = atan2d(tand(gclat), cosd(ha));
    let top_ra = ra - mpar * rho * cosd(gclat) * sind(ha) / cosd(dec);
    let sin_g = sind(g);
    let top_dec = if sin_g.abs() > 1e-9 {
        dec - mpar * rho * sind(gclat) * sind(g - dec) / sin_g
    } else {
        // gclat ≈ 0 (near the equator, or unknown location): limit of the above.
        dec + mpar * rho * cosd(ha) * sind(dec)
    };
    Equatorial {
        ra_hours: rev(top_ra) / 15.0,
        dec_deg: top_dec,
    }
}

/// Apparent equatorial coordinates (of date) of `obj` at the given time and observer location.
///
/// `unix_secs` is the (fractional) Unix timestamp in UTC. `lat_deg` is the observer's geodetic
/// latitude (north positive) and `lon_deg_east` its longitude (east positive, INDI's
/// convention). The observer location only affects the Moon (topocentric parallax); the Sun and
/// planets are geocentric, so passing `0.0, 0.0` still yields correct coordinates for them.
pub fn position(obj: SolarObject, unix_secs: f64, lat_deg: f64, lon_deg_east: f64) -> Equatorial {
    let d = day_number(unix_secs);
    let ecl = obliquity(d);
    let s = sun(d);

    match obj {
        SolarObject::Sun => {
            let xs = s.r * cosd(s.lon);
            let ys = s.r * sind(s.lon);
            ecl_to_equatorial(xs, ys, 0.0, ecl)
        }
        SolarObject::Moon => {
            // Local sidereal time (degrees): GMST0 = Sun's mean longitude + 180°, plus the
            // Earth's rotation since 0h UT, plus the observer's longitude.
            let ut_hours = unix_secs.rem_euclid(86400.0) / 3600.0;
            let lst_deg = rev(s.l + 180.0 + ut_hours * 15.0 + lon_deg_east);
            moon_position(d, &s, ecl, lat_deg, lst_deg)
        }
        _ => planet_position(obj, d, &s, ecl),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Unix seconds for a UTC calendar date at 00:00, via the same epoch arithmetic the module
    /// uses, so the tests are independent of any date library.
    fn unix_utc(days_since_epoch: i64) -> f64 {
        days_since_epoch as f64 * 86400.0
    }

    // Days from 1970-01-01 to a few reference dates (computed by hand / cross-checked).
    const D_2000_01_01: i64 = 10957; // 2000-01-01 00:00 UT
    const D_1990_04_19: i64 = 7413; // 1990-04-19: Schlyter's worked-example date

    #[test]
    fn schlyter_day_number_matches_reference() {
        // Schlyter's example gives d = -3543 for 1990 April 19.0 UT.
        assert!((day_number(unix_utc(D_1990_04_19)) - (-3543.0)).abs() < 1e-6);
        assert!((day_number(unix_utc(D_2000_01_01)) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn sun_new_year_2000_is_low_in_the_south() {
        // On 1 Jan the Sun sits near ecliptic longitude 280° → RA ≈ 18h45m, Dec ≈ -23°.
        let c = position(SolarObject::Sun, unix_utc(D_2000_01_01), 0.0, 0.0);
        assert!(
            (18.5..19.0).contains(&c.ra_hours),
            "sun RA {} h out of range",
            c.ra_hours
        );
        assert!(
            (-23.5..-22.5).contains(&c.dec_deg),
            "sun Dec {}° out of range",
            c.dec_deg
        );
    }

    #[test]
    fn sun_declination_tracks_the_seasons() {
        // Northern summer solstice ≈ day 172 (2000-06-21); winter solstice ≈ day 355.
        let summer = position(SolarObject::Sun, unix_utc(D_2000_01_01 + 171), 0.0, 0.0);
        let winter = position(SolarObject::Sun, unix_utc(D_2000_01_01 + 354), 0.0, 0.0);
        assert!(summer.dec_deg > 23.0, "summer Dec {}°", summer.dec_deg);
        assert!(winter.dec_deg < -23.0, "winter Dec {}°", winter.dec_deg);
    }

    #[test]
    fn all_targets_yield_valid_coordinates() {
        let t = unix_utc(D_2000_01_01);
        for &obj in SolarObject::all() {
            let c = position(obj, t, 45.0, 5.0);
            assert!(
                (0.0..24.0).contains(&c.ra_hours),
                "{}: RA {} h",
                obj.label(),
                c.ra_hours
            );
            assert!(
                (-90.0..=90.0).contains(&c.dec_deg),
                "{}: Dec {}°",
                obj.label(),
                c.dec_deg
            );
        }
    }

    #[test]
    fn moon_topocentric_shift_is_bounded_and_nonzero() {
        // Geocentric (fallback) vs. an observer at 45°N should differ by a real but sub-degree
        // amount — the Moon's parallax.
        let t = unix_utc(D_2000_01_01);
        let geo = position(SolarObject::Moon, t, 0.0, 0.0);
        let topo = position(SolarObject::Moon, t, 45.0, 5.0);
        let dra = (geo.ra_hours - topo.ra_hours).abs() * 15.0; // degrees
        let ddec = (geo.dec_deg - topo.dec_deg).abs();
        let sep = (dra * dra + ddec * ddec).sqrt();
        assert!(sep > 0.05, "parallax shift {sep}° unexpectedly small");
        assert!(sep < 1.2, "parallax shift {sep}° unexpectedly large");
    }

    #[test]
    fn planets_lie_near_the_ecliptic_plane() {
        // Every planet stays within a few degrees of the ecliptic, so |Dec| can never approach
        // the pole — a coarse sanity bound on the geometry.
        let t = unix_utc(D_2000_01_01);
        for &obj in &[
            SolarObject::Mercury,
            SolarObject::Venus,
            SolarObject::Mars,
            SolarObject::Jupiter,
            SolarObject::Saturn,
        ] {
            let c = position(obj, t, 0.0, 0.0);
            assert!(c.dec_deg.abs() < 30.0, "{}: Dec {}°", obj.label(), c.dec_deg);
        }
    }
}
