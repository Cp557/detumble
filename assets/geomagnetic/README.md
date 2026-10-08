# IGRF-14 source data

The unmodified official coefficient table was retrieved on 2026-10-05 from
https://www.ngdc.noaa.gov/IAGA/vmod/coeffs/igrf14coeffs.txt.

SHA-256: `8f8d88403028fc4ee92c4f38d97b46e0a87e2cfc496045b43c9e26c1d6b0903c`.

Run `python3 assets/scripts/generate_igrf14_coefficients.py` to regenerate the
compiled 2025 main-field and 2025–2030 secular-variation coefficients. Other
historical coefficients remain in the source table but are outside the simulator's
supported date range. The generated table makes runs independent of downloads and
working-directory asset paths.

The spherical-harmonic recurrence follows NOAA/BGS `igrf14syn` from
https://www.ngdc.noaa.gov/IAGA/vmod/igrf14.f. Its independent Fortran outputs are
stored as fixtures in the magnetic-field tests. See `THIRD_PARTY.md` for attribution.

For the public overview, see [how the field is calculated](../../docs/MAGNETIC_FIELD.md).
The [technical reference](../../docs/REFERENCE.md) documents frame and epoch
conventions. Reference-fixture tolerances describe numerical agreement with the
Fortran program, not accuracy relative to actual Earth measurements.
