"""Regenerate the compiled 2025/SV table from the bundled official IGRF-14 data."""
from pathlib import Path
import hashlib

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'assets/geomagnetic/igrf14coeffs.txt'
EXPECTED_SHA256 = '8f8d88403028fc4ee92c4f38d97b46e0a87e2cfc496045b43c9e26c1d6b0903c'


def generate():
    data = SOURCE.read_bytes()
    if hashlib.sha256(data).hexdigest() != EXPECTED_SHA256:
        raise ValueError('Official IGRF-14 coefficient checksum changed')
    rows = {}
    for line in data.decode().splitlines():
        fields = line.split()
        if len(fields) >= 30 and fields[0] in ('g', 'h'):
            rows[fields[0], int(fields[1]), int(fields[2])] = tuple(map(float, fields[-2:]))
    lines = ['// Generated from assets/geomagnetic/igrf14coeffs.txt; see THIRD_PARTY.md.',
             '#pragma once', '#include <array>', 'namespace detumble::detail {',
             'struct IgrfCoefficient { int degree; int order; double g; double h; double dg; double dh; };',
             'inline constexpr std::array<IgrfCoefficient, 104> igrf14_coefficients{{']
    for degree in range(1, 14):
        for order in range(degree + 1):
            g, dg = rows['g', degree, order]
            h, dh = rows.get(('h', degree, order), (0.0, 0.0))
            lines.append(f'    {{{degree}, {order}, {g}, {h}, {dg}, {dh}}},')
    lines.extend(['}};', '}  // namespace detumble::detail', ''])
    (ROOT / 'src/igrf14_coefficients.hpp').write_text('\n'.join(lines))


if __name__ == '__main__':
    generate()
