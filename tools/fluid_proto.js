'use strict';
// Reference port of the MSPM0 FLIP implementation (same Q8.8 integer math,
// same update order) used to sanity-check the physics before flashing.

const fs = require('fs');
const zlib = require('zlib');

const N = 32, GRAVITY = 24, VMAX = 1024;
const NP = parseInt(process.env.NP || '300', 10);

const FLIP_KEEP = parseInt(process.env.FLIP || '230', 10); /* Q8.8 */
const GX = parseInt(process.env.GX || '0', 10); /* sideways gravity, like tilt */
const ITERS = parseInt(process.env.ITERS || '40', 10);
const WARM = process.env.WARM !== '0';
const OMEGA = parseInt(process.env.OMEGA || '256', 10); /* Q8.8, 256 = plain GS */
const WATER = [0, 120, 255], BG = [0, 0, 0];

const clamp = (v, lo, hi) => (v < lo ? lo : (v > hi ? hi : v));
const min32 = (a, b) => (a < b ? a : b);
/* round-to-nearest, symmetric shift. Plain >> floors, which biases every
 * negative product downwards; applied per particle per frame that tiny bias
 * acts as a constant acceleration and the water slowly slides left. */
const rshr = (v, s) =>
    (v >= 0) ? ((v + (1 << (s - 1))) >> s) : -(((-v) + (1 << (s - 1))) >> s);

const fu = [], fv = [], fuw = [], fvw = [], fp = [], fd = [];
for (let i = 0; i <= N; i++) { fu.push(new Int32Array(N)); fuw.push(new Int32Array(N)); }
for (let i = 0; i < N; i++) { fv.push(new Int32Array(N + 1)); fvw.push(new Int32Array(N + 1)); }
for (let i = 0; i < N; i++) { fp.push(new Int32Array(N)); fd.push(new Int32Array(N)); }
const px = new Int32Array(NP), py = new Int32Array(NP);
const vx = new Int32Array(NP), vy = new Int32Array(NP);
const ox = new Int32Array(NP), oy = new Int32Array(NP);
const mask = new Uint8Array(N * N); /* 1 = cell holds at least one particle */
const MASKED = process.env.MASK !== '0';

const GRAV_MASKED = process.env.GMASK !== '0';

function isFluid(i, j) {
    return MASKED ? mask[i * N + j] !== 0 : true;
}

function hasParticle(i, j) {
    return GRAV_MASKED ? mask[i * N + j] !== 0 : true;
}

function sampleU(x, y) {
    let i = clamp(x >> 8, 0, N - 1);
    let j = clamp((y - 128) >> 8, 0, N - 1);
    const j1 = min32(j + 1, N - 1);
    const fx = x & 0xFF, fy = (y - 128) & 0xFF;
    const a = fu[i][j], b = fu[i + 1][j], c = fu[i][j1], d = fu[i + 1][j1];
    const top = a + rshr((b - a) * fx, 8);
    const bot = c + rshr((d - c) * fx, 8);
    return top + rshr((bot - top) * fy, 8);
}

function sampleV(x, y) {
    let i = clamp((x - 128) >> 8, 0, N - 1);
    const i1 = min32(i + 1, N - 1);
    let j = clamp(y >> 8, 0, N - 1);
    const fx = (x - 128) & 0xFF, fy = y & 0xFF;
    const a = fv[i][j], b = fv[i][j + 1], c = fv[i1][j], d = fv[i1][j + 1];
    const top = a + rshr((c - a) * fx, 8);
    const bot = b + rshr((d - b) * fx, 8);
    return top + rshr((bot - top) * fy, 8);
}

function clearGrids() {
    for (let i = 0; i <= N; i++) { fu[i].fill(0); fuw[i].fill(0); }
    for (let i = 0; i < N; i++) { fv[i].fill(0); fvw[i].fill(0); }
}

function p2g() {
    for (let p = 0; p < NP; p++) {
        const x = px[p], y = py[p], vxp = vx[p], vyp = vy[p];

        let i = clamp(x >> 8, 0, N - 1);
        let j = clamp((y - 128) >> 8, 0, N - 1);
        const j1 = min32(j + 1, N - 1);
        let fx = x & 0xFF, fy = (y - 128) & 0xFF;
        let w00 = ((256 - fx) * (256 - fy)) >> 8;
        let w10 = (fx * (256 - fy)) >> 8;
        let w01 = ((256 - fx) * fy) >> 8;
        let w11 = (fx * fy) >> 8;

        fu[i][j] = clamp(fu[i][j] + rshr(vxp * w00, 8), -30000, 30000);
        fu[i + 1][j] = clamp(fu[i + 1][j] + rshr(vxp * w10, 8), -30000, 30000);
        fu[i][j1] = clamp(fu[i][j1] + rshr(vxp * w01, 8), -30000, 30000);
        fu[i + 1][j1] = clamp(fu[i + 1][j1] + rshr(vxp * w11, 8), -30000, 30000);

        fuw[i][j] = clamp(fuw[i][j] + w00, 0, 30000);
        fuw[i + 1][j] = clamp(fuw[i + 1][j] + w10, 0, 30000);
        fuw[i][j1] = clamp(fuw[i][j1] + w01, 0, 30000);
        fuw[i + 1][j1] = clamp(fuw[i + 1][j1] + w11, 0, 30000);

        i = clamp((x - 128) >> 8, 0, N - 1);
        const i1 = min32(i + 1, N - 1);
        j = clamp(y >> 8, 0, N - 1);
        fx = (x - 128) & 0xFF; fy = y & 0xFF;
        w00 = ((256 - fx) * (256 - fy)) >> 8;
        w10 = (fx * (256 - fy)) >> 8;
        w01 = ((256 - fx) * fy) >> 8;
        w11 = (fx * fy) >> 8;

        fv[i][j] = clamp(fv[i][j] + rshr(vyp * w00, 8), -30000, 30000);
        fv[i1][j] = clamp(fv[i1][j] + rshr(vyp * w10, 8), -30000, 30000);
        fv[i][j + 1] = clamp(fv[i][j + 1] + rshr(vyp * w01, 8), -30000, 30000);
        fv[i1][j + 1] = clamp(fv[i1][j + 1] + rshr(vyp * w11, 8), -30000, 30000);

        fvw[i][j] = clamp(fvw[i][j] + w00, 0, 30000);
        fvw[i1][j] = clamp(fvw[i1][j] + w10, 0, 30000);
        fvw[i][j + 1] = clamp(fvw[i][j + 1] + w01, 0, 30000);
        fvw[i1][j + 1] = clamp(fvw[i1][j + 1] + w11, 0, 30000);
    }
}

function normalise() {
    for (let i = 0; i <= N; i++) {
        for (let j = 0; j < N; j++) {
            const w = fuw[i][j];
            fu[i][j] = w > 8 ? clamp(((fu[i][j] * 256) / w) | 0, -VMAX, VMAX) : 0;
        }
    }
    for (let i = 0; i < N; i++) {
        for (let j = 0; j <= N; j++) {
            const w = fvw[i][j];
            fv[i][j] = w > 8 ? clamp(((fv[i][j] * 256) / w) | 0, -VMAX, VMAX) : 0;
        }
    }
    for (let j = 0; j < N; j++) { fu[0][j] = 0; fu[N][j] = 0; }
    for (let i = 0; i < N; i++) { fv[i][0] = 0; fv[i][N] = 0; }
}

function step() {
    /* band optimisation: only the rows that hold particles (plus a margin)
     * take part in gravity, divergence and the pressure solve */
    let minRow = N - 1;
    mask.fill(0);
    for (let p = 0; p < NP; p++) {
        minRow = Math.min(minRow, py[p] >> 8);
        mask[clamp(px[p] >> 8, 0, N - 1) * N + clamp(py[p] >> 8, 0, N - 1)] = 1;
    }
    const topJ = clamp(minRow - 2, 0, N - 1);
    /* dilate the mask by one cell: the surface moves, and a cell that is air
     * now can hold particles next frame - if it flickers between fluid and
     * air the pressure solve injects energy */
    if (MASKED && process.env.DILATE !== '0') {
        const m2 = Uint8Array.from(mask);
        for (let i = 0; i < N; i++) {
            for (let j = 0; j < N; j++) {
                if (!mask[i * N + j]) continue;
                for (let dj = -1; dj <= 1; dj++) {
                    for (let di = -1; di <= 1; di++) {
                        const ii = i + di, jj = j + dj;
                        if (ii >= 0 && ii < N && jj >= 0 && jj < N)
                            m2[ii * N + jj] = 1;
                    }
                }
            }
        }
        mask.set(m2);
    }

    for (let p = 0; p < NP; p++) {
        ox[p] = sampleU(px[p], py[p]);
        oy[p] = sampleV(px[p], py[p]);
    }
    clearGrids();
    p2g();
    normalise();

    /* gravity on the grid, not on the particles: the projection can then
     * cancel it for resting fluid (hydrostatic pressure) */
    for (let i = 0; i < N; i++)
        for (let j = topJ + 1; j < N; j++)
            if (hasParticle(i, j) || hasParticle(i, j - 1))
                fv[i][j] = clamp(fv[i][j] + GRAVITY, -VMAX, VMAX);
    if (GX !== 0)
        for (let i = 1; i < N; i++)
            for (let j = topJ; j < N; j++)
                if (hasParticle(i, j) || hasParticle(i - 1, j))
                    fu[i][j] = clamp(fu[i][j] + GX, -VMAX, VMAX);

    for (let i = 0; i < N; i++)
        for (let j = topJ; j < N; j++)
            fd[i][j] = fu[i + 1][j] - fu[i][j] + fv[i][j + 1] - fv[i][j];

    if (!WARM) for (let i = 0; i < N; i++) fp[i].fill(0);
    for (let k = 0; k < ITERS; k++) {
        for (let i = 0; i < N; i++) {
            for (let j = topJ; j < N; j++) {
                if (!isFluid(i, j)) { fp[i][j] = 0; continue; } /* air */
                const pL = i > 0 ? fp[i - 1][j] : fp[i][j];
                const pR = i < N - 1 ? fp[i + 1][j] : fp[i][j];
                const pU = j > topJ ? fp[i][j - 1] : fp[i][j];
                const pD = j < N - 1 ? fp[i][j + 1] : fp[i][j];
                const gs = rshr(pL + pR + pU + pD - fd[i][j], 2);
                const np = fp[i][j] + rshr((gs - fp[i][j]) * OMEGA, 8);
                fp[i][j] = clamp(np, -16000, 16000);
            }
        }
    }
    for (let i = 1; i < N; i++)
        for (let j = topJ; j < N; j++)
            fu[i][j] = clamp(fu[i][j] - (fp[i][j] - fp[i - 1][j]), -VMAX, VMAX);
    for (let j = Math.max(1, topJ + 1); j < N; j++)
        for (let i = 0; i < N; i++)
            fv[i][j] = clamp(fv[i][j] - (fp[i][j] - fp[i][j - 1]), -VMAX, VMAX);

    for (let p = 0; p < NP; p++) {
        const vgx = sampleU(px[p], py[p]);
        const vgy = sampleV(px[p], py[p]);
        let nvx = clamp(vgx + rshr((vx[p] - ox[p]) * FLIP_KEEP, 8), -VMAX, VMAX);
        let nvy = clamp(vgy + rshr((vy[p] - oy[p]) * FLIP_KEEP, 8), -VMAX, VMAX);
        let nx = px[p] + nvx, ny = py[p] + nvy;
        if (nx < 128) { nx = 128; nvx = 0; }
        if (nx > N * 256 - 128) { nx = N * 256 - 128; nvx = 0; }
        if (ny < 128) { ny = 128; nvy = 0; }
        if (ny > N * 256 - 128) { ny = N * 256 - 128; nvy = 0; }
        px[p] = nx; py[p] = ny; vx[p] = nvx; vy[p] = nvy;
    }

    let maxDiv = 0, maxP = 0;
    for (let i = 0; i < N; i++) {
        for (let j = 0; j < N; j++) {
            maxDiv = Math.max(maxDiv, Math.abs(fu[i + 1][j] - fu[i][j] + fv[i][j + 1] - fv[i][j]));
            maxP = Math.max(maxP, Math.abs(fp[i][j]));
        }
    }
    let energy = 0;
    for (let p = 0; p < NP; p++) energy += Math.abs(vx[p]) + Math.abs(vy[p]);
    if (++stepCount % 20 === 0) {
        console.log(`  frame ${stepCount}: div=${maxDiv} p=${maxP} sum|v|=${energy}`);
    }
}

function init() {
    let n = 0;
    /* clean lattice: 30 columns x 10 rows, `per` particles per cell, so the
     * initial state has exactly uniform density */
    const per = Math.max(1, Math.round(NP / 300));
    for (let j = 0; j < 10 && n < NP; j++) {
        for (let i = 0; i < 30 && n < NP; i++) {
            for (let k = 0; k < per && n < NP; k++) {
                const jx = (k & 1) ? 64 : -64;
                const jy = (k >> 1) ? 64 : -64;
                px[n] = (i << 8) + 128 + jx;
                py[n] = ((N - 1 - j) << 8) + 128 + jy;
                vx[n] = 0; vy[n] = 0;
                n++;
            }
        }
    }
    for (let i = 0; i < N; i++) { fp[i].fill(0); fd[i].fill(0); }
    clearGrids();
}

function renderPNG(path, scale) {
    const W = N * scale, H = N * scale;
    const count = new Int32Array(N * N);
    for (let p = 0; p < NP; p++) {
        const i = clamp(px[p] >> 8, 0, N - 1);
        const j = clamp(py[p] >> 8, 0, N - 1);
        count[i * N + j]++;
    }
    const raw = Buffer.alloc((W * 3 + 1) * H);
    let o = 0;
    for (let y = 0; y < H; y++) {
        raw[o++] = 0;
        for (let x = 0; x < W; x++) {
            const i = (x / scale) | 0, j = (y / scale) | 0;
            const c = count[i * N + j] > 0 ? WATER : BG;
            raw[o++] = c[0]; raw[o++] = c[1]; raw[o++] = c[2];
        }
    }
    const chunk = (type, data) => {
        const len = Buffer.alloc(4); len.writeUInt32BE(data.length);
        const td = Buffer.concat([Buffer.from(type, 'ascii'), data]);
        const crc = Buffer.alloc(4); crc.writeUInt32BE(zlib.crc32(td) >>> 0);
        return Buffer.concat([len, td, crc]);
    };
    const ihdr = Buffer.alloc(13);
    ihdr.writeUInt32BE(W, 0); ihdr.writeUInt32BE(H, 4);
    ihdr[8] = 8; ihdr[9] = 2;
    const png = Buffer.concat([
        Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]),
        chunk('IHDR', ihdr),
        chunk('IDAT', zlib.deflateSync(raw)),
        chunk('IEND', Buffer.alloc(0)),
    ]);
    fs.writeFileSync(path, png);
    return count.reduce((a, b) => a + b, 0);
}

init();
let stepCount = 0;
const frames = (process.env.FRAME_LIST || '0,1,5,15,30,60,120')
    .split(',').map(Number);
const totalFrames = parseInt(process.env.FRAMES || '120', 10);
let maxAbsFu = 0, maxAbsFv = 0, maxAbsP = 0;
for (let f = 0; f <= totalFrames; f++) {
    if (frames.includes(f)) {
        const total = renderPNG(`frame_${String(f).padStart(3, '0')}.png`, 4);
        console.log(`frame ${f}: cells with water = ${total}`);
    }
    step();
    for (let i = 0; i <= N; i++) for (let j = 0; j < N; j++) maxAbsFu = Math.max(maxAbsFu, Math.abs(fu[i][j]));
    for (let i = 0; i < N; i++) for (let j = 0; j <= N; j++) maxAbsFv = Math.max(maxAbsFv, Math.abs(fv[i][j]));
}
let minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, maxV = 0;
for (let p = 0; p < NP; p++) {
    minX = Math.min(minX, px[p]); maxX = Math.max(maxX, px[p]);
    minY = Math.min(minY, py[p]); maxY = Math.max(maxY, py[p]);
    maxV = Math.max(maxV, Math.abs(vx[p]), Math.abs(vy[p]));
}
console.log(`particles x range: ${minX}..${maxX}  y range: ${minY}..${maxY}  (valid 128..${N * 256 - 128})`);
console.log(`max |v| = ${maxV} (limit ${VMAX}), max |u| = ${maxAbsFu}, max |v| grid = ${maxAbsFv}`);
