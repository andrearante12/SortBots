// Gaussian-splat mode for the operator view's 3D view (webui/operator_cloud.js), a port
// of the old dashboard's splat_view.js (deleted with index.html/app.js, 2026-10-23) plus
// the panel that drives the splat worker. See docs/real-robot.md, "Gaussian splats".
//
// The renderer has no dependency on three.js beyond reading its camera: WebGL2 on a
// canvas of its own, stacked over the three.js canvas with pointer-events off, so
// OrbitControls keeps working underneath.
//
// Format: the .splat file splat/convert.py writes — 32 bytes per gaussian,
//   pos f32x3 | scale f32x3 (linear) | rgba u8x4 | quat u8x4 (w,x,y,z; q*128+128)
// already sorted most-visible-first by the trainer.
//
// Technique (and the shaders, nearly verbatim) from antimatter15/splat, MIT:
// each gaussian is packed into two RGBA32UI texels (centre as float bits; 3D
// covariance as six halfs + colour), a Web Worker depth-sorts on every camera
// move (16-bit counting sort, front to back), and one instanced quad per
// gaussian is blended front-to-back with ONE_MINUS_DST_ALPHA. Differences from
// upstream: the camera comes from three.js (map frame, z-up — converted to the
// OpenCV camera the shaders expect), the 2D covariance gets gsplat's 0.3 px
// low-pass so it renders like it trained, and rendering is on demand only —
// the page shares the GPU with Isaac Sim on the workstation.
//
// The splat is trained post-run by splat/worker.py on the WORKSTATION, never on the
// Jetson. This page may be served by the Jetson's serve.py, so the worker is another
// origin: its URL is ?splat_worker= or /api/splat/config, and every request is CORS.
//
// window.sortbotsSplat = { SplatView, WORKER_SRC, createPanel(opts), sourceFor(worker, location) }
"use strict";

(function () {
  const WORKER_SRC = `
  "use strict";
  let buffer = null, count = 0, f32 = null, u8 = null, lastView = null, sorting = false, pendingView = null;

  function packHalf2x16(x, y) {
    return (toHalf(x) | (toHalf(y) << 16)) >>> 0;
  }
  const _f = new Float32Array(1), _i = new Int32Array(_f.buffer);
  function toHalf(v) {
    _f[0] = v;
    const x = _i[0];
    let bits = (x >> 16) & 0x8000;
    let m = (x >> 12) & 0x07ff;
    const e = (x >> 23) & 0xff;
    if (e < 103) return bits;
    if (e > 142) { bits |= 0x7c00; bits |= (e == 255 ? 0 : 1) && x & 0x007fffff; return bits; }
    if (e < 113) { m |= 0x0800; bits |= (m >> (114 - e)) + ((m >> (113 - e)) & 1); return bits; }
    bits |= ((e - 112) << 10) | (m >> 1);
    bits += m & 1;
    return bits;
  }

  function buildTexture() {
    const texwidth = 1024 * 2;
    const texheight = Math.max(1, Math.ceil((2 * count) / texwidth));
    const texdata = new Uint32Array(texwidth * texheight * 4);
    const tc = new Uint8Array(texdata.buffer);
    const tf = new Float32Array(texdata.buffer);
    // Robust bounds: 2nd-98th percentile of a sample, so one floater 200 m out
    // doesn't make "reset view" frame empty space.
    const step = Math.max(1, Math.floor(count / 20000));
    const xs = [], ys = [], zs = [];
    for (let i = 0; i < count; i++) {
      tf[8 * i + 0] = f32[8 * i + 0];
      tf[8 * i + 1] = f32[8 * i + 1];
      tf[8 * i + 2] = f32[8 * i + 2];
      if (i % step === 0) { xs.push(f32[8 * i]); ys.push(f32[8 * i + 1]); zs.push(f32[8 * i + 2]); }
      tc[4 * (8 * i + 7) + 0] = u8[32 * i + 24 + 0];
      tc[4 * (8 * i + 7) + 1] = u8[32 * i + 24 + 1];
      tc[4 * (8 * i + 7) + 2] = u8[32 * i + 24 + 2];
      tc[4 * (8 * i + 7) + 3] = u8[32 * i + 24 + 3];
      const scale = [f32[8 * i + 3], f32[8 * i + 4], f32[8 * i + 5]];
      const rot = [
        (u8[32 * i + 28 + 0] - 128) / 128, (u8[32 * i + 28 + 1] - 128) / 128,
        (u8[32 * i + 28 + 2] - 128) / 128, (u8[32 * i + 28 + 3] - 128) / 128,
      ];
      const M = [
        1.0 - 2.0 * (rot[2] * rot[2] + rot[3] * rot[3]),
        2.0 * (rot[1] * rot[2] + rot[0] * rot[3]),
        2.0 * (rot[1] * rot[3] - rot[0] * rot[2]),
        2.0 * (rot[1] * rot[2] - rot[0] * rot[3]),
        1.0 - 2.0 * (rot[1] * rot[1] + rot[3] * rot[3]),
        2.0 * (rot[2] * rot[3] + rot[0] * rot[1]),
        2.0 * (rot[1] * rot[3] + rot[0] * rot[2]),
        2.0 * (rot[2] * rot[3] - rot[0] * rot[1]),
        1.0 - 2.0 * (rot[1] * rot[1] + rot[2] * rot[2]),
      ].map((k, j) => k * scale[Math.floor(j / 3)]);
      const s = [
        M[0] * M[0] + M[3] * M[3] + M[6] * M[6],
        M[0] * M[1] + M[3] * M[4] + M[6] * M[7],
        M[0] * M[2] + M[3] * M[5] + M[6] * M[8],
        M[1] * M[1] + M[4] * M[4] + M[7] * M[7],
        M[1] * M[2] + M[4] * M[5] + M[7] * M[8],
        M[2] * M[2] + M[5] * M[5] + M[8] * M[8],
      ];
      texdata[8 * i + 4] = packHalf2x16(4 * s[0], 4 * s[1]);
      texdata[8 * i + 5] = packHalf2x16(4 * s[2], 4 * s[3]);
      texdata[8 * i + 6] = packHalf2x16(4 * s[4], 4 * s[5]);
    }
    const pct = (a, p) => { a.sort((u, v) => u - v); return a[Math.min(a.length - 1, Math.floor(p * a.length))]; };
    const bounds = count ? {
      min: [pct(xs, 0.02), pct(ys, 0.02), pct(zs, 0.02)],
      max: [pct(xs, 0.98), pct(ys, 0.98), pct(zs, 0.98)],
    } : null;
    self.postMessage({ type: "texture", texdata, texwidth, texheight, count, bounds }, [texdata.buffer]);
  }

  function sort(view) {
    // Camera-space z of every centre (view is OpenCV w2c, column-major).
    let maxDepth = -Infinity, minDepth = Infinity;
    const sizeList = new Int32Array(count);
    const depths = new Float32Array(count);
    for (let i = 0; i < count; i++) {
      const d = view[2] * f32[8 * i] + view[6] * f32[8 * i + 1] + view[10] * f32[8 * i + 2];
      depths[i] = d;
      if (d > maxDepth) maxDepth = d;
      if (d < minDepth) minDepth = d;
    }
    const depthInv = (256 * 256 - 1) / ((maxDepth - minDepth) || 1);
    const counts = new Uint32Array(256 * 256);
    for (let i = 0; i < count; i++) {
      sizeList[i] = ((depths[i] - minDepth) * depthInv) | 0;
      counts[sizeList[i]]++;
    }
    const starts = new Uint32Array(256 * 256);
    for (let i = 1; i < 256 * 256; i++) starts[i] = starts[i - 1] + counts[i - 1];
    const depthIndex = new Uint32Array(count);
    for (let i = 0; i < count; i++) depthIndex[starts[sizeList[i]]++] = i;  // near first
    self.postMessage({ type: "order", depthIndex, count }, [depthIndex.buffer]);
  }

  function maybeSort() {
    if (!pendingView || !count) return;
    const view = pendingView;
    pendingView = null;
    if (lastView) {
      // Skip a re-sort when the viewing direction barely changed (upstream's
      // heuristic): order depends on direction far more than on translation.
      const dot = lastView[2] * view[2] + lastView[6] * view[6] + lastView[10] * view[10];
      const dt = Math.abs(lastView[14] - view[14]);
      if (Math.abs(dot - 1) < 0.0005 && dt < 0.05) return;
    }
    lastView = view;
    sort(view);
  }

  self.onmessage = (e) => {
    const m = e.data;
    if (m.type === "load") {
      buffer = m.buffer;
      count = Math.floor(buffer.byteLength / 32);
      f32 = new Float32Array(buffer, 0, count * 8);
      u8 = new Uint8Array(buffer, 0, count * 32);
      lastView = null;
      buildTexture();
    } else if (m.type === "view") {
      // Coalesce: views that queued up behind a running sort just overwrite
      // each other; one sort runs for the newest.
      pendingView = m.view;
      if (!sorting) {
        sorting = true;
        setTimeout(() => { sorting = false; maybeSort(); }, 0);
      }
    }
  };
  `;

  const VERT = `#version 300 es
  precision highp float;
  precision highp int;
  uniform highp usampler2D u_texture;
  uniform mat4 projection, view;
  uniform vec2 focal;
  uniform vec2 viewport;
  in vec2 position;
  in int index;
  out vec4 vColor;
  out vec2 vPosition;
  void main () {
    uvec4 cen = texelFetch(u_texture, ivec2((uint(index) & 0x3ffu) << 1, uint(index) >> 10), 0);
    vec4 cam = view * vec4(uintBitsToFloat(cen.xyz), 1);
    vec4 pos2d = projection * cam;
    float clip = 1.2 * pos2d.w;
    if (pos2d.z < -clip || pos2d.x < -clip || pos2d.x > clip || pos2d.y < -clip || pos2d.y > clip) {
      gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
      return;
    }
    uvec4 cov = texelFetch(u_texture, ivec2(((uint(index) & 0x3ffu) << 1) | 1u, uint(index) >> 10), 0);
    vec2 u1 = unpackHalf2x16(cov.x), u2 = unpackHalf2x16(cov.y), u3 = unpackHalf2x16(cov.z);
    mat3 Vrk = mat3(u1.x, u1.y, u2.x, u1.y, u2.y, u3.x, u2.x, u3.x, u3.y);
    mat3 J = mat3(
      focal.x / cam.z, 0., -(focal.x * cam.x) / (cam.z * cam.z),
      0., -focal.y / cam.z, (focal.y * cam.y) / (cam.z * cam.z),
      0., 0., 0.
    );
    mat3 T = transpose(mat3(view)) * J;
    mat3 cov2d = transpose(T) * Vrk * T;
    // gsplat rasterizes with eps2d = 0.3 px; without it thin gaussians that
    // looked right in training alias into sparkles here.
    cov2d[0][0] += 0.3;
    cov2d[1][1] += 0.3;
    float mid = (cov2d[0][0] + cov2d[1][1]) / 2.0;
    float radius = length(vec2((cov2d[0][0] - cov2d[1][1]) / 2.0, cov2d[0][1]));
    float lambda1 = mid + radius, lambda2 = mid - radius;
    if (lambda2 < 0.0) return;
    vec2 diagonalVector = normalize(vec2(cov2d[0][1], lambda1 - cov2d[0][0]));
    vec2 majorAxis = min(sqrt(2.0 * lambda1), 1024.0) * diagonalVector;
    vec2 minorAxis = min(sqrt(2.0 * lambda2), 1024.0) * vec2(diagonalVector.y, -diagonalVector.x);
    vColor = clamp(pos2d.z / pos2d.w + 1.0, 0.0, 1.0) *
      vec4((cov.w) & 0xffu, (cov.w >> 8) & 0xffu, (cov.w >> 16) & 0xffu, (cov.w >> 24) & 0xffu) / 255.0;
    vPosition = position;
    vec2 vCenter = vec2(pos2d) / pos2d.w;
    gl_Position = vec4(vCenter + position.x * majorAxis / viewport + position.y * minorAxis / viewport, 0.0, 1.0);
  }`;

  const FRAG = `#version 300 es
  precision highp float;
  in vec4 vColor;
  in vec2 vPosition;
  out vec4 fragColor;
  void main () {
    float A = -dot(vPosition, vPosition);
    if (A < -4.0) discard;
    float B = exp(A) * vColor.a;
    fragColor = vec4(B * vColor.rgb, B);
  }`;

  function compile(gl, type, src) {
    const s = gl.createShader(type);
    gl.shaderSource(s, src);
    gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
    return s;
  }

  class SplatView {
    // container: the element the three.js canvas lives in (#recon-canvas).
    // onChange: called when a new sort lands and the frame should be redrawn.
    constructor(container, onChange) {
      this.onChange = onChange || (() => {});
      this.canvas = document.createElement("canvas");
      this.canvas.className = "splat-canvas";
      Object.assign(this.canvas.style, {
        position: "absolute", inset: "0", pointerEvents: "none", display: "none",
        background: "#0a0c10",
      });
      container.appendChild(this.canvas);
      const gl = this.canvas.getContext("webgl2", { antialias: false, premultipliedAlpha: true });
      if (!gl) throw new Error("WebGL2 unavailable");
      this.gl = gl;
      const prog = gl.createProgram();
      gl.attachShader(prog, compile(gl, gl.VERTEX_SHADER, VERT));
      gl.attachShader(prog, compile(gl, gl.FRAGMENT_SHADER, FRAG));
      gl.linkProgram(prog);
      if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
      gl.useProgram(prog);
      this.prog = prog;
      this.u = {};
      for (const n of ["projection", "view", "focal", "viewport", "u_texture"]) {
        this.u[n] = gl.getUniformLocation(prog, n);
      }
      this.vao = gl.createVertexArray();
      gl.bindVertexArray(this.vao);
      const quad = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, quad);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-2, -2, 2, -2, 2, 2, -2, 2]), gl.STATIC_DRAW);
      const aPos = gl.getAttribLocation(prog, "position");
      gl.enableVertexAttribArray(aPos);
      gl.vertexAttribPointer(aPos, 2, gl.FLOAT, false, 0, 0);
      this.indexBuffer = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, this.indexBuffer);
      const aIdx = gl.getAttribLocation(prog, "index");
      gl.enableVertexAttribArray(aIdx);
      gl.vertexAttribIPointer(aIdx, 1, gl.INT, false, 0, 0);
      gl.vertexAttribDivisor(aIdx, 1);
      this.texture = gl.createTexture();
      gl.bindTexture(gl.TEXTURE_2D, this.texture);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
      gl.uniform1i(this.u.u_texture, 0);
      gl.disable(gl.DEPTH_TEST);
      gl.enable(gl.BLEND);
      gl.blendFuncSeparate(gl.ONE_MINUS_DST_ALPHA, gl.ONE, gl.ONE_MINUS_DST_ALPHA, gl.ONE);
      gl.blendEquationSeparate(gl.FUNC_ADD, gl.FUNC_ADD);

      this.count = 0;          // gaussians in the texture
      this.drawCount = 0;      // gaussians in the current sort order
      this.bounds = null;
      this.view = new Float32Array(16);
      const url = URL.createObjectURL(new Blob([WORKER_SRC], { type: "text/javascript" }));
      this.worker = new Worker(url);
      URL.revokeObjectURL(url);
      this.worker.onmessage = (e) => this._onWorker(e.data);
      this._loaded = null;
    }

    setVisible(on) {
      this.canvas.style.display = on ? "block" : "none";
    }

    // Fetch a .splat and upload it. onProgress(fraction) while downloading.
    async load(url, onProgress) {
      const res = await fetch(url);
      if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
      const total = Number(res.headers.get("Content-Length")) || 0;
      let buf;
      if (res.body && total) {
        buf = new Uint8Array(total);
        const reader = res.body.getReader();
        let got = 0;
        for (;;) {
          const { done, value } = await reader.read();
          if (done) break;
          buf.set(value, got);
          got += value.length;
          if (onProgress) onProgress(got / total);
        }
        buf = buf.subarray(0, got - (got % 32));
      } else {
        const ab = await res.arrayBuffer();
        buf = new Uint8Array(ab, 0, ab.byteLength - (ab.byteLength % 32));
      }
      const copy = buf.slice().buffer;   // own, transferable, 32-aligned length
      return new Promise((resolve) => {
        this._loaded = resolve;
        this.drawCount = 0;
        this.worker.postMessage({ type: "load", buffer: copy }, [copy]);
      });
    }

    _onWorker(m) {
      const gl = this.gl;
      if (m.type === "texture") {
        gl.bindTexture(gl.TEXTURE_2D, this.texture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA32UI, m.texwidth, m.texheight, 0,
          gl.RGBA_INTEGER, gl.UNSIGNED_INT, m.texdata);
        this.count = m.count;
        this.bounds = m.bounds;
        if (this._loaded) { this._loaded({ count: m.count, bounds: m.bounds }); this._loaded = null; }
        // Force a first sort for whatever the camera is doing right now.
        this.worker.postMessage({ type: "view", view: this.view.slice() });
      } else if (m.type === "order") {
        gl.bindBuffer(gl.ARRAY_BUFFER, this.indexBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, m.depthIndex, gl.DYNAMIC_DRAW);
        this.drawCount = m.count;
        this.onChange();
      }
    }

    // Draw with a three.js PerspectiveCamera whose world is the ROS map frame.
    render(camera) {
      const gl = this.gl;
      const dpr = window.devicePixelRatio || 1;
      const w = Math.max(1, Math.round(this.canvas.clientWidth * dpr));
      const h = Math.max(1, Math.round(this.canvas.clientHeight * dpr));
      if (this.canvas.width !== w || this.canvas.height !== h) {
        this.canvas.width = w;
        this.canvas.height = h;
      }
      gl.viewport(0, 0, w, h);
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT);
      if (!this.count) return;

      // three.js view (OpenGL camera: -z forward, y up) -> OpenCV camera
      // (z forward, y down), which is what both the shaders and the trainer
      // use: negate the camera's y and z rows.
      camera.updateMatrixWorld();
      const e = camera.matrixWorldInverse.elements;
      const v = this.view;
      for (let c = 0; c < 4; c++) {
        v[c * 4 + 0] = e[c * 4 + 0];
        v[c * 4 + 1] = -e[c * 4 + 1];
        v[c * 4 + 2] = -e[c * 4 + 2];
        v[c * 4 + 3] = e[c * 4 + 3];
      }
      const fy = (h / 2) / Math.tan((camera.fov * Math.PI) / 360);
      const fx = fy;
      const zn = camera.near, zf = camera.far;
      const proj = new Float32Array([
        (2 * fx) / w, 0, 0, 0,
        0, -(2 * fy) / h, 0, 0,
        0, 0, zf / (zf - zn), 1,
        0, 0, -(zf * zn) / (zf - zn), 0,
      ]);
      this.worker.postMessage({ type: "view", view: v.slice() });
      if (!this.drawCount) return;

      gl.useProgram(this.prog);
      gl.bindVertexArray(this.vao);
      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_2D, this.texture);
      gl.uniformMatrix4fv(this.u.projection, false, proj);
      gl.uniformMatrix4fv(this.u.view, false, v);
      gl.uniform2fv(this.u.focal, new Float32Array([fx, fy]));
      gl.uniform2fv(this.u.viewport, new Float32Array([w, h]));
      gl.drawArraysInstanced(gl.TRIANGLE_FAN, 0, 4, this.drawCount);
    }
  }

  // ------------------------------------------------------------------ the panel

  // "local" when the worker runs on the host that serves this page (sim: the
  // workstation does both), else this page's origin, which the worker pulls the
  // map's pose graph from (real: the Jetson's serve.py, GET /api/maps/<name>/db).
  // Pure, for the tests.
  function sourceFor(worker, loc) {
    const loopback = (h) => h === "localhost" || h === "127.0.0.1" || h === "::1" || h === "[::1]";
    const wh = new URL(worker).hostname;
    const ph = loc.hostname;
    return wh === ph || (loopback(wh) && loopback(ph)) ? "local" : loc.origin;
  }

  const TERMINAL = ["done", "failed", "cancelled"];
  const POLL_MS = 3000;
  const DOWN_MSG = "Splat worker not running. Start it on the workstation: scripts/splat.sh serve";

  // ui: {pick, mapSel, build, status, msg} elements; container: where the splat canvas goes.
  // hooks: requestRender(), frame(bounds) (camera fit), setInfo(text), softwareGL.
  function createPanel(container, ui, hooks) {
    const QS = new URLSearchParams(location.search);
    let view = null, viewError = null, worker = null, loadedUrl = null, timer = null;
    let active = false, reachable = null;
    const seen = new Map(); // job id -> state, to notice "done"

    // hidden follows emptiness, so an empty status or message takes no room whichever
    // way the markup starts out.
    const status = (text, title) => {
      ui.status.textContent = text || ""; ui.status.title = title || ""; ui.status.hidden = !text;
    };
    const message = (text) => { ui.msg.textContent = text || ""; ui.msg.hidden = !text; };

    async function workerUrl() {
      if (worker) return worker;
      let url = QS.get("splat_worker");
      if (!url) {
        try {
          const res = await fetch("/api/splat/config");
          if (res.ok) url = (await res.json()).worker_url;
        } catch (e) { /* older serve.py: no endpoint */ }
      }
      worker = url ? String(url).replace(/\/+$/, "") : null;
      return worker;
    }

    async function wfetch(path, opts) {
      const base = await workerUrl();
      if (!base) throw new Error("no splat worker configured (configs/splat.yaml worker_url)");
      const res = await fetch(base + path, opts);
      const body = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(body.error || `${res.status} ${res.statusText}`);
      return body;
    }

    function setReachable(ok, err) {
      reachable = ok;
      ui.pick.disabled = !ok;
      if (!ok) {
        ui.build.disabled = true;
        status("worker unreachable", String((err && err.message) || err || ""));
        if (!(view && view.count)) message(DOWN_MSG);
      }
    }

    async function refreshLists() {
      let splats;
      try {
        ({ splats } = await wfetch("/api/splats"));
      } catch (e) {
        setReachable(false, e);
        return false;
      }
      setReachable(true);
      const keep = ui.pick.value;
      ui.pick.textContent = "";
      for (const sp of splats) {
        const o = document.createElement("option");
        o.value = sp.url;
        const when = sp.finished ? new Date(sp.finished * 1000).toLocaleString() : "";
        const g = sp.gaussians >= 1e6 ? `${(sp.gaussians / 1e6).toFixed(2)}M`
          : `${Math.max(1, Math.round(sp.gaussians / 1e3))}k`;
        o.textContent = `${sp.map} · ${g}`;
        o.title = `${sp.job}${sp.psnr ? ` · ${sp.psnr} dB` : ""} · ${when} · from ${sp.source}`;
        ui.pick.appendChild(o);
      }
      if (!splats.length) {
        const o = document.createElement("option");
        o.value = ""; o.textContent = "(no splats yet)";
        ui.pick.appendChild(o);
      } else if (keep && [...ui.pick.options].some((o) => o.value === keep)) {
        ui.pick.value = keep;
      }
      // The maps a build can start from: this page's own library (/api/maps on the host
      // serving it, the Jetson on the robot), complete DBs only — a "pending" entry has
      // no pose graph yet, and serve.py refuses to stream one anyway.
      let maps = [];
      try {
        const body = await (await fetch("/api/maps")).json();
        maps = (body.maps || []).filter((m) => m.db_state === "complete");
      } catch (e) { /* no library: Build stays disabled */ }
      const keepMap = ui.mapSel.value;
      ui.mapSel.textContent = "";
      for (const m of maps) {
        const o = document.createElement("option");
        o.value = m.name; o.textContent = m.name; o.title = m.title || m.name;
        ui.mapSel.appendChild(o);
      }
      if (!maps.length) {
        const o = document.createElement("option");
        o.value = ""; o.textContent = "(no saved maps)";
        ui.mapSel.appendChild(o);
      } else if (keepMap && maps.some((m) => m.name === keepMap)) ui.mapSel.value = keepMap;
      ui.mapSel.disabled = !maps.length;
      ui.build.disabled = !maps.length;
      ui.build.title = maps.length ? "Train a splat from this saved map on the workstation's GPU (about 4 min)"
        : "No saved map to build from. Save one from the session menu first (Stop, keep the map)";
      if (!(view && view.count)) {
        message(splats.length ? "" : maps.length
          ? "No splats yet. Pick a saved map and press Build."
          : "No splats yet, and no saved map to build one from. Save a map first.");
      }
      return true;
    }

    async function loadPicked() {
      const rel = ui.pick.value;
      if (!rel || !view) return;
      const url = (await workerUrl()) + rel;
      if (url === loadedUrl) return;
      loadedUrl = url;
      status("loading…");
      message("");
      try {
        const { count, bounds } = await view.load(url, (f) => status(`loading ${Math.round(f * 100)}%`));
        hooks.setInfo(`splat · ${count.toLocaleString()} gaussians`);
        status("");
        if (bounds) hooks.frame(bounds);
        hooks.requestRender();
      } catch (e) {
        loadedUrl = null;
        status("load failed", String(e.message || e));
      }
    }

    async function poll() {
      let jobs;
      try {
        ({ jobs } = await wfetch("/api/jobs"));
      } catch (e) {
        setReachable(false, e);
        return;
      }
      if (reachable === false) { await refreshLists(); if (!active) return; loadPicked(); }
      const running = jobs.find((j) => !TERMINAL.includes(j.state));
      let finished = null;
      for (const j of jobs) {
        const prev = seen.get(j.id);
        if (prev && prev !== j.state && j.state === "done") finished = j;
        seen.set(j.id, j.state);
      }
      if (running) {
        const p = running.progress;
        const detail = p
          ? ` ${p.step}/${p.iters} · ${p.psnr} dB · ${Math.ceil((p.eta_s || 0) / 60)} min left` : "";
        status(`${running.map}: ${running.state.replace(/_/g, " ")}${detail}`,
          running.state === "waiting_for_gpu" ? "Isaac Sim is using the GPU; the job starts when the sim stops" : "");
      } else if (jobs[0] && jobs[0].finished > Date.now() / 1000 - 600) {
        // The newest job's outcome, for ten minutes: long enough to notice a failure
        // after walking away, short enough not to linger forever.
        const j = jobs[0];
        status(`${j.map}: ${j.state}`, j.error || "");
      }
      if (finished) {
        await refreshLists();
        const opt = [...ui.pick.options].find((o) => o.value.includes(`/${finished.id}.splat`));
        if (opt) { ui.pick.value = opt.value; loadPicked(); }
      }
    }

    async function build() {
      const map = ui.mapSel.value;
      if (!map) return;
      ui.build.disabled = true;
      try {
        const base = await workerUrl();
        const job = await wfetch("/api/jobs", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ map, source: sourceFor(base, location) }),
        });
        seen.set(job.id, job.state);
        status(`${map}: queued`);
      } catch (e) {
        status("build refused", String(e.message || e));
      } finally {
        ui.build.disabled = !reachable;
      }
    }

    ui.pick.addEventListener("change", loadPicked);
    ui.build.addEventListener("click", build);

    function show() {
      active = true;
      if (!view && !viewError) {
        try { view = new SplatView(container, () => hooks.requestRender()); } catch (e) { viewError = String(e.message || e); }
      }
      if (viewError) {
        hooks.setInfo("splat unavailable");
        message(`Splat mode needs WebGL2, which this browser isn't providing (${viewError}).`);
        return;
      }
      view.setVisible(true);
      hooks.setInfo(view.count ? `splat · ${view.count.toLocaleString()} gaussians` : "splat");
      if (view.count && view.bounds) hooks.frame(view.bounds);
      if (!timer) {
        refreshLists().then((ok) => { if (ok && active) loadPicked(); });
        poll();
        timer = setInterval(poll, POLL_MS);
      }
      hooks.requestRender();
    }

    function hide() {
      active = false;
      if (view) view.setVisible(false);
      if (timer) { clearInterval(timer); timer = null; }
      message("");
    }

    return {
      show, hide,
      render(camera) { if (view) view.render(camera); },
      get bounds() { return view && view.count ? view.bounds : null; },
      get count() { return view ? view.count : 0; },
    };
  }

  window.sortbotsSplat = { SplatView, WORKER_SRC, createPanel, sourceFor };
})();
