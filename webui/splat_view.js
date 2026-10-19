// Gaussian-splat renderer for the 3D panel's "splat" mode (see app.js initRecon
// and docs/splat.md). No build step, no dependency on three.js beyond reading
// its camera: WebGL2 on a canvas of its own, stacked over the three.js canvas
// with pointer-events off so OrbitControls keeps working underneath.
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
// this panel shares the GPU with Isaac Sim (see app.js animate()).
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

  window.SplatView = SplatView;
})();
