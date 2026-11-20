#!/usr/bin/env bash
# Models for nodes/dynamic_segment.py: export, fetch, build engines, check, bench.
#
#   scripts/dynamic_seg_models.sh export    # any machine with uv: YOLO-World with
#                                           # configs/dynamic_segment.yaml's vocabulary
#                                           # baked in -> models/yolo_world.onnx
#   scripts/dynamic_seg_models.sh sam       # check NanoSAM's two ONNX files are in models/
#   scripts/dynamic_seg_models.sh engines   # Jetson: TensorRT FP16 .engine next to each .onnx
#   scripts/dynamic_seg_models.sh check     # Jetson: can THIS environment run TensorRT?
#   scripts/dynamic_seg_models.sh bench [--backend ort|trt]
#                                           # ms per stage + peak RSS, no ROS
#
# models/ is gitignored: weights are large, and an .engine only runs on the GPU
# and TensorRT version that built it, so each machine builds its own. The ONNX
# files are portable: export once on a workstation, copy models/*.onnx over.
#
# The vocabulary is baked in at export (YOLO-World set_classes), so editing
# `detector.classes` in configs/dynamic_segment.yaml means re-running export
# and engines. dynamic_segment.py refuses to start when `always`/`if_moving`
# name a class the export doesn't have.
#
# Licences: YOLO-World weights are GPL-3.0 (Tencent) / AGPL-3.0 (Ultralytics),
# accepted for this project 2026-11-20. NanoSAM is Apache-2.0.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODELS="$REPO_ROOT/models"
CONFIG="$REPO_ROOT/configs/dynamic_segment.yaml"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
# YOLO-World size: s is the Orin Nano default; m if bench shows headroom.
YOLO_WEIGHTS="${YOLO_WEIGHTS:-yolov8s-worldv2.pt}"

mkdir -p "$MODELS"

cmd_export() {
  command -v uv >/dev/null || { echo "ERROR: needs uv (https://docs.astral.sh/uv/)"; exit 1; }
  echo "[models] exporting $YOLO_WEIGHTS with the vocabulary from $CONFIG"
  echo "[models] (downloads the weights, CLIP and a CPU torch into a throwaway uv env)"
  # CPU torch: export needs no GPU, and the CUDA wheels are gigabytes.
  ( cd "$MODELS" && uv run --no-project \
      --index https://download.pytorch.org/whl/cpu --index-strategy unsafe-best-match \
      --with torch --with ultralytics --with onnx --with onnxslim --with pyyaml \
      --with "clip @ git+https://github.com/ultralytics/CLIP.git" \
      python - "$CONFIG" "$YOLO_WEIGHTS" <<'EOF'
import shutil, sys
from pathlib import Path
import yaml
from ultralytics import YOLOWorld
cfg = yaml.safe_load(Path(sys.argv[1]).read_text())
classes = cfg["detector"]["classes"]
m = YOLOWorld(sys.argv[2])
m.set_classes(classes)
out = Path(m.export(format="onnx", imgsz=int(cfg["detector"]["size"]), opset=17, simplify=True))
shutil.move(str(out), "yolo_world.onnx")
print(f"[models] models/yolo_world.onnx: {len(classes)} classes {classes}")
EOF
  )
}

cmd_sam() {
  local missing=0
  for f in nanosam_encoder.onnx nanosam_decoder.onnx; do
    if [[ -f "$MODELS/$f" ]]; then echo "[models] ok: models/$f"; else echo "[models] missing: models/$f"; missing=1; fi
  done
  if (( missing )); then
    cat <<'EOF'
[models] NanoSAM's ONNX files come from its README (github.com/NVIDIA-AI-IOT/nanosam,
         "Download the image encoder" / "mask decoder"): the ResNet18 image encoder
         and MobileSAM's mask decoder. Save them as
           models/nanosam_encoder.onnx   (resnet18_image_encoder.onnx)
           models/nanosam_decoder.onnx   (mobile_sam_mask_decoder.onnx)
         They are not fetched automatically: the links are Google Drive shares.
EOF
    exit 1
  fi
}

cmd_engines() {
  [[ -x "$TRTEXEC" ]] || { echo "ERROR: no trtexec at $TRTEXEC (set TRTEXEC=...); build engines where TensorRT is installed"; exit 1; }
  # Build with the SAME TensorRT that will run them: an engine from the host's
  # trtexec only deserializes against the host's libnvinfer (check tells you
  # whether the container sees that library).
  build() {  # onnx, extra trtexec args...
    local onnx="$1"; shift
    [[ -f "$MODELS/$onnx" ]] || { echo "[models] skip $onnx (missing)"; return; }
    echo "[models] $onnx -> ${onnx%.onnx}.engine"
    "$TRTEXEC" --onnx="$MODELS/$onnx" --saveEngine="$MODELS/${onnx%.onnx}.engine" --fp16 "$@" \
      >"$MODELS/${onnx%.onnx}.trtexec.log" 2>&1 \
      || { echo "[models] FAILED, see models/${onnx%.onnx}.trtexec.log"; return 1; }
  }
  build yolo_world.onnx
  build nanosam_encoder.onnx
  # The decoder's point count varies per prompt: box corners (2) plus an
  # optional blob centroid (3). Bounds leave room for a few more.
  build nanosam_decoder.onnx \
    --minShapes=point_coords:1x1x2,point_labels:1x1 \
    --optShapes=point_coords:1x3x2,point_labels:1x3 \
    --maxShapes=point_coords:1x10x2,point_labels:1x10
}

cmd_check() {
  # Phase 0.1 of the plan: which way TensorRT gets into the Jazzy container.
  echo "== python / ubuntu"
  python3 -c 'import sys, platform; print(sys.version.split()[0], platform.machine())'
  grep PRETTY_NAME /etc/os-release || true
  echo "== tensorrt python"
  python3 -c 'import tensorrt as t; print("tensorrt", t.__version__)' 2>&1 | tail -1
  echo "== cuda python"
  python3 -c 'from cuda import cudart; print("cuda-python ok")' 2>&1 | tail -1
  echo "== onnxruntime"
  python3 -c 'import onnxruntime as o; print(o.__version__, o.get_available_providers())' 2>&1 | tail -1
  echo "== libnvinfer / libcudart visible here (nvidia runtime CSV mounts or installed)"
  ldconfig -p 2>/dev/null | grep -E 'libnvinfer\.so|libcudart\.so' || echo "none in ldconfig"
  ls /usr/lib/aarch64-linux-gnu/libnvinfer* /usr/local/cuda*/lib64/libcudart* 2>/dev/null || true
  echo "== trtexec"
  ls -l "$TRTEXEC" 2>/dev/null || echo "no trtexec at $TRTEXEC"
  echo "== nvidia runtime mount lists (host only)"
  ls /etc/nvidia-container-runtime/host-files-for-container.d/ 2>/dev/null || echo "(not visible here)"
}

cmd_bench() {
  python3 "$REPO_ROOT/nodes/dynamic_segment.py" --bench \
    --platform "${SORTBOTS_PLATFORM:-sim}" "$@"
}

case "${1:-}" in
  export)  cmd_export;;
  sam)     cmd_sam;;
  engines) cmd_engines;;
  check)   cmd_check;;
  bench)   shift; cmd_bench "$@";;
  *) sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 2;;
esac
