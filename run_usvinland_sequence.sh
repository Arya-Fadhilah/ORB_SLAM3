#!/bin/bash
# Cara pakai:
#   ./run_usvinland_sequence.sh <nama_sequence> <path_folder_sequence> <path_timestamp> <mono|stereo>
#
# Contoh:
#   ./run_usvinland_sequence.sh seq02_lebar ~/USVinLand_seq02/Rectified ~/USVinLand_seq02/Timestamp.txt mono
#   ./run_usvinland_sequence.sh seq02_lebar ~/USVinLand_seq02/Rectified ~/USVinLand_seq02/Timestamp.txt stereo

set -e  # berhenti kalau ada command yang gagal, jangan lanjut diam-diam

SEQ_NAME=$1
SEQ_PATH=$2
TIMES_PATH=$3
MODE=$4

if [ -z "$SEQ_NAME" ] || [ -z "$SEQ_PATH" ] || [ -z "$TIMES_PATH" ] || [ -z "$MODE" ]; then
    echo "Usage: $0 <nama_sequence> <path_folder_sequence> <path_timestamp> <mono|stereo>"
    exit 1
fi

cd ~/asv_ws/src/ORB_SLAM3

RESULTS_DIR=~/asv_ws/usvinland_results/${SEQ_NAME}_${MODE}
mkdir -p "$RESULTS_DIR"

echo "=== Menjalankan sequence '$SEQ_NAME' mode '$MODE' ==="
echo "Hasil akan disimpan di: $RESULTS_DIR"

if [ "$MODE" == "mono" ]; then
    ./Examples/Monocular/mono_usvinland \
        Vocabulary/ORBvoc.txt \
        Examples/Monocular/usvinland.yaml \
        "$SEQ_PATH" \
        "$TIMES_PATH" || echo "WARNING: program exit dengan error (kemungkinan segfault saat shutdown GUI) -- lanjut coba pindahkan file yang sudah tersimpan"

    mv CameraTrajectory_mono.txt "$RESULTS_DIR/" 2>/dev/null || echo "WARNING: CameraTrajectory_mono.txt tidak ditemukan"
    mv KeyFrameTrajectory_mono.txt "$RESULTS_DIR/" 2>/dev/null || echo "WARNING: KeyFrameTrajectory_mono.txt tidak ditemukan"
    mv map*_PointCloud_mono.ply "$RESULTS_DIR/" 2>/dev/null || echo "WARNING: tidak ada file PointCloud_mono ditemukan"

elif [ "$MODE" == "stereo" ]; then
    ./Examples/Stereo/stereo_usvinland \
        Vocabulary/ORBvoc.txt \
        Examples/Stereo/usvinland.yaml \
        "$SEQ_PATH" \
        "$TIMES_PATH"

    mv CameraTrajectory.txt "$RESULTS_DIR/" 2>/dev/null || echo "WARNING: CameraTrajectory.txt tidak ditemukan"
    mv KeyFrameTrajectory.txt "$RESULTS_DIR/" 2>/dev/null || echo "WARNING: KeyFrameTrajectory.txt tidak ditemukan"
    mv map*_PointCloud.ply "$RESULTS_DIR/" 2>/dev/null || true

else
    echo "MODE harus 'mono' atau 'stereo', bukan '$MODE'"
    exit 1
fi

echo "=== Selesai. Cek hasil di $RESULTS_DIR ==="
ls -la "$RESULTS_DIR"
