#!/bin/bash
BASEDIR=$(dirname "$0")
CONFDIR=/home/oliver/thesis/catkin_ws/deps/conf/ivector_extractor.fixed.conf
export GST_PLUGIN_PATH=/home/oliver/thesis/catkin_ws/deps/
echo "gst_plugin_path: $GST_PLUGIN_PATH"
echo "config_path: $CONFDIR"

gst-launch-1.0 --gst-debug="" -q alsasrc ! decodebin ! audioconvert ! audioresample ! kaldinnet2onlinedecoder \
  use-threaded-decoder=false \
  model=$GST_PLUGIN_PATH/final.mdl \
  fst=$GST_PLUGIN_PATH/HCLG.fst \
  word-syms=$GST_PLUGIN_PATH/words.txt \
  feature-type=mfcc \
  mfcc-config=$GST_PLUGIN_PATH/conf/mfcc.conf \
  ivector-extraction-config=$GST_PLUGIN_PATH/conf/ivector_extractor.fixed.conf \
  max-active=10000 \
  beam=10.0 \
  lattice-beam=8.0 \
  do-endpointing=true \
  endpoint-silence-phones="1:2:3:4:5:6:7:8:9:10" \
  chunk-length-in-secs=0.2 \
! filesink location=/dev/stdout buffer-mode=2

