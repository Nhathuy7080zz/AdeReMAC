# AdaReMAC V4: outstanding integration work

Date: 2026-10-19. This branch makes safety fixes to the existing camera sketches and portable paths for the legacy synthetic-data exporter. The new **real Edge Impulse Arduino inference sketch**, COCO-to-EI dataset converter, Vietnamese technical report and handover have been produced in a separate downloadable AdaReMAC V4 source ZIP and Word documents but are **not yet synchronized to this GitHub branch**.

## Confirmed limitations

- `ai_model_data.h` is NOT a full Edge Impulse MobileNetV2 FOMO model. The 12,000 examples in the exporter are synthetic feature vectors, not labeled images. The quoted 99.97% is not real-world object-detection accuracy.
- `rtl/adaremac_core.v` implements a signed INT8 MAC peripheral, not a fully accelerated YOLO/FOMO/SSD CNN. Two 16-bit SPI writes per MAC severely limit throughput.
- OV5640 register `0x4202` controls frame masking, not MIPI power. Do not force raw camera clock/pad registers or the regulator without device-specific verification. Standby bit `0x3008[6]` can be tested when the stream is idle.
- Streaming 30 FPS and 30 AI inferences per second are distinct measurements. Neither has been measured in this repository.

## Next steps

1. Copy `firmware_arduino/3_ei_fomo_stream`, `ai_eval/coco_to_edge_impulse.py` and both V4 Markdown documents from the generated ZIP to this branch.
2. Export a real INT8 Edge Impulse FOMO Arduino library (e.g. MobileNetV2 alpha 0.10, 96x96), adjust `ei_model_config.h` and compile/upload ESP32-S3 N16R8.
3. Import a licensed already-annotated COCO person dataset with `bounding_boxes.labels`, validate precision/recall on OV5640 imagery.
4. Synthesize Tang Nano 20K RTL with Gowin EDA; use MAC exact signed self-test. The FPGA is not a full object detector yet.
5. Collect real `stream_fps`, `ai_fps`, `decode_ms`, `infer_ms`, sensor surface temperature and power traces; update results before acceptance.
