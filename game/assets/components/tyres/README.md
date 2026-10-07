# Garage tyre component images

Transparent product renders generated for the garage upgrade cards. `catalog.json` explicitly maps selectable definition IDs to image, label and size; it also lists reserved drag/slick styles without adding physical tyre definitions or selectable parts.

- road.png: drainage channels and diagonal tread blocks.
- track.png: semi-slick with broad contact patches and sparse grooves.
- drag.png: tall rounded sidewall, almost smooth tread and two straight grooves.
- slick.png: completely smooth contact surface.

The current tyre files specify dimensions and force coefficients, not measured tread geometry. These images are authored visual representations of the fictional parts, not scans or evidence of an exact real tyre. Front/rear variants share the style thumbnail; their actual dimensions remain in the captions and physics definitions. Future definitions should receive explicit presentation entries, rather than choosing images from their friction coefficient. Individual tread geometry and tyre customization can later supply generated thumbnails from the actual component mesh.

All captions and state labels are native UI, not baked into the artwork. Runtime PNG loading and per-screen texture caching avoid reliance on editor import side effects. Images do not intercept pointer input; the whole card remains the installation button.