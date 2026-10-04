# Final report

Final report for the 3006ICT group project, in the ICCV 2025 template (camera-ready).
`main.pdf` is the version to submit. Upload it to Canvas on its own, not inside the code ZIP.

## Build

```bash
tectonic -X compile main.tex      # or upload this folder to Overleaf
```

TeX Live's `pdflatex` also works if `silence.sty` is installed. `main.tex` loads `fontenc` T1 so that
XeTeX-based engines (tectonic) use the template's Times font.

## Layout

| Path | Content |
| --- | --- |
| `main.tex`, `preamble.tex` | document, author block, macros |
| `sec/0_abstract.tex` … `sec/8_contribution.tex` | abstract, sections 1–8, contribution statement |
| `sec/9_appendix.tex` | A planner comparison, B full encoder comparison, C development history |
| `main.bib` | references |
| `figs/` | every figure used in the report (vector PDF) |
| `iccv.sty`, `ieeenat_fullname.bst` | ICCV 2025 author kit |

## Figures

| Figure | File | Source |
| --- | --- | --- |
| 1 | `system_overview.drawio.pdf` | draw.io diagram `system_overview.drawio`, exported with headless draw.io |
| 2 | `perception_samples.pdf` | reference images, view-matched augmentation, captured camera frames |
| 3, 4 | `p_confusion.pdf`, `p_tsne.pdf` | hard-set predictions; frozen-encoder features on the normal set |
| 5, 6 | `p_crop_row.pdf`, `p_multiview_final.pdf` | yaw-sweep scores of the prototype and fine-tuned DINOv3-S |
| 7 | `p_qual.pdf` | camera frames scored by the submitted and the first detector |
| 8 | `route_planning.pdf` | controller's station order and A* planner, replayed offline |
| 9 | `trajectories.pdf` | referee-logged trajectories of the three student missions |
| 10 | `mc_versions.pdf`, `mc_ablations.pdf` | referee results of the benchmark runs |
| 11 | `p_headline.pdf` | encoder-study metrics |
| 12 | `git_graph.pdf` | commit graph of this repository |

All plots share one style (Liberation Serif, a Times clone, with STIX maths) so that they match
the body text.
