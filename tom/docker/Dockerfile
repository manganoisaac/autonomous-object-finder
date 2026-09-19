# Headless Webots R2025a for offline Monte Carlo runs (Docker locally, Apptainer on HPC).
FROM cyberbotics/webots:R2025a-ubuntu22.04
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip libxcb-cursor0 bc && \
    pip3 install --no-cache-dir numpy==1.26.4 opencv-python-headless==4.10.0.84 scipy==1.13.1 && \
    rm -rf /var/lib/apt/lists/*
COPY vendor_assets.py /opt/vendor_assets.py
RUN python3 /opt/vendor_assets.py /usr/local/webots
# Built-in node defaults (e.g. RotationalMotor.sound) and vendored PROTOs point at
# raw.githubusercontent.com; redirect them to the local install so nothing is fetched.
RUN grep -rlZ "raw.githubusercontent.com/cyberbotics/webots/R2025a/" \
      /usr/local/webots/resources/nodes /usr/local/webots/projects --include=*.wrl --include=*.proto | \
    xargs -0 -r sed -i "s|https://raw.githubusercontent.com/cyberbotics/webots/R2025a/|webots://|g"
ENV LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
    MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1
