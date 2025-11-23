# Dockerfile - reproducible Linux environment for Flyt deconflict demo
FROM python:3.11-slim

# create non-root user
RUN useradd --create-home appuser
WORKDIR /home/appuser/workspace

USER root
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg \
    && rm -rf /var/lib/apt/lists/*

USER appuser

# copy requirements first for caching
COPY --chown=appuser:appuser requirements.txt .

RUN python -m pip install --upgrade pip setuptools wheel \
 && pip install --no-cache-dir -r requirements.txt

# copy project files
COPY --chown=appuser:appuser src/ ./src/
COPY --chown=appuser:appuser data/ ./data/
COPY --chown=appuser:appuser README.md ./

# default to bash for interactive work
CMD [ "bash" ]
