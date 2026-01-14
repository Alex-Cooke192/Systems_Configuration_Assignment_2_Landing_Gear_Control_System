FROM python:3.11-slim AS base
WORKDIR /app
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
COPY requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r /app/requirements.txt
COPY . /app

# --- test stage ---
FROM base AS test
# If you keep test deps separate:
COPY requirements-dev.txt /app/requirements-dev.txt
RUN pip install --no-cache-dir -r /app/requirements-dev.txt
CMD ["pytest", "-q"]

# --- runtime stage ---
FROM base AS runtime
CMD ["python", "-m", "cli"]
