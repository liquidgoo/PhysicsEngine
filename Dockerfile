FROM ubuntu
ARG PROJ_DIR="sample"
ARG CONAN_PROFILES="${PROJ_DIR}/conan_profiles/"

ENV PROJ_DIR="${PROJ_DIR}" \
    CONAN_PROFILES="${CONAN_PROFILES}"

WORKDIR ${PROJ_DIR}

RUN mkdir -p ~/.conan2/profiles && \
    mkdir -p ${CONAN_PROFILES} && \
    ln -sf ${CONAN_PROFILES} ~/.conan2/profiles/project_profiles
    #chown -R vscode:vscode /workspace

# Base system update & toolchain install
RUN echo "[Docker] Stage: update & install toolchain" && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    gcc g++ clang libclang-rt-18-dev gdb\
    gcc-mingw-w64-x86-64 \
    g++-mingw-w64-x86-64 \
    gdb-mingw-w64 \
    cmake ninja-build make \
    git python3 python3-pip \
    clang-format clang-tidy && \
    pip install --no-cache-dir --break-system-packages conan pre-commit \
    clangd && \
    rm -rf /var/lib/apt/lists/*

# Conan initialisation
RUN echo "[Docker] Conan profile detect" && \
    conan profile detect --force
# Pre-commit hook installation
RUN echo "[Docker] pre-commit install-hooks" \
    pre-commit install-hooks

# Default entrypoint
ENTRYPOINT [ "/bin/bash" ]