FROM ubuntu

# Base system update & toolchain install
RUN echo "[Docker] Stage: update & install toolchain" && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    gcc g++ clang libclang-rt-18-dev gdb\
    gcc-mingw-w64-x86-64 \
    g++-mingw-w64-x86-64 \
    gdb-mingw-w64 \
    wine64 wine-binfmt \
    cmake ninja-build make \
    git python3 python3-pip \
    clang-format clang-tidy && \
    pip install --no-cache-dir --break-system-packages conan pre-commit \
    clangd && \
    rm -rf /var/lib/apt/lists/*

ENV PATH="${PATH}:/root/.local/bin"

# Conan initialisation
RUN echo "[Docker] Conan profile detect" && \
    conan profile detect --force
# Pre-commit hook installation
RUN echo "[Docker] pre-commit install-hooks" \
    pre-commit install-hooks
WORKDIR "/sample"

# Default entrypoint
ENTRYPOINT [ "/bin/bash" ]