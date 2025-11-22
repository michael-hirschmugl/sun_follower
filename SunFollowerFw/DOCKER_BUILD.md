# Docker Build Environment für SunFollowerFw

Dieses Dockerfile stellt eine portable Build-Umgebung für das SunFollowerFw-Projekt bereit.

## Voraussetzungen

- Docker installiert auf Ihrem System
- Optional: Docker Compose (für vereinfachtes Arbeiten)

## Enthaltene Tools

Das Docker-Image enthält folgende Build-Tools:

- **ARM GNU Toolchain** (arm-none-eabi-gcc) Version 13.2.Rel1
- **CMake** Version 3.27.7
- **Ninja** Build System
- Git und weitere Basis-Tools

## Schnellstart

### Option 1: Mit dem Build-Helper-Script (empfohlen)

Das `docker-build.sh` Script vereinfacht den Build-Prozess:

```bash
# Docker Image erstellen
./docker-build.sh image

# Firmware bauen (Debug-Modus)
./docker-build.sh build

# Firmware bauen (Release-Modus)
./docker-build.sh release

# Build-Artefakte löschen
./docker-build.sh clean

# Interaktive Shell im Container starten
./docker-build.sh shell
```

### Option 2: Mit Docker Compose

```bash
# Container starten und interaktive Shell öffnen
docker-compose run --rm build-env

# Im Container dann:
mkdir -p build && cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug ..
ninja
```

### Option 3: Direkt mit Docker-Kommandos

```bash
# Docker Image bauen
docker build -t sunfollower-build:latest .

# Container ausführen und bauen
docker run --rm -v $(pwd):/workspace -w /workspace sunfollower-build:latest bash -c "
    mkdir -p build && cd build
    cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug ..
    ninja
"

# Interaktive Shell
docker run --rm -it -v $(pwd):/workspace -w /workspace sunfollower-build:latest /bin/bash
```

## Build-Modi

- **Debug**: Kompiliert mit Debug-Informationen und ohne Optimierung (`-O0 -g3`)
- **Release**: Kompiliert mit Größenoptimierung (`-Os -g0`)

## Ausgabe

Nach erfolgreichem Build finden Sie die kompilierte Firmware:
- **Binary**: `build/SunFollowerFw.elf`
- **Map-Datei**: `build/SunFollowerFw.map`

## Toolchain-Versionen verifizieren

Im Container können Sie die installierten Versionen überprüfen:

```bash
arm-none-eabi-gcc --version
cmake --version
ninja --version
```

## Fehlerbehebung

### Docker Image neu bauen

Falls Probleme auftreten, bauen Sie das Image neu:

```bash
docker build --no-cache -t sunfollower-build:latest .
```

### Berechtigungsprobleme

Falls Build-Artefakte im Container erstellt wurden und Sie keine Schreibrechte haben:

```bash
# Als root im Container aufräumen
docker run --rm -v $(pwd):/workspace -w /workspace --user root sunfollower-build:latest \
    bash -c "rm -rf build && chown -R $(id -u):$(id -g) /workspace"
```

## Vorteile der Docker-Build-Umgebung

- ✅ Konsistente Build-Umgebung auf allen Systemen
- ✅ Keine Installation der Toolchain auf dem Host-System nötig
- ✅ Einfaches Teilen der Build-Umgebung im Team
- ✅ Reproduzierbare Builds
- ✅ Isolation von Systemabhängigkeiten

## Integration in CI/CD

Das Docker-Image kann einfach in CI/CD-Pipelines verwendet werden:

```yaml
# Beispiel für GitLab CI/CD
build:
  image: sunfollower-build:latest
  script:
    - mkdir -p build && cd build
    - cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
    - ninja
  artifacts:
    paths:
      - build/SunFollowerFw.elf
```

## Anpassungen

Die Dockerfile kann bei Bedarf angepasst werden, z.B.:
- Andere Toolchain-Version
- Zusätzliche Build-Tools
- Andere CMake-Version

Bearbeiten Sie dazu einfach das `Dockerfile` und bauen Sie das Image neu.
