# Observable home starting point

**Current direction (September 6, 2026): build our own dashboard from scratch.** [Homebase](../../apps/homebase/README.md) is the implemented local frontend. Home Assistant is an optional future adapter, not a prerequisite. Its older deployment proposal below is retained for reference; no HA installation is pending or underway. Homebase includes a house schematic, reported connection graph, device configuration and non-executing task previews. Live integrations remain to be built.

The current Windows PC is the initial host, with a later dedicated PC as the migration target. Read [architecture.md](architecture.md) for the researched design, L70 capability gates and Mini-Arm findings. [device-registry.example.json](device-registry.example.json) illustrates the future settings model. It is not consumed by this Compose setup.

**Runtime correction:** Home Assistant's current container guide explicitly excludes Docker Desktop. The Compose files below are a native Linux Docker Engine foundation, not a validated Windows Docker Desktop deployment. The recommended Windows hub is Home Assistant OS in a VM; AI and the future coordinator can still run in separate containers. See [runtime decision](runtime-decision.md). Selection is pending; no HA service has been deployed.

The previous Raspberry Pi 3B HA installation is unavailable with suspected SD-card corruption. The [device integration plan](device-integration-plan.md) covers the reported Verizon router, Snapmaker/OctoPi, Allen + Roth shades and GE laundry appliances. Owner-specific inventory is stored separately in the Git-ignored `home-inventory.local.json` and is not consumed by Compose.

## What exists

- A Docker Compose foundation for Home Assistant and optional Ollama, each with its own persistent named volume.
- An optional NVIDIA GPU override.
- A repeatable read-only Mini-Arm structural audit.
- A design and staged implementation plan. The custom coordinator, world model, camera pipeline, Eufy integration and ROS hardware bridge are **not implemented or installed** by this foundation.

Compose syntax has been checked locally. Images have not been pulled and containers have not been launched. A read-only HTTP check of the supplied OctoPi address timed out; no authenticated device/account integration has been tested. No actuator commands have been sent.

## Initial host setup

For Home Assistant OS on Windows, follow the VM route in [runtime decision](runtime-decision.md). The commands below apply only after a native Linux Docker Engine is provisioned for the alternative container route. The inspected WSL distribution has a Docker Desktop CLI symlink but no installed native daemon; it is not ready for these commands.

```bash
# Run on the selected Docker Engine host; preserve an existing .env.
[ -f .env ] || cp .env.example .env
docker compose config --quiet
docker compose up -d homeassistant
```

Open `http://localhost:8123` and complete local onboarding. The default port is bound to this PC's loopback interface. To use phones or other machines, set `HA_BIND_IP` to the host's reserved LAN IP and configure the appropriate Windows firewall scope. Do not put credentials in the example registry; use the integration's local setup UI.

This bridge-network setup targets manual IP/cloud integrations but is not runtime-tested and does not promise multicast discovery, Bluetooth or USB access. [Home Assistant's Linux container guide](https://www.home-assistant.io/installation/linux/) requires Docker Engine rather than Docker Desktop and uses host networking in its deployment example. Review host networking and hardware access on the selected Linux host. WSL introduces additional Windows/LAN networking considerations; using a native engine does not by itself validate full WSL device support. Container installation does not include the Home Assistant OS app/add-on management environment; companion services are separate Compose services.

For the L70, follow the capability-first sequence in the architecture document. The suggested Eufy component is a **custom** integration; it is not bundled by this Compose file. Review/pin its version and install it into HA separately before onboarding the device.

## Optional local inference

CPU-only service (model download and selection are separate):

```powershell
docker compose --profile local-ai up -d ollama
```

NVIDIA service:

```powershell
docker compose -f compose.yaml -f compose.gpu.yaml --profile local-ai config --quiet
docker compose -f compose.yaml -f compose.gpu.yaml --profile local-ai up -d ollama
```

[Docker's GPU documentation](https://docs.docker.com/desktop/features/gpu/) requires the Windows WSL2 backend and a compatible NVIDIA driver/kernel. The override follows [Compose GPU reservations](https://docs.docker.com/compose/how-tos/gpu-support/). GPU passthrough is not yet tested on this host. On a dedicated Linux NVIDIA host, provision its supported GPU container runtime separately. For a PC without an NVIDIA GPU, omit the override or use the API backend.

If HA and Ollama run in the same Compose network, the HA Ollama integration would use `http://ollama:11434`; from the container host, use `http://localhost:11434`. Never put `localhost:11434` into an HA container or VM expecting it to mean Windows. An HAOS VM needs a separately reachable, appropriately scoped inference endpoint; the current loopback-only Ollama port will not provide that connection. No model is automatically downloaded. Benchmark a small quantized model with short context before combining it with camera inference on the 8 GB GPU.

API inference uses HA's OpenAI integration and local secret entry. The future coordinator will implement a backend adapter of its own. Do not send interior images to an external model unless that camera's configuration explicitly allows it.

## Portability and restore

Images contain software; named volumes contain the actual home configuration, integration credentials and downloaded models. Copying only the Compose file does not migrate the home.

After a verified first deployment, record the exact image digests in `.env` and the integration versions in the inventory. Keep application code, versioned example configuration and deployment definitions in Git; keep secrets, live HA data and backups out of Git.

Migration procedure:

1. Make a Home Assistant backup and a protected copy of deployment settings; record source versions. Stop services for consistent volume-level copies where used.
2. Export the `observable-home_ha_config` volume using an appropriate Docker volume backup procedure. Ollama's model volume can be backed up or regenerated. Encrypt backups containing account tokens.
3. Provision the destination Docker/runtime and restore the volumes with their original permissions and names. Restore a backup compatible with the target HA installation method; test that path before retiring the source.
4. Reconfigure LAN IP/firewall/discovery and GPU/USB access. Use stable hostnames and reserved device IPs.
5. Stop the source deployment before bringing the destination online. Verify state and authentication, then test one supervised action. Avoid two active coordinators controlling the same equipment.

The future coordinator's database, media retention and robot calibration artifacts will each need persistent volumes and restore checks as they are implemented. Never use `docker compose down -v` for ordinary shutdown; it removes named volumes. `docker compose stop` retains them.

## Model audit

From the Mini-Arm repository root:

```powershell
python docs/smart-home/audit_model.py
```

This checks static URDF/SRDF/control references and prints findings. It does not import device drivers, open serial ports, validate kinematics against hardware or certify a ROS build.
