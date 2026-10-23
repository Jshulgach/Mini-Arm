# Home Assistant runtime decision

Status on September 6, 2026: superseded by the user's custom-dashboard direction. [Homebase](../../apps/homebase/README.md) now runs independently as a local frontend. Home Assistant is optional and no HA runtime selection is required to proceed. The alternatives below are historical proposals; no Home Assistant image was pulled, volume created or service started.

## Correction to the initial proposal

[Home Assistant's current Linux installation guide](https://www.home-assistant.io/installation/linux/) explicitly requires Docker Engine and excludes Docker Desktop. The earlier recommendation to deploy HA directly on Docker Desktop was incorrect. Compose syntax validation did not establish runtime support.

The PC has a Windows Home edition. WSL Ubuntu 24.04 uses systemd, but no native Docker daemon/package was found. Its `/usr/bin/docker` is a Desktop integration symlink. No VirtualBox installation was found at the standard path or in PATH, and no Hyper-V management command was found. A Windows feature query needed elevation, so installed Windows feature status was not established. Starting the installed Docker Desktop executable did not produce a reachable daemon during the check; do not treat this as a diagnosis of the Desktop installation.

## Recommended: Home Assistant OS VM plus separate AI containers

Use the [official Windows VM instructions](https://www.home-assistant.io/installation/windows/). Proposed initial allocation: 2 virtual CPUs, 4 GB RAM and the official virtual disk with at least 32 GB of usable disk space. Verify the image's requirements before import. VirtualBox is the candidate hypervisor on this Home-edition PC; installing it requires Windows administrator interaction and can install network drivers.

Use EFI and a bridged adapter per the official guide. Determine the VM's actual address and reserve it in the Verizon router after first boot. WSL and existing development repositories can remain in place. Containerize inference and the future custom coordinator independently; they communicate with HA through its API using configured addresses.

Acceptance: HA onboarding renders, the owner creates their account locally, the VM remains reachable after a restart, and a backup is saved outside the VM disk. Then connect the L70 and OctoPi through verified integration paths. Owner credentials must be entered locally, not stored in deployment notes.

Migration: use HA backups and a tested restore onto the future dedicated host. Move custom service Compose definitions and their persistent state separately. Do not run two active hubs/coordinators against the same actuators during migration.

## Alternative: native Docker Engine in a dedicated Linux/WSL environment

Preserves the HA Compose route, but container networking, Windows/LAN reachability, lifecycle and backups become our responsibility. Prefer a dedicated environment instead of replacing Docker Desktop's existing links in the user's robotics WSL installation. Provision Docker Engine using its [Ubuntu installation documentation](https://docs.docker.com/engine/install/ubuntu/). WSL is a development compromise whose device discovery and hardware access require validation; the generic Linux Docker instructions do not guarantee all WSL integrations.

The existing Compose files are a starting point for this route, not a deployment result. Container mode lacks HAOS's app management; companion services run independently.

## Next owner action after selection

For the recommended route: install the selected hypervisor if needed, then import and boot the official HAOS image. The agent can prepare/import/configure where permissions allow; any Windows administrator prompt must be completed locally. For the alternative: provision the isolated Linux runtime and validate networking before creating HA accounts or pairing devices.
