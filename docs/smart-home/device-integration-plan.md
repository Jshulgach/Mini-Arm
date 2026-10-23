# Device integration plan

Updated September 6, 2026 from owner-supplied inventory and current primary documentation. Actual local addresses and reported device details are in the Git-ignored `home-inventory.local.json`. This file describes candidate integrations; no devices are configured or controlled yet.

## Home Assistant rebuild

The previous Raspberry Pi 3B installation is unavailable with suspected SD-card corruption. Rebuild on this PC after resolving the [runtime choice](runtime-decision.md): HAOS VM is now recommended because HA's guide excludes Docker Desktop; native Docker Engine is the alternative. A backup, if one exists outside the card, may save configuration work; its availability is unknown and recovery is not a prerequisite to a fresh instance. Do not reformat the old card as part of this setup.

Persist HA data in the selected VM disk or container volume, make protected backups outside that disk, and verify restoration before retiring a working host. No additional Raspberry Pi purchase is necessary to begin. Keep the OctoPi Raspberry Pi as the printer controller.

## Verizon CR1000A

The owner supplied the router model, firmware and hardware revision. Firmware currency and actual settings have not been verified. Reserve addresses for the PC and OctoPi after confirming their current DHCP entries. A router device list is the next useful input to the network graph; associations and wiring remain unknown until reported or discovered.

The [CR1000A manual, IoT Network section](https://www.verizon.com/business/verizonpartnersolutions/resources/userguides/verizon-router-cr1000a-user-guide-business.pdf) describes a separate 2.4 GHz IoT SSID whose devices can communicate with the primary network without intervening firewall restrictions. This helps onboarding compatible appliances; it is not an isolated VLAN. The guest network restricts access to the primary network, so it is unsuitable for devices requiring direct access from this HA deployment without additional routing work. No SSID, firewall or router settings were changed.

## Snapmaker 2.0 through OctoPi

Reuse Home Assistant's built-in [OctoPrint integration](https://www.home-assistant.io/integrations/octoprint/). It offers job state, progress, temperatures, timing and a camera if OctoPrint provides one. Its onboarding uses the OctoPrint Application Keys flow: sign into the matching OctoPrint user to approve the integration request locally.

The supplied candidate address did not respond to a bounded HTTP HEAD request on port 80, including a retry outside the restricted sandbox. This does not prove that the Pi is off or the address is wrong. Check the Pi's power and the router's connected-device list, then verify its web interface before onboarding.

Start by observing an existing print and comparing values with OctoPrint. Confirm Snapmaker size variant, active toolhead and connection medium. Treat 3D printing, laser and CNC as different capabilities; do not expose a generic arbitrary G-code tool through the household assistant. There is no verification that the installed Snapmaker/OctoPrint combination exposes every integration feature.

## Allen + Roth shades

The owner identified two shades: living room item `2318932`, kitchen item `2318939`, both with remote FCC ID `2AVVDAC136-06`. Their [FCC test report, page 4](https://fcc.report/FCC-ID/2AVVDAC136-06/4744170.pdf) identifies A-OK model AC136-06 operating at 433.92 MHz with OOK modulation. This establishes a sub-GHz RF remote path rather than the earlier Zigbee hypothesis. The radio's message format and any code-counter behavior have not been decoded; frequency alone does not establish generic RF-blaster compatibility.

The [Lowe's shade manual](https://pdf.lowes.com/installationguides/193197786199_install.pdf) includes both item numbers. Lowe's lists item 2318932 as model 78620; the kitchen's separate retail model has not been verified. Both are now individual records with stable room assignments.

Recommended integration path: Home Assistant -> local Bond integration -> Bond Bridge/Pro -> 433.92 MHz shade receiver. [Bond's Allen + Roth support article](https://olibra.zendesk.com/hc/en-us/articles/4415111940507--Bond-Bridge-Pro-allen-roth-shade-help) documents open/close/stop for compatible shades, and [Home Assistant's Bond integration](https://www.home-assistant.io/integrations/bond/) supports local communication. A single centrally located bridge is the initial topology proposal, conditional on RF coverage to both rooms. Whether the owner already has a bridge is unknown.

The exact-FCC result of Bond's dynamic compatibility lookup was not visible through text retrieval, so compatibility on these installed units is still a commissioning check. Use the [vendor lookup](https://bondhome.io/supported-devices/?fcc=2AVVDAC136-06) or vendor support before purchase. Pair each shade separately and verify open, stop, close, independent addressing and range. Position reporting must be tested separately from successful command transmission; commands or elapsed travel time are not measured shade position. A Zigbee coordinator alone will not provide this 433.92 MHz remote path.

## GE washer and dryer

Preserve the full owner-supplied model strings: washer `GFW550SSNB1WW`, dryer `GFD55GSSN0WW`. Shorter retail family names are not proof of identical API support.

[GE's Wi-Fi setup instructions](https://products.geappliances.com/appliance/gea-support-search-content?contentId=23884) use the appliance network/password label during SmartHQ onboarding, then join the appliance to the home Wi-Fi network. Enter those credentials into the local setup app. Whether these units are already enrolled is unknown.

First candidate: [GE Appliances' SmartHQ custom integration](https://github.com/geappliances/geappliances-smarthq-integration). It uses the cloud developer API with OAuth and discovers entities from each appliance's services. Setup currently requires registering an application in the SmartHQ Developer Portal and entering its client credentials into HA before account authorization. It is a custom integration, not preinstalled by this Compose foundation. Exact available sensors and controls on these two units remain unverified.

Initial capability targets: cycle state, remaining time where available, availability and a transition into completed state. Compare against a real cycle before enabling completion notifications. Prevent a reconnect or stale state from generating duplicate completion events. Keep remote start disabled until its device-specific capability and physical prerequisites are understood.

## Integration order and open inputs

1. Bring up the fresh HA instance with persistent storage and a backup destination.
2. Continue L70 capability discovery as the user's preferred first control milestone. Its app status, address and attachment security remain unknown.
3. Locate OctoPi and add printer telemetry as an independent early visibility milestone.
4. Enroll/verify both appliances in SmartHQ, then discover their exposed services.
5. Confirm Bond compatibility/ownership and commission the two identified RF shades.
6. Add Ring models when available; proceed with Mini-Arm model and firmware repair on the bench.

Needed now: current OctoPi address/power status, Bond bridge ownership/compatibility check, SmartHQ enrollment status and the outstanding L70 details. Router serial number and WAN MAC are not needed for adapter selection. No account passwords or API keys are included in these files.
