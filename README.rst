.. _matter_template_sample:

Matter: Occupancy Sensor
########################

.. contents::
   :local:
   :depth: 2

This sample demonstrates a Matter Occupancy Sensor implementation that uses :ref:`Bluetooth Channel Sounding <ug_ble_channel_sounding>` for RF-based occupancy detection.
The sample implements the Matter OccupancySensing cluster and can detect occupancy using Radio Frequency (RF) Sensing through Channel Sounding distance measurements.
Optionally, the sample can also support a Passive Infrared (PIR) sensor for traditional motion detection.

The sample enables commissioning on the device, which allows it to join a Matter network built on top of a low-power, 802.15.4 Thread network.
This device works as a Thread :ref:`Minimal End Device <thread_ot_device_types>`.

The RF Sensing functionality uses the Channel Sounding RAS (Ranging and Sensing) Initiator to measure distances to paired reflector devices.
Each paired reflector device gets its own Matter endpoint for independent occupancy detection.
When a reflector device is within the configured occupancy threshold distance, the corresponding endpoint is marked as occupied.

Requirements
************

The sample supports the following development kits:

.. table-from-sample-yaml::

For testing purposes, that is to commission the device and :ref:`control it remotely <matter_template_network_mode>` through a Thread network, you also need a Matter controller device :ref:`configured on PC or smartphone <ug_matter_configuring>`. This requires additional hardware depending on the setup you choose.

.. note::
    |matter_gn_required_note|

IPv6 network support
====================

This sample supports Matter over Thread for the ``nrf54l15dk/nrf54l15/cpuapp`` board target.

Overview
********

The sample starts the Bluetooth® LE advertising automatically and prepares the Matter device for commissioning into a Matter-enabled Thread network.
The sample uses LEDs to show the state of the connection and RF Sensing mode.
You can press a button to start the factory reset when needed.

RF Sensing Occupancy Detection
==============================

The sample implements RF Sensing occupancy detection using Bluetooth Channel Sounding.
It acts as a Channel Sounding RAS Initiator and connects to paired reflector devices to measure distances.
Each paired reflector device is assigned a unique Matter endpoint (starting from endpoint 1).
When the measured distance to a reflector is below the configured occupancy threshold, that endpoint is marked as occupied.

The sample supports:
* Multiple paired reflector devices (configurable via ``CONFIG_BT_MAX_PAIRED``, default: 2)
* Preemptive mode for Channel Sounding, which prioritizes reconnection to last known devices
* Configurable occupancy threshold distance
* Automatic occupancy timeout handling

PIR Sensor Support (Optional)
==============================

The sample optionally supports a PIR (Passive Infrared) sensor for traditional motion-based occupancy detection.
PIR support can be enabled by setting ``CONFIG_PIR_SUPPORT=y`` in the project configuration.
When enabled, the PIR sensor uses a dedicated Matter endpoint (endpoint 1 + CONFIG_BT_MAX_PAIRED).
The PIR sensor can be simulated using a button for testing purposes when ``CONFIG_SIMULATED_PIR_SENSOR=y`` is set.

.. _matter_template_network_mode:

Remote testing in a network
===========================

Testing in a Matter-enabled Thread network requires a Matter controller that you can configure on PC or mobile device.
By default, the Matter accessory device has IPv6 networking disabled.
You must pair the device with the Matter controller over Bluetooth LE to get the configuration from the controller to use the device within a Thread network.
You can enable the controller after :ref:`building and running the sample <matter_template_network_testing>`.

To pair the device, the controller must get the :ref:`matter_template_network_mode_onboarding` from the Matter accessory device and commission the device into the network.

Commissioning in Matter
-----------------------

In Matter, the commissioning procedure takes place over Bluetooth LE between a Matter accessory device and the Matter controller, where the controller has the commissioner role.
When the procedure has completed, the device is equipped with all information needed to securely operate in the Matter network.

During the last part of the commissioning procedure (the provisioning operation), the Matter controller sends the Thread network credentials to the Matter accessory device.
As a result, the device can join the IPv6 network and communicate with other devices in the network.

.. _matter_template_network_mode_onboarding:

Onboarding information
++++++++++++++++++++++

When you start the commissioning procedure, the controller must get the onboarding information from the Matter accessory device.
The onboarding information representation depends on your commissioner setup.

For this sample, you can use one of the following :ref:`onboarding information formats <ug_matter_network_topologies_commissioning_onboarding_formats>` to provide the commissioner with the data payload that includes the device discriminator and the setup PIN code:

  .. list-table:: Template sample onboarding information
     :header-rows: 1

     * - QR Code
       - QR Code Payload
       - Manual pairing code
     * - Scan the following QR code with the app for your ecosystem:

         .. figure:: ../../../doc/nrf/images/matter_qr_code_template_sample.png
            :width: 200px
            :alt: QR code for commissioning the template device

       - MT:Y.K9042C00KA0648G00
       - 34970112332

.. include:: ../lock/README.rst
    :start-after: matter_door_lock_sample_onboarding_start
    :end-before: matter_door_lock_sample_onboarding_end

|matter_cd_info_note_for_samples|

Configuration
*************

|config|

.. _matter_template_custom_configs:

Matter occupancy sensor custom configurations
==============================================

.. include:: ../light_bulb/README.rst
    :start-after: matter_light_bulb_sample_configuration_file_types_start
    :end-before: matter_light_bulb_sample_configuration_file_types_end

RF Sensing Configuration
-------------------------

The RF Sensing occupancy detection can be configured using the following Kconfig options:

* ``CONFIG_BT_MAX_PAIRED`` - Maximum number of paired reflector devices (default: 2)
  Each paired device gets its own Matter endpoint for occupancy detection.
* ``CONFIG_RFS_OCCUPANCY_THRESHOLD`` - Distance threshold in meters for occupancy detection
  When a reflector is within this distance, the endpoint is marked as occupied.

PIR Sensor Configuration
------------------------

To enable PIR sensor support, add the following to your project configuration:

.. code-block:: console

    CONFIG_PIR_SUPPORT=y

For testing without a physical PIR sensor, you can enable simulation mode:

.. code-block:: console

    CONFIG_PIR_SUPPORT=y
    CONFIG_SIMULATED_PIR_SENSOR=y

Matter occupancy sensor with Trusted Firmware-M
===============================================

.. matter_template_build_with_tfm_start

The sample supports using :ref:`Trusted Firmware-M <ug_tfm>` on the nRF54L15 DK.
The memory map of the sample has been aligned to meet the :ref:`ug_tfm_partition_alignment_requirements`.

You can build the sample with Trusted Firmware-M support by adding the ``ns`` suffix to the ``nrf54l15dk/nrf54l15/cpuapp`` board target.

For example:

.. code-block:: console

    west build -p -b nrf54l15dk/nrf54l15/cpuapp/ns

.. matter_template_build_with_tfm_end

.. |Bluetooth| replace:: Bluetooth

.. include:: /includes/advanced_conf_matter.txt

Matter occupancy sensor using only internal memory
===================================================

For the nRF54L15 DK, you can configure the sample to use only the internal RRAM for storage.
It applies to the DFU as well, which means that both the currently running firmware and the new firmware to be updated will be stored within the device's internal RRAM memory.
See the Device Firmware Upgrade support section above for information about the DFU process.

The DFU image can fit in the internal flash memory thanks to the usage of :ref:`MCUboot image compression<ug_matter_device_bootloader_image_compression>`.

This configuration is disabled by default for the Matter occupancy sensor sample.
To enable it, set the ``FILE_SUFFIX`` CMake option to ``internal``.

The following is an example command to build the sample for the nRF54L15 DK with support for Matter OTA DFU and DFU over Bluetooth SMP, and using internal RRAM only:

.. code-block:: console

    west build -p -b nrf54l15dk/nrf54l15/cpuapp -- -DCONFIG_CHIP_DFU_OVER_BT_SMP=y -DFILE_SUFFIX=internal

To build the sample for the same purpose, but in the ``release`` configuration, use the following command:

.. code-block:: console

    west build -p -b nrf54l15dk/nrf54l15/cpuapp -- -DCONFIG_CHIP_DFU_OVER_BT_SMP=y -DFILE_SUFFIX=internal -DEXTRA_CONF_FILE=prj_release.conf

In this case, the size of the MCUboot secondary partition used for storing the new application image is approximately 30%-40% smaller than it would be when using a configuration with external flash memory support.

User interface
**************

LED 0:
   .. include:: /includes/matter_sample_state_led.txt

LED 1:
   * **Solid ON**: Channel Sounding is in non-preemptive mode (default)
   * **Blinking** (1 second interval): Channel Sounding is in preemptive mode

Button 0:
   .. include:: /includes/matter_sample_button.txt

Button 1:
   Press to toggle Channel Sounding mode between non-preemptive (default) and preemptive.
   The current mode is indicated by LED 1:
   * Non-preemptive mode: LED 1 is solid ON
   * Preemptive mode: LED 1 blinks every 1 second

   In preemptive mode, Channel Sounding procedures reconnect to last known devices with higher priority.
   In non-preemptive mode, devices are scanned and connected in the order they were last paired.

.. include:: /includes/matter_segger_usb.txt


Building and running
********************

.. |sample path| replace:: :file:`samples/matter/template`

.. include:: /includes/build_and_run.txt

.. |sample_or_app| replace:: sample
.. |ipc_radio_dir| replace:: :file:`sysbuild/ipc_radio`

.. include:: /includes/ipc_radio_conf.txt

Selecting a configuration
=========================

Before you start testing the application, you can select one of the :ref:`matter_template_custom_configs`.
See :ref:`app_build_file_suffixes` and :ref:`cmake_options` for more information how to select a configuration.

Testing with Reflector Devices
===============================

To test the RF Sensing occupancy detection, you need one or more devices running the :ref:`channel_sounding_ras_reflector` sample.
The reflector devices must be paired with the occupancy sensor device through Matter commissioning.
Each paired reflector device will be assigned a unique Matter endpoint for occupancy detection.

The occupancy sensor will periodically perform Channel Sounding procedures to measure distances to all paired reflector devices.
When a reflector is detected within the configured occupancy threshold distance, the corresponding Matter endpoint is marked as occupied.

Testing
=======

When you have built the sample and programmed it to your development kit, it automatically starts the Bluetooth LE advertising and the **LED 0** starts flashing (Short Flash On).
At this point, you can press **Button 0** for six seconds to initiate the factory reset of the device.

.. _matter_template_network_testing:

Testing in a network
--------------------

To test the sample in a Matter-enabled Thread network, complete the following steps:

1. |connect_kit|
#. |connect_terminal_ANSI|
#. Commission the device into a Matter network by following the guides linked on the :ref:`ug_matter_configuring` page for the Matter controller you want to use.
   The guides walk you through the following steps:

   * Configure the Thread Border Router.
   * Build and install the Matter controller.
   * Commission the device.
     You can use the :ref:`matter_template_network_mode_onboarding` listed earlier on this page.
   * Send Matter commands.

   At the end of this procedure, **LED 0** of the Matter device programmed with the sample starts flashing in the Short Flash Off state.
   This indicates that the device is fully provisioned, but does not yet have full IPv6 network connectivity.
#. Keep the **Button 0** pressed for more than six seconds to initiate factory reset of the device.

   The device reboots after all its settings are erased.

Upgrading the device firmware
=============================

To upgrade the device firmware, complete the steps listed for the selected method in the :doc:`matter:nrfconnect_examples_software_update` tutorial of the Matter documentation.

Dependencies
************

This sample uses the Matter library that includes the |NCS| platform integration layer:

* `Matter`_

In addition, the sample uses the following |NCS| components:

* :ref:`dk_buttons_and_leds_readme`
* :ref:`Bluetooth Channel Sounding <ug_ble_channel_sounding>` - For RF Sensing occupancy detection
* :ref:`RAS (Ranging and Sensing) service <ug_ble_channel_sounding_ras>` - For Channel Sounding distance measurements

The sample depends on the following Zephyr libraries:

* :ref:`zephyr:logging_api`
* :ref:`zephyr:kernel_api`
* :ref:`zephyr:bluetooth_api` - For Channel Sounding functionality
